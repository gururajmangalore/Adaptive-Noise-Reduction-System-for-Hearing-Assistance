// =============================================================================
// Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
// Module Name:     lms_weight_updater
// Description:     Synthesizable LMS Weight Update Engine. Dynamically adapts
//                  filter weights using the stochastic gradient descent equation:
//                      w_k(n+1) = w_k(n) + mu * e(n) * x(n-k)
//                  Features 32-bit high-precision accumulation (Q1.31) to prevent
//                  gradient limit cycles and truncation stall, combined with
//                  saturating arithmetic to eliminate two's-complement overflow.
// Standard:        IEEE 1364-2001 (Verilog-2001)
// =============================================================================

`timescale 1ns / 1ps

module lms_weight_updater #(
    parameter NUM_TAPS    = 16, // Number of filter taps
    parameter DATA_WIDTH  = 16, // Audio sample width (Q1.15)
    parameter COEFF_WIDTH = 32, // Internal coefficient precision (Q1.31)
    parameter STEP_SHIFT  = 10  // Calibrated exponent shift for step size mu
)(
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               update_enable,    // Strobe when e(n) is valid
    input  wire        [7:0]                  mu,               // Convergence step size [0..255]
    input  wire signed [DATA_WIDTH-1:0]       error_in,         // Residual error signal e(n) in Q1.15
    input  wire        [NUM_TAPS*DATA_WIDTH-1:0] x_delay_bus,   // Delay line samples x(n-k)
    output wire        [NUM_TAPS*DATA_WIDTH-1:0] coeff_weights, // Upper 16 bits of weights (Q1.15)
    output reg                                overflow_flag     // Status telemetry flag
);

    // -------------------------------------------------------------------------
    // High-Precision 32-bit Weight Accumulator Registers (Q1.31)
    // Bit 31: Sign bit
    // Bits [30:15]: Top 16 bits fed to FIR multiplier as Q1.15 coefficient
    // Bits [14:0]: Fractional accumulator bits capturing sub-LSB gradient steps
    // -------------------------------------------------------------------------
    reg signed [COEFF_WIDTH-1:0] w_acc [0:NUM_TAPS-1];
    integer tap_idx;

    // Export the top 16 bits (Q1.15) of each 32-bit accumulator to the FIR core
    genvar g;
    generate
        for (g = 0; g < NUM_TAPS; g = g + 1) begin : gen_coeff_export
            assign coeff_weights[(g+1)*DATA_WIDTH-1 : g*DATA_WIDTH] = 
                w_acc[g][COEFF_WIDTH-1 : COEFF_WIDTH-DATA_WIDTH];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Parallel Adaptation Logic
    // For each tap k:
    //   1. Gradient = e(n) * x(n-k)  -> 32-bit signed (Q2.30)
    //   2. Scale    = mu * Gradient   -> 41-bit signed
    //   3. Shift    = Scale >>> STEP_SHIFT
    //   4. Saturated 32-bit Accumulate: w_acc[k] <= sat32(w_acc[k] + delta)
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] current_x;
    reg signed [31:0]           grad_prod;
    reg signed [40:0]           scaled_grad;
    reg signed [COEFF_WIDTH:0]  delta_w;
    reg signed [COEFF_WIDTH:0]  sum_w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                w_acc[tap_idx] <= {COEFF_WIDTH{1'b0}};
            end
            overflow_flag <= 1'b0;
        end else begin
            overflow_flag <= 1'b0; // Pulse flag on event

            if (update_enable) begin
                for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    // Extract tap sample x(n-k) from flattened bus using indexed part-select
                    current_x = $signed(x_delay_bus[tap_idx*DATA_WIDTH +: DATA_WIDTH]);

                    // Step 1: Stochastic gradient calculation
                    grad_prod = error_in * current_x;

                    // Step 2: Multiplication by 8-bit unsigned step size mu
                    scaled_grad = $signed({1'b0, mu}) * grad_prod;

                    // Step 3: Arithmetic right shift by STEP_SHIFT parameter
                    delta_w = scaled_grad >>> STEP_SHIFT;

                    // Step 4: 33-bit addition with saturation detection
                    sum_w = $signed({w_acc[tap_idx][COEFF_WIDTH-1], w_acc[tap_idx]}) + delta_w;

                    // Saturation logic for signed 32-bit fixed point:
                    // Max positive: +2,147,483,647 (0x7FFFFFFF)
                    // Min negative: -2,147,483,648 (0x80000000)
                    if (sum_w > 33'sd2147483647) begin
                        w_acc[tap_idx] <= 32'sh7FFF_FFFF;
                        overflow_flag  <= 1'b1;
                    end else if (sum_w < -33'sd2147483648) begin
                        w_acc[tap_idx] <= 32'sh8000_0000;
                        overflow_flag  <= 1'b1;
                    end else begin
                        w_acc[tap_idx] <= sum_w[COEFF_WIDTH-1:0];
                    end
                end
            end
        end
    end

endmodule

