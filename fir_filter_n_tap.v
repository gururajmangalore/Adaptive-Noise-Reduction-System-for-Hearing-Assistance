
 // =============================================================================
// Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
// Module Name:     fir_filter_n_tap
// Description:     Synthesizable N-tap FIR filter core designed for Q1.15 fixed-
//                  point audio filtering. Operates with parallel multiply-
//                  accumulate hardware mapped to FPGA DSP slices (e.g., DSP48E).
//                  Features internal half-LSB rounding and saturation clipping.
// Standard:        IEEE 1364-2001 (Verilog-2001)
// =============================================================================

`timescale 1ns / 1ps

module fir_filter_n_tap #(
    parameter NUM_TAPS   = 16, // Number of filter taps
    parameter DATA_WIDTH = 16  // Audio sample bit width (Q1.15)
)(
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               sample_valid,
    input  wire signed [DATA_WIDTH-1:0]       sample_in,       // Reference noise x(n)
    input  wire        [NUM_TAPS*DATA_WIDTH-1:0] coeff_weights, // Flattened 16-bit Q1.15 coefficients
    output reg  signed [DATA_WIDTH-1:0]       filter_out,      // Filtered noise estimate y(n) in Q1.15
    output reg                                filter_valid,    // Output strobe (valid 1 cycle after sample)
    output wire        [NUM_TAPS*DATA_WIDTH-1:0] x_delay_bus   // Exported sample delay line for LMS update
);

    // -------------------------------------------------------------------------
    // Reference Sample Delay Line Register Array
    // Holds x(n), x(n-1), ..., x(n - NUM_TAPS + 1)
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] x_pipe [0:NUM_TAPS-1];
    integer tap_idx;

    // Bus export: slice the internal delay line into a flattened wire bus
    genvar g;
    generate
        for (g = 0; g < NUM_TAPS; g = g + 1) begin : gen_delay_export
            assign x_delay_bus[(g+1)*DATA_WIDTH-1 : g*DATA_WIDTH] = x_pipe[g];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pipeline Stage 1: Parallel Multiplication (DSP Mapping)
    // Product format: Q1.15 * Q1.15 = Q2.30 (32 bits signed)
    // -------------------------------------------------------------------------
    reg signed [31:0] mult_stage [0:NUM_TAPS-1];
    reg               valid_stage1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                x_pipe[tap_idx]     <= {DATA_WIDTH{1'b0}};
                mult_stage[tap_idx] <= 32'sd0;
            end
            valid_stage1 <= 1'b0;
        end else begin
            valid_stage1 <= sample_valid;

            if (sample_valid) begin
                // Shift delay line
                x_pipe[0] <= sample_in;
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    x_pipe[tap_idx] <= x_pipe[tap_idx - 1];
                end

                // Multiply: new sample x(n) and historical samples with current weights
                // coeff_weights is sliced into individual signed 16-bit Q1.15 values
                mult_stage[0] <= sample_in * $signed(coeff_weights[0 +: DATA_WIDTH]);
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    mult_stage[tap_idx] <= x_pipe[tap_idx - 1] * 
                        $signed(coeff_weights[tap_idx*DATA_WIDTH +: DATA_WIDTH]);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Stage 2: Adder Tree Accumulation, Rounding & Saturation
    // Accumulator width: 32 bits + 8 guard bits = 40 bits signed
    // -------------------------------------------------------------------------
    reg signed [39:0] y_acc;
    reg signed [39:0] y_round;
    integer acc_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filter_out   <= {DATA_WIDTH{1'b0}};
            filter_valid <= 1'b0;
        end else begin
            filter_valid <= valid_stage1;

            if (valid_stage1) begin
                // Sum all DSP products
                y_acc = 40'sd0;
                for (acc_idx = 0; acc_idx < NUM_TAPS; acc_idx = acc_idx + 1) begin
                    y_acc = y_acc + mult_stage[acc_idx];
                end

                // Apply half-LSB rounding before truncation: add 2^14 (0x4000)
                y_round = y_acc + 40'sh00_0000_4000;

                // Scale down from Q2.30 to Q1.15 by arithmetic shifting right by 15
                // Check for positive/negative overflow beyond 16-bit signed range [-32768, +32767]
                // Note: Using arithmetic shift (>>>) preserves signedness across bit slices
                if ((y_round >>> 15) > 40'sd32767) begin
                    filter_out <= 16'sh7FFF; // Positive saturation (+32767)
                end else if ((y_round >>> 15) < -40'sd32768) begin
                    filter_out <= 16'sh8000; // Negative saturation (-32768)
                end else begin
                    filter_out <= y_round[30:15]; // Exact Q1.15 slice
                end
            end else begin
                filter_valid <= 1'b0;
            end
        end
    end

endmodule
