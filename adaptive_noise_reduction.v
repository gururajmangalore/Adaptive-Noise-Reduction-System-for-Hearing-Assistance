// =============================================================================
// Project:     Adaptive Noise Reduction System for Hearing Assistance
// Module Name: adaptive_noise_reduction
// Description: Synthesizable Least Mean Squares (LMS) Adaptive FIR Filter for
//              real-time acoustic noise cancellation and speech intelligibility
//              enhancement in hearing aid / assistive audio hardware.
//
// LMS Formulation:
//   x(n)       = ref_noise (reference ambient acoustic noise)
//   d(n)       = noisy_in  (primary microphone: speech s(n) + acoustic noise n_0(n))
//   y(n)       = sum_{k=0}^{TAPS-1} (w_k(n) * x(n-k))   [FIR noise estimate]
//   e(n)       = d(n) - y(n) = clean_out                [Cleaned speech output]
//   w_k(n+1)   = w_k(n) + mu * e(n) * x(n-k)            [Gradient descent update]
//
// Fixed-Point & Precision Details:
//   - Audio samples: 16-bit signed Q1.15 [-1.0, +0.999969]
//   - Filter weights: 32-bit signed Q1.31 accumulators (upper 16 bits used as tap weights)
//   - Step size mu: Hardware-friendly power-of-two arithmetic right shift (MU_SHIFT)
//   - Saturation arithmetic on all accumulators and outputs prevents clipping wrap-around.
// =============================================================================

`timescale 1ns / 1ps

`ifndef ADAPTIVE_NOISE_REDUCTION_V
`define ADAPTIVE_NOISE_REDUCTION_V

module adaptive_noise_reduction #(
    parameter TAPS       = 8, // Number of adaptive filter taps (configurable: 4, 8, 16)
    parameter DATA_WIDTH = 16, // Audio sample width (Q1.15 signed)
    parameter COEFF_W    = 32, // Internal weight accumulator bit-width (Q1.31)
    parameter MU_SHIFT   = 5   // LMS step size factor: mu = 2^(-MU_SHIFT)
)(
    input  wire                          clk,       // System clock
    input  wire                          rst_n,     // Asynchronous active-low reset
    input  wire signed [DATA_WIDTH-1:0]  noisy_in,  // Primary mic input: speech + noise d(n)
    input  wire signed [DATA_WIDTH-1:0]  ref_noise, // Reference mic input: correlated noise x(n)
    output reg  signed [DATA_WIDTH-1:0]  clean_out  // Filtered speech output: e(n) = d(n) - y(n)
);

    // -------------------------------------------------------------------------
    // Internal Registers and Data Paths
    // -------------------------------------------------------------------------
    // History delay line for previous reference noise samples x(n-1) to x(n-TAPS+1)
    reg signed [DATA_WIDTH-1:0] x_delay [0:TAPS-2];

    // Adaptive filter tap weights (32-bit Q1.31 accumulators)
    reg signed [COEFF_W-1:0]    weights [0:TAPS-1];

    // Current tapped delay line vector x_vec: x(n), x(n-1), ..., x(n-TAPS+1)
    wire signed [DATA_WIDTH-1:0] x_vec [0:TAPS-1];
    assign x_vec[0] = ref_noise;
    genvar g;
    generate
        for (g = 1; g < TAPS; g = g + 1) begin : gen_x_vec
            assign x_vec[g] = x_delay[g-1];
        end
    endgenerate

    // Intermediate variables for calculation
    reg signed [47:0]           fir_acc;
    reg signed [DATA_WIDTH-1:0] y_estimate;
    reg signed [DATA_WIDTH:0]   err_unclamped; // 17-bit for overflow detection
    reg signed [DATA_WIDTH-1:0] err_clamped;
    reg signed [31:0]           grad_prod;
    reg signed [31:0]           weight_delta;
    reg signed [32:0]           weight_sum;    // 33-bit for overflow detection

    integer i;

    // -------------------------------------------------------------------------
    // Synchronous Pipeline: FIR Filtering, Error Extraction & LMS Weight Update
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset delay line, tap weights, and output
            clean_out <= {DATA_WIDTH{1'b0}};
            for (i = 0; i < TAPS - 1; i = i + 1) begin
                x_delay[i] <= {DATA_WIDTH{1'b0}};
            end
            for (i = 0; i < TAPS; i = i + 1) begin
                weights[i] <= {COEFF_W{1'b0}};
            end
        end else begin
            // -----------------------------------------------------------------
            // Step 1: FIR Filtering (Compute estimated interference y(n))
            // -----------------------------------------------------------------
            fir_acc = 48'sd0;
            for (i = 0; i < TAPS; i = i + 1) begin
                fir_acc = fir_acc + ($signed(weights[i][COEFF_W-1:COEFF_W-DATA_WIDTH]) * $signed(x_vec[i]));
            end

            // Check for positive and negative saturation/overflow in Q2.30 -> Q1.15
            if (fir_acc > 48'sd1073725440) begin // Max positive clamp (+32767 in Q2.30)
                y_estimate = 16'sh7FFF;
            end else if (fir_acc < -48'sd1073741824) begin // Min negative clamp (-32768 in Q2.30)
                y_estimate = -16'sh8000;
            end else begin
                // Rounding addition before right-shifting by 15 bits
                y_estimate = (fir_acc + 48'sd16384) >>> 15;
            end

            // -----------------------------------------------------------------
            // Step 2: Error Calculation with Clamping (Speech Signal Extraction)
            // e(n) = noisy_in(n) - y_estimate(n)
            // -----------------------------------------------------------------
            err_unclamped = $signed(noisy_in) - $signed(y_estimate);

            if (err_unclamped > 17'sd32767) begin
                err_clamped = 16'sh7FFF; // Clamp to max positive Q1.15
            end else if (err_unclamped < -17'sd32768) begin
                err_clamped = -16'sh8000; // Clamp to min negative Q1.15
            end else begin
                err_clamped = err_unclamped[15:0];
            end

            // Register clean speech output
            clean_out <= err_clamped;

            // -----------------------------------------------------------------
            // Step 3: Shift Register Update
            // -----------------------------------------------------------------
            x_delay[0] <= ref_noise;
            for (i = 1; i < TAPS - 1; i = i + 1) begin
                x_delay[i] <= x_delay[i-1];
            end

            // -----------------------------------------------------------------
            // Step 4: LMS Tap Weight Update
            // w_k(n+1) = w_k(n) + mu * e(n) * x(n-k)
            // -----------------------------------------------------------------
            for (i = 0; i < TAPS; i = i + 1) begin
                grad_prod = $signed(err_clamped) * $signed(x_vec[i]);
                weight_delta = grad_prod >>> MU_SHIFT;
                weight_sum = $signed({weights[i][COEFF_W-1], weights[i]}) + $signed({weight_delta[31], weight_delta});

                // Weight saturation clamping (32-bit Q1.31)
                if (weight_sum > 33'sh0_7FFF_FFFF) begin
                    weights[i] <= 32'sh7FFF_FFFF;
                end else if (weight_sum < -33'sh0_8000_0000) begin
                    weights[i] <= -32'sh8000_0000;
                end else begin
                    weights[i] <= weight_sum[31:0];
                end
            end
        end
    end

endmodule

`endif // ADAPTIVE_NOISE_REDUCTION_V
