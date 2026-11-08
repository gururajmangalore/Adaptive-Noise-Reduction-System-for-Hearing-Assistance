// =============================================================================
// Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
// Module Name:     adaptive_noise_canceller.v
// Description:     16-bit (Q1.15) 8-tap synthesizable LMS Adaptive Filter for
//                  hearing assistance and vocal clarity in simulated microgravity
//                  and magnetic levitation environments.
//
// DSP Algorithm:
//   - Filter Topology: 8-tap FIR filter computing interference estimate y(n).
//   - Error Subtraction: e(n) = d(n) - y(n) (isolates speech, cancels noise).
//   - Adaptation Rule: Stochastic Gradient Descent (LMS):
//         w_k(n+1) = w_k(n) + mu * e(n) * x(n-k)
//
// Key Arithmetic & FPGA Implementation Details:
//   - Q1.15 signed fixed-point audio samples (range [-1.0, +0.999969]).
//   - 32-bit Q1.31 internal coefficient registers preventing deadband stalls.
//   - Hardware anti-wrap saturation on error subtraction and weight accumulators.
//   - Symmetric half-LSB rounding (+0x4000) before Q2.30 to Q1.15 truncation.
//
// Standard:        IEEE 1364-2001 (Verilog-2001 synthesizable)
// =============================================================================

`timescale 1ns / 1ps

module adaptive_noise_canceller #(
    parameter NUM_TAPS    = 8,   // 8-tap FIR adaptive filter
    parameter DATA_WIDTH  = 16,  // Audio sample bit width (Q1.15 signed)
    parameter COEFF_WIDTH = 32,  // Internal weight accumulator precision (Q1.31)
    parameter STEP_SHIFT  = 8    // Exponent right-shift scaling for step size mu
)(
    input  wire                   clk,             // System clock (e.g., 50 MHz - 100 MHz)
    input  wire                   rst_n,           // Active-low asynchronous/synchronous reset
    input  wire                   sample_valid,    // Sampling strobe: 1-cycle pulse per audio sample
    input  wire signed [15:0]     primary_speech,  // Primary mic: Speech + Microgravity Ambient Noise d(n)
    input  wire signed [15:0]     reference_noise, // Secondary mic: Ambient MagLev/Pump Noise x(n)
    input  wire        [7:0]      mu,              // LMS step-size parameter (convergence speed)
    output reg  signed [15:0]     cleaned_audio,   // Filtered audio signal output: e(n) = d(n) - y(n)
    output reg  signed [15:0]     error_signal,    // Residual error / noise estimate
    output reg  signed [15:0]     noise_estimate,  // Reconstructed ambient interference y(n)
    output reg                    out_valid        // Output valid strobe (1 cycle latency)
);

    // -------------------------------------------------------------------------
    // 1. Reference Noise Sample Delay Line
    // Holds historical samples: x(n), x(n-1), ..., x(n-7)
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] x_delay [0:NUM_TAPS-1];
    integer tap_idx;

    // -------------------------------------------------------------------------
    // 2. High-Precision 32-bit Coefficient Accumulators (Q1.31)
    // Top 16 bits [31:16] act as Q1.15 weights for the FIR multiplier.
    // Lower 16 bits [15:0] accumulate sub-LSB gradient updates without stalling.
    // -------------------------------------------------------------------------
    reg signed [COEFF_WIDTH-1:0] w_acc [0:NUM_TAPS-1];

    // Pipeline alignment register for primary microphone speech d(n)
    reg signed [DATA_WIDTH-1:0] primary_d1;

    // -------------------------------------------------------------------------
    // 3. Pipeline Stage 1: Parallel Multiplication (DSP Mapping)
    // Q1.15 * Q1.15 = Q2.30 (32 bits signed)
    // -------------------------------------------------------------------------
    reg signed [31:0] mult_stage [0:NUM_TAPS-1];
    reg               valid_stage1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                x_delay[tap_idx]    <= 16'sh0000;
                mult_stage[tap_idx] <= 32'sd0;
            end
            primary_d1   <= 16'sh0000;
            valid_stage1 <= 1'b0;
        end else begin
            valid_stage1 <= sample_valid;

            if (sample_valid) begin
                // Latch primary microphone sample for phase-aligned subtraction
                primary_d1 <= primary_speech;

                // Shift new reference sample into tapped delay line
                x_delay[0] <= reference_noise;
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    x_delay[tap_idx] <= x_delay[tap_idx - 1];
                end

                // Multiply new sample and historical taps with current weights
                // $signed() is mandatory on bit-slices in Verilog to prevent unsigned promotion
                mult_stage[0] <= reference_noise * $signed(w_acc[0][31:16]);
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    mult_stage[tap_idx] <= x_delay[tap_idx - 1] * $signed(w_acc[tap_idx][31:16]);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. Pipeline Stage 2: Adder Tree, Rounding & 16-bit Saturation
    // Sums 8 products in 40-bit accumulator with 8 guard bits.
    // -------------------------------------------------------------------------
    reg signed [39:0] y_acc;
    reg signed [39:0] y_round;
    reg signed [15:0] y_sat;
    reg signed [16:0] e_diff_raw;
    reg signed [15:0] e_sat;
    integer acc_idx;

    always @(*) begin
        // Accumulate 8 parallel DSP products
        y_acc = 40'sd0;
        for (acc_idx = 0; acc_idx < NUM_TAPS; acc_idx = acc_idx + 1) begin
            y_acc = y_acc + mult_stage[acc_idx];
        end

        // Symmetric half-LSB rounding: add 2^14 (0x4000)
        y_round = y_acc + 40'sh00_0000_4000;

        // Scale down Q2.30 to Q1.15 via signed arithmetic shift (>>> 15)
        // Saturate to 16-bit signed range [-32768, +32767]
        if ((y_round >>> 15) > 40'sd32767) begin
            y_sat = 16'sh7FFF; // Positive saturation (+32767)
        end else if ((y_round >>> 15) < -40'sd32768) begin
            y_sat = 16'sh8000; // Negative saturation (-32768)
        end else begin
            y_sat = y_round[30:15]; // Exact Q1.15 slice
        end

        // Error Subtraction: e(n) = d(n) - y(n)
        e_diff_raw = $signed({primary_d1[15], primary_d1}) - $signed({y_sat[15], y_sat});

        // Saturation logic for error signal
        if (e_diff_raw > 17'sd32767) begin
            e_sat = 16'sh7FFF;
        end else if (e_diff_raw < -17'sd32768) begin
            e_sat = 16'sh8000;
        end else begin
            e_sat = e_diff_raw[15:0];
        end
    end

    // -------------------------------------------------------------------------
    // 5. Output Registration & LMS Coefficient Adaptation
    // Equation: w_k(n+1) = w_k(n) + mu * e(n) * x(n-k)
    // -------------------------------------------------------------------------
    reg signed [31:0]          grad_prod;
    reg signed [40:0]          scaled_grad;
    reg signed [COEFF_WIDTH:0] delta_w;
    reg signed [COEFF_WIDTH:0] sum_w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cleaned_audio  <= 16'sh0000;
            error_signal   <= 16'sh0000;
            noise_estimate <= 16'sh0000;
            out_valid      <= 1'b0;

            for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                w_acc[tap_idx] <= {COEFF_WIDTH{1'b0}};
            end
        end else begin
            out_valid <= valid_stage1;

            if (valid_stage1) begin
                // Register primary outputs
                cleaned_audio  <= e_sat;
                error_signal   <= e_sat;
                noise_estimate <= y_sat;

                // Update all 8 filter tap weights in parallel
                for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    // Stochastic gradient: e(n) * x(n-k) (32-bit signed Q2.30)
                    grad_prod = e_sat * x_delay[tap_idx];

                    // Scale by unsigned 8-bit parameter mu
                    scaled_grad = $signed({1'b0, mu}) * grad_prod;

                    // Calibrated right shift to scale gradient step into Q1.31
                    delta_w = scaled_grad >>> STEP_SHIFT;

                    // 33-bit addition for overflow detection
                    sum_w = $signed({w_acc[tap_idx][COEFF_WIDTH-1], w_acc[tap_idx]}) + delta_w;

                    // 32-bit Anti-Windup Saturation
                    if (sum_w > 33'sd2147483647) begin
                        w_acc[tap_idx] <= 32'sh7FFF_FFFF;
                    end else if (sum_w < -33'sd2147483648) begin
                        w_acc[tap_idx] <= 32'sh8000_0000;
                    end else begin
                        w_acc[tap_idx] <= sum_w[COEFF_WIDTH-1:0];
                    end
                end
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule

