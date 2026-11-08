// =============================================================================
// Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
// Module Name:     adaptive_noise_reduction_top
// Description:     Top-Level Synthesizable Adaptive Noise Reduction System
//                  designed for crew hearing assistance and mission-critical voice
//                  intelligibility in simulated microgravity/antigravity habitats.
//
// Operational Context:
//   - Background Interference: High-frequency acoustic resonance from fluid pump
//     loops (cryogenic/cooling), active magnetic levitation control hum (1 kHz -
//     4 kHz PWM/bearing switching noise), and microgravity acoustic dispersion.
//   - Target Signal: Astronaut vocal communications captured by primary mic array.
//   - Cancellation Topology: Adaptive FIR filter using Normalized/Standard Least
//     Mean Squares (LMS) stochastic gradient descent. The secondary reference
//     sensor captures the correlated ambient electromagnetic/acoustic interference.
//     The filter dynamically reconstructs the acoustic coupling path y(n) ≈ n_0(n).
//     Subtracting y(n) from primary speech d(n) yields the cleaned audio e(n) = s(n).
//
// Key Features:
//   - Q1.15 fixed-point signed arithmetic (16-bit audio precision).
//   - 32-bit Q1.31 internal coefficient registers preventing deadband quantization.
//   - Fully pipelined architecture (Stage 0 -> Stage 1 -> Stage 2) for Fmax > 150 MHz.
//   - Hardware saturation arithmetic on all subtractors and accumulators.
//
// Standard:        IEEE 1364-2001 (Verilog-2001)
// =============================================================================

`timescale 1ns / 1ps

module adaptive_noise_reduction_top #(
    parameter NUM_TAPS    = 16, // Number of adaptive FIR filter taps
    parameter DATA_WIDTH  = 16, // Audio sample format width (Q1.15 signed)
    parameter COEFF_WIDTH = 32, // Internal weight accumulator precision (Q1.31)
    parameter STEP_SHIFT  = 10  // Exponent right-shift factor for step size mu
)(
    // Clocking & Control
    input  wire                   clk,             // System master clock (e.g., 50 MHz - 100 MHz)
    input  wire                   rst_n,           // Asynchronous active-low reset
    input  wire                   sample_valid,    // Sampling strobe: 1-cycle pulse per audio sample

    // Audio Microphone Inputs
    input  wire signed [15:0]     primary_speech,  // Primary mic: Speech + Microgravity Acoustic Noise d(n)
    input  wire signed [15:0]     reference_noise, // Secondary mic: Correlated levitation/pump noise x(n)
    input  wire        [7:0]      mu,              // LMS convergence step-size parameter

    // Processed Audio Outputs
    output reg  signed [15:0]     cleaned_audio,   // Filtered audio signal output: e(n) = d(n) - y(n)
    output reg  signed [15:0]     error_signal,    // Residual error / noise estimate
    output reg  signed [15:0]     noise_estimate,  // Filtered interference reconstruction y(n)
    output reg                    out_valid,       // Output sample strobe
    output wire                   overflow_alert   // Telemetry alert for saturation/clipping events
);

    // -------------------------------------------------------------------------
    // Internal Interconnect Busses & Control Signals
    // -------------------------------------------------------------------------
    wire [NUM_TAPS*DATA_WIDTH-1:0] coeff_weights_bus;
    wire [NUM_TAPS*DATA_WIDTH-1:0] x_delay_bus;
    wire signed [DATA_WIDTH-1:0]   fir_y_out;
    wire                           fir_y_valid;
    wire                           updater_overflow;

    // -------------------------------------------------------------------------
    // Pipeline Alignment Delay Register for Primary Mic Speech d(n)
    // The FIR filter core has a 1-cycle latency from sample_valid to fir_y_valid.
    // To maintain phase alignment, d(n) is registered across the same delay.
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] primary_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            primary_d1 <= 16'sh0000;
        end else if (sample_valid) begin
            primary_d1 <= primary_speech;
        end
    end

    // -------------------------------------------------------------------------
    // Submodule 1: N-Tap FIR Filter Core (Estimates Interference y(n))
    // -------------------------------------------------------------------------
    fir_filter_n_tap #(
        .NUM_TAPS   (NUM_TAPS),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_fir_filter (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_valid (sample_valid),
        .sample_in    (reference_noise),
        .coeff_weights(coeff_weights_bus),
        .filter_out   (fir_y_out),
        .filter_valid (fir_y_valid),
        .x_delay_bus  (x_delay_bus)
    );

    // -------------------------------------------------------------------------
    // Error Calculation & Saturation Logic
    // e(n) = d(n) - y(n)
    // Both d(n) and y(n) are 16-bit signed Q1.15 numbers.
    // Their difference can reach [-65535, +65534], requiring a 17-bit subtractor.
    // Saturation clamps the result to signed 16-bit Q1.15 [-32768, +32767].
    // -------------------------------------------------------------------------
    reg signed [16:0] e_diff_raw;
    reg signed [15:0] e_saturated;
    reg               diff_overflow;

    always @(*) begin
        e_diff_raw = $signed({primary_d1[15], primary_d1}) - $signed({fir_y_out[15], fir_y_out});

        if (e_diff_raw > 17'sd32767) begin
            e_saturated   = 16'sh7FFF; // Positive saturation (+32767)
            diff_overflow = 1'b1;
        end else if (e_diff_raw < -17'sd32768) begin
            e_saturated   = 16'sh8000; // Negative saturation (-32768)
            diff_overflow = 1'b1;
        end else begin
            e_saturated   = e_diff_raw[15:0];
            diff_overflow = 1'b0;
        end
    end

    // Output registration
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cleaned_audio  <= 16'sh0000;
            error_signal   <= 16'sh0000;
            noise_estimate <= 16'sh0000;
            out_valid      <= 1'b0;
        end else begin
            out_valid <= fir_y_valid;
            if (fir_y_valid) begin
                cleaned_audio  <= e_saturated;
                error_signal   <= e_saturated;
                noise_estimate <= fir_y_out;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Submodule 2: Adaptive Weight Update Engine (LMS Algorithm)
    // -------------------------------------------------------------------------
    lms_weight_updater #(
        .NUM_TAPS    (NUM_TAPS),
        .DATA_WIDTH  (DATA_WIDTH),
        .COEFF_WIDTH (COEFF_WIDTH),
        .STEP_SHIFT  (STEP_SHIFT)
    ) u_weight_updater (
        .clk          (clk),
        .rst_n        (rst_n),
        .update_enable(fir_y_valid),
        .mu           (mu),
        .error_in     (e_saturated),
        .x_delay_bus  (x_delay_bus),
        .coeff_weights(coeff_weights_bus),
        .overflow_flag(updater_overflow)
    );

    // Combined telemetry alert
    assign overflow_alert = diff_overflow | updater_overflow;

endmodule

