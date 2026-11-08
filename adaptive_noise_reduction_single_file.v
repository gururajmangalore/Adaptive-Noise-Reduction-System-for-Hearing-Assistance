// =============================================================================
// Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
// Module/Package:  adaptive_noise_reduction_single_file.v
// Description:     Complete, Self-Contained Synthesizable Verilog HDL Package
//                  and Testbench for an Adaptive Noise Reduction System designed
//                  for astronaut hearing assistance in microgravity/antigravity
//                  environments.
//
// Contents:
//   1. fir_filter_n_tap            - N-tap FIR filter core with DSP mapping
//   2. lms_weight_updater          - 32-bit Q1.31 LMS adaptation engine
//   3. adaptive_noise_reduction_top - Top-level system integration
//   4. tb_adaptive_noise_reduction - Complete self-checking testbench
//
// Standard:        IEEE 1364-2001 (Verilog-2001)
// =============================================================================

`timescale 1ns / 1ps

// =============================================================================
// MODULE 1: fir_filter_n_tap
// =============================================================================
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

    reg signed [DATA_WIDTH-1:0] x_pipe [0:NUM_TAPS-1];
    integer tap_idx;

    genvar g;
    generate
        for (g = 0; g < NUM_TAPS; g = g + 1) begin : gen_delay_export
            assign x_delay_bus[(g+1)*DATA_WIDTH-1 : g*DATA_WIDTH] = x_pipe[g];
        end
    endgenerate

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
                x_pipe[0] <= sample_in;
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    x_pipe[tap_idx] <= x_pipe[tap_idx - 1];
                end

                // Multiply: new sample x(n) and historical samples with current weights
                // Using standard Verilog-2001 indexed part-select [base +: width]
                mult_stage[0] <= sample_in * $signed(coeff_weights[0 +: DATA_WIDTH]);
                for (tap_idx = 1; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    mult_stage[tap_idx] <= x_pipe[tap_idx - 1] * 
                        $signed(coeff_weights[tap_idx*DATA_WIDTH +: DATA_WIDTH]);
                end
            end
        end
    end

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
                y_acc = 40'sd0;
                for (acc_idx = 0; acc_idx < NUM_TAPS; acc_idx = acc_idx + 1) begin
                    y_acc = y_acc + mult_stage[acc_idx];
                end

                // Half-LSB rounding: add 2^14 (0x4000)
                y_round = y_acc + 40'sh00_0000_4000;

                // Signed arithmetic shift right by 15 with saturation
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

// =============================================================================
// MODULE 2: lms_weight_updater
// =============================================================================
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

    reg signed [COEFF_WIDTH-1:0] w_acc [0:NUM_TAPS-1];
    integer tap_idx;

    genvar g;
    generate
        for (g = 0; g < NUM_TAPS; g = g + 1) begin : gen_coeff_export
            assign coeff_weights[(g+1)*DATA_WIDTH-1 : g*DATA_WIDTH] = 
                w_acc[g][COEFF_WIDTH-1 : COEFF_WIDTH-DATA_WIDTH];
        end
    endgenerate

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
            overflow_flag <= 1'b0;

            if (update_enable) begin
                for (tap_idx = 0; tap_idx < NUM_TAPS; tap_idx = tap_idx + 1) begin
                    current_x = $signed(x_delay_bus[tap_idx*DATA_WIDTH +: DATA_WIDTH]);
                    grad_prod = error_in * current_x;
                    scaled_grad = $signed({1'b0, mu}) * grad_prod;
                    delta_w = scaled_grad >>> STEP_SHIFT;
                    sum_w = $signed({w_acc[tap_idx][COEFF_WIDTH-1], w_acc[tap_idx]}) + delta_w;

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

// =============================================================================
// MODULE 3: adaptive_noise_reduction_top
// =============================================================================
module adaptive_noise_reduction_top #(
    parameter NUM_TAPS    = 16, // Number of adaptive FIR filter taps
    parameter DATA_WIDTH  = 16, // Audio sample format width (Q1.15 signed)
    parameter COEFF_WIDTH = 32, // Internal weight accumulator precision (Q1.31)
    parameter STEP_SHIFT  = 10  // Exponent right-shift factor for step size mu
)(
    input  wire                   clk,             // System master clock
    input  wire                   rst_n,           // Asynchronous active-low reset
    input  wire                   sample_valid,    // Sampling strobe: 1-cycle pulse per audio sample
    input  wire signed [15:0]     primary_speech,  // Primary mic: Speech + Microgravity Noise d(n)
    input  wire signed [15:0]     reference_noise, // Secondary mic: Correlated reference noise x(n)
    input  wire        [7:0]      mu,              // LMS convergence step-size parameter
    output reg  signed [15:0]     cleaned_audio,   // Filtered audio signal output: e(n) = d(n) - y(n)
    output reg  signed [15:0]     error_signal,    // Residual error / noise estimate
    output reg  signed [15:0]     noise_estimate,  // Filtered interference reconstruction y(n)
    output reg                    out_valid,       // Output sample strobe
    output wire                   overflow_alert   // Telemetry alert for saturation/clipping events
);

    wire [NUM_TAPS*DATA_WIDTH-1:0] coeff_weights_bus;
    wire [NUM_TAPS*DATA_WIDTH-1:0] x_delay_bus;
    wire signed [DATA_WIDTH-1:0]   fir_y_out;
    wire                           fir_y_valid;
    wire                           updater_overflow;

    // Delay register to align primary speech with FIR output latency
    reg signed [DATA_WIDTH-1:0] primary_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            primary_d1 <= 16'sh0000;
        end else if (sample_valid) begin
            primary_d1 <= primary_speech;
        end
    end

    // FIR Filter Instance
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

    // Error Subtraction & Saturation: e(n) = d(n) - y(n)
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

    // Registered Audio Outputs
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

    // LMS Weight Updater Instance
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

    assign overflow_alert = diff_overflow | updater_overflow;

endmodule

// =============================================================================
// MODULE 4: tb_adaptive_noise_reduction (Testbench)
// =============================================================================
module tb_adaptive_noise_reduction;

    parameter CLK_PERIOD   = 20;     // 50 MHz Master Clock
    parameter NUM_TAPS     = 16;
    parameter DATA_WIDTH   = 16;
    parameter COEFF_WIDTH  = 32;
    parameter STEP_SHIFT   = 10;
    parameter TOTAL_SAMPLES= 2000;

    localparam real PI     = 3.14159265358979323846;
    localparam real FS     = 16000.0; // 16 kHz sample rate

    reg                     clk;
    reg                     rst_n;
    reg                     sample_valid;
    reg  signed [15:0]      primary_speech;
    reg  signed [15:0]      reference_noise;
    reg         [7:0]       mu;

    wire signed [15:0]      cleaned_audio;
    wire signed [15:0]      error_signal;
    wire signed [15:0]      noise_estimate;
    wire                    out_valid;
    wire                    overflow_alert;

    adaptive_noise_reduction_top #(
        .NUM_TAPS   (NUM_TAPS),
        .DATA_WIDTH (DATA_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .STEP_SHIFT (STEP_SHIFT)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .sample_valid   (sample_valid),
        .primary_speech (primary_speech),
        .reference_noise(reference_noise),
        .mu             (mu),
        .cleaned_audio  (cleaned_audio),
        .error_signal   (error_signal),
        .noise_estimate (noise_estimate),
        .out_valid      (out_valid),
        .overflow_alert (overflow_alert)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    integer sample_idx;
    real    time_sec;
    real    speech_real;
    real    noise_ref_real;
    real    noise_prim_real;
    real    prim_mic_real;

    real    x_d1, x_d2, x_d3;
    reg signed [15:0] speech_history [0:TOTAL_SAMPLES-1];

    real    init_noise_sq_sum;
    real    final_noise_sq_sum;
    real    init_rms;
    real    final_rms;
    real    attenuation_db;
    real    residual_noise;
    integer init_sample_count;
    integer final_sample_count;

    initial begin
        rst_n               = 1'b0;
        sample_valid        = 1'b0;
        primary_speech      = 16'sh0000;
        reference_noise     = 16'sh0000;
        mu                  = 8'd32;

        x_d1                = 0.0;
        x_d2                = 0.0;
        x_d3                = 0.0;
        init_noise_sq_sum   = 0.0;
        final_noise_sq_sum  = 0.0;
        init_sample_count   = 0;
        final_sample_count  = 0;

        $display("===============================================================================");
        $display(" AEROSPACE LIFE-SUPPORT ELECTRONICS - BIO-ACOUSTIC ADAPTIVE NOISE REDUCTION ");
        $display(" Simulation Environment: Microgravity Habitat / Magnetic Levitation Testing    ");
        $display(" Clock: 50 MHz | Fs: 16.0 kHz | Word Length: Q1.15 | Filter Taps: %0d taps    ", NUM_TAPS);
        $display(" LMS Step Size (mu): %0d | Exponent Shift: %0d", mu, STEP_SHIFT);
        $display("===============================================================================");

        #(CLK_PERIOD * 10);
        @(posedge clk);
        rst_n = 1'b1;
        #(CLK_PERIOD * 5);

        $display("\n[T = %0t ns] Reset released. Starting audio sample injection...", $time);
        $display("-------------------------------------------------------------------------------");
        $display("  Sample # | Time(ms) | PrimMic d(n) | RefNoise x(n) | NoiseEst y(n) | Cleaned e(n) ");
        $display("-------------------------------------------------------------------------------");

        for (sample_idx = 0; sample_idx < TOTAL_SAMPLES; sample_idx = sample_idx + 1) begin
            time_sec = sample_idx / FS;

            // 440 Hz Speech
            speech_real = 4500.0 * $sin(2.0 * PI * 440.0 * time_sec);
            speech_history[sample_idx] = $rtoi(speech_real);

            // 2400 Hz MagLev + 3600 Hz Pump Noise
            noise_ref_real = 8000.0 * $sin(2.0 * PI * 2400.0 * time_sec) +
                             4500.0 * $sin(2.0 * PI * 3600.0 * time_sec);

            // Acoustic path H(z) = 0.50 + 0.25*z^-1 - 0.15*z^-2 + 0.08*z^-3
            noise_prim_real = 0.50 * noise_ref_real +
                              0.25 * x_d1 -
                              0.15 * x_d2 +
                              0.08 * x_d3;

            x_d3 = x_d2;
            x_d2 = x_d1;
            x_d1 = noise_ref_real;

            prim_mic_real = speech_real + noise_prim_real;

            @(posedge clk);
            sample_valid    <= 1'b1;
            primary_speech  <= $rtoi(prim_mic_real);
            reference_noise <= $rtoi(noise_ref_real);

            @(posedge clk);
            sample_valid    <= 1'b0;

            while (!out_valid) begin
                @(posedge clk);
            end

            residual_noise = $itor(cleaned_audio) - speech_real;

            if (sample_idx >= 10 && sample_idx < 110) begin
                init_noise_sq_sum = init_noise_sq_sum + (residual_noise * residual_noise);
                init_sample_count = init_sample_count + 1;
            end

            if (sample_idx >= TOTAL_SAMPLES - 200) begin
                final_noise_sq_sum = final_noise_sq_sum + (residual_noise * residual_noise);
                final_sample_count = final_sample_count + 1;
            end

            if (sample_idx == 0 || sample_idx == 100 || sample_idx == 250 || 
                sample_idx == 500 || sample_idx == 1000 || sample_idx == 1500 || 
                sample_idx == 1999) begin
                $display("  %8d | %8.2f | %12d | %13d | %13d | %12d",
                         sample_idx, time_sec * 1000.0,
                         primary_speech, reference_noise, noise_estimate, cleaned_audio);
            end

            #(CLK_PERIOD * 40);
        end

        init_rms  = $sqrt(init_noise_sq_sum / init_sample_count);
        final_rms = $sqrt(final_noise_sq_sum / final_sample_count);

        if (final_rms > 0.0)
            attenuation_db = 20.0 * ($ln(init_rms / final_rms) / $ln(10.0));
        else
            attenuation_db = 99.0;

        $display("-------------------------------------------------------------------------------");
        $display("\n===============================================================================");
        $display("                        CONVERGENCE & PERFORMANCE REPORT                       ");
        $display("===============================================================================");
        $display("  Total Samples Simulated      : %0d", TOTAL_SAMPLES);
        $display("  Initial Ambient Noise RMS    : %8.2f (Q1.15 LSBs)", init_rms);
        $display("  Steady-State Residual RMS    : %8.2f (Q1.15 LSBs)", final_rms);
        $display("  Total Noise Attenuation      : %8.2f dB", attenuation_db);
        $display("===============================================================================");

        if (attenuation_db >= 18.0) begin
            $display(">>> TEST PASSED: Adaptive filter achieved %0.2f dB attenuation (Target >= 18.0 dB) <<<", attenuation_db);
            $display(">>> Vocal intelligibility restored; maglev/pump resonance successfully cancelled! <<<\n");
        end else begin
            $display(">>> TEST FAILED: Insufficient noise reduction (%0.2f dB < 18.0 dB) <<<", attenuation_db);
            $display(">>> Check LMS convergence parameters or tap depth. <<<\n");
        end

        $finish;
    end

    initial begin
        #(CLK_PERIOD * 200000);
        $display("[WATCHDOG ERROR] Simulation timeout reached without completion!");
        $finish;
    end

endmodule

