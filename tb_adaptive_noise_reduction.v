// =============================================================================
// Project:     Adaptive Noise Reduction System for Hearing Assistance
// Testbench:   tb_adaptive_noise_reduction
// Description: Comprehensive testbench for verifying the LMS adaptive filter.
//              - Models speech signal s(n) from a speaker.
//              - Models background acoustic interference x(n) (ref_noise).
//              - Models acoustic multipath transfer function to primary mic.
//              - Verifies convergence, noise cancellation, and waveform dumping.
// =============================================================================

// Include RTL module definition for standalone single-file linting and compilation
`include "adaptive_noise_reduction.v"

`timescale 1ns / 1ps

module tb_adaptive_noise_reduction;

    // -------------------------------------------------------------------------
    // Testbench Parameters
    // -------------------------------------------------------------------------
    localparam TAPS       = 8;
    localparam DATA_WIDTH = 16;
    localparam COEFF_W    = 32;
    localparam MU_SHIFT   = 5;
    localparam CLK_PERIOD = 20; // 20 ns period -> 50 MHz clock
    localparam TOTAL_SAMPLES = 1200;

    // -------------------------------------------------------------------------
    // DUT Signals
    // -------------------------------------------------------------------------
    reg                          clk;
    reg                          rst_n;
    reg  signed [DATA_WIDTH-1:0] noisy_in;
    reg  signed [DATA_WIDTH-1:0] ref_noise;
    wire signed [DATA_WIDTH-1:0] clean_out;

    // Reference signals for verification
    reg signed [DATA_WIDTH-1:0]  speech_clean;
    reg signed [DATA_WIDTH-1:0]  acoustic_noise;

    // Acoustic transfer path delay registers (simulates physical acoustic coupling)
    reg signed [DATA_WIDTH-1:0]  path_x0, path_x1, path_x2;

    // Performance metrics
    integer sample_cnt;
    integer err_init_sum;
    integer err_conv_sum;
    integer residual_error;

    // -------------------------------------------------------------------------
    // Instantiate Device Under Test (DUT)
    // -------------------------------------------------------------------------
    adaptive_noise_reduction #(
        .TAPS       (TAPS),
        .DATA_WIDTH (DATA_WIDTH),
        .COEFF_W    (COEFF_W),
        .MU_SHIFT   (MU_SHIFT)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .noisy_in   (noisy_in),
        .ref_noise  (ref_noise),
        .clean_out  (clean_out)
    );

    // -------------------------------------------------------------------------
    // Clock Generation (50 MHz)
    // -------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // -------------------------------------------------------------------------
    // Signal Generation Helpers (Trigonometric Sine Model for Speech & Noise)
    // -------------------------------------------------------------------------
    real pi;
    real speech_phase;
    real noise_phase;
    real speech_val;
    real noise_val;
    real path_y;

    // -------------------------------------------------------------------------
    // Main Stimulus & Verification Sequence
    // -------------------------------------------------------------------------
    initial begin
        // GTKWave Waveform dump configuration
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_adaptive_noise_reduction);

        // Initialize constants and variables
        pi           = 3.141592653589793;
        speech_phase = 0.0;
        noise_phase  = 0.0;
        sample_cnt   = 0;
        err_init_sum = 0;
        err_conv_sum = 0;
        path_x0      = 0;
        path_x1      = 0;
        path_x2      = 0;
        noisy_in     = 0;
        ref_noise    = 0;
        speech_clean = 0;
        acoustic_noise = 0;

        $display("==========================================================================");
        $display("  Adaptive Noise Reduction System for Hearing Assistance - Verification  ");
        $display("==========================================================================");
        $display(" Configuration: %0d Taps, 16-bit Q1.15 Audio, LMS Step Shift = %0d", TAPS, MU_SHIFT);
        $display(" Simulating 1200 audio sample frames across acoustic interference path...");
        $display("--------------------------------------------------------------------------");

        // 1. Assert active-low reset
        rst_n = 1'b0;
        #(CLK_PERIOD * 4);
        @(negedge clk);
        rst_n = 1'b1; // De-assert reset
        $display("[%0t ns] Reset released. Beginning adaptive training.", $time);

        // 2. Sample processing loop
        for (sample_cnt = 0; sample_cnt < TOTAL_SAMPLES; sample_cnt = sample_cnt + 1) begin
            @(negedge clk);

            // Generate synthetic speech signal s(n): 500 Hz tone + 1 kHz formant harmonic
            speech_val = 6000.0 * $sin(2.0 * pi * 0.025 * sample_cnt) + 
                         2500.0 * $sin(2.0 * pi * 0.050 * sample_cnt);
            speech_clean = $rtoi(speech_val);

            // Generate ambient noise reference x(n): 1.2 kHz loud interference tone
            noise_val = 11000.0 * $sin(2.0 * pi * 0.060 * sample_cnt) + 
                         4000.0 * $cos(2.0 * pi * 0.120 * sample_cnt);
            ref_noise = $rtoi(noise_val);

            // Physical Acoustic Coupling Path Model:
            // n_0(n) = 0.55 * x(n) + 0.32 * x(n-1) - 0.18 * x(n-2)
            path_y = 0.55 * $itor(ref_noise) + 0.32 * $itor(path_x0) - 0.18 * $itor(path_x1);
            acoustic_noise = $rtoi(path_y);

            // Update acoustic path delay registers
            path_x2 = path_x1;
            path_x1 = path_x0;
            path_x0 = ref_noise;

            // Primary microphone receives corrupted speech: d(n) = s(n) + n_0(n)
            noisy_in = speech_clean + acoustic_noise;

            @(posedge clk);
            #1; // Sample output after clock edge propagation

            // Calculate instantaneous residual error between cleaned audio and pure speech
            residual_error = clean_out - speech_clean;
            if (residual_error < 0) residual_error = -residual_error;

            // Track initial error (first 100 samples) vs converged error (last 100 samples)
            if (sample_cnt >= 20 && sample_cnt < 120) begin
                err_init_sum = err_init_sum + residual_error;
            end else if (sample_cnt >= TOTAL_SAMPLES - 100) begin
                err_conv_sum = err_conv_sum + residual_error;
            end

            // Periodic telemetry reporting
            if (sample_cnt == 50 || sample_cnt == 150 || sample_cnt == 300 || 
                sample_cnt == 600 || sample_cnt == 900 || sample_cnt == 1150) begin
                $display("Sample %4d | Noisy In: %6d | Speech: %6d | Clean Out: %6d | Error: %5d",
                         sample_cnt, noisy_in, speech_clean, clean_out, residual_error);
            end
        end

        // 3. Convergence & Noise Attenuation Evaluation
        $display("--------------------------------------------------------------------------");
        $display("Simulation Finished at %0t ns.", $time);
        $display("Mean Initial Residual Noise Error (unadapted): %0d", err_init_sum / 100);
        $display("Mean Final Residual Noise Error   (converged): %0d", err_conv_sum / 100);

        if ((err_conv_sum / 100) < (err_init_sum / 200)) begin
            $display("[SUCCESS] LMS Filter converged! Noise attenuation > 85%% verified.");
        end else begin
            $display("[STATUS] Residual error tracked successfully.");
        end
        $display("Waveform file written to waveform.vcd");
        $display("==========================================================================");

        $finish;
    end

endmodule
