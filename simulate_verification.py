#!/usr/bin/env python3
"""
=============================================================================
Company/Project: Aerospace Life-Support Electronics & Bio-Acoustics Lab
Script Name:     simulate_verification.py
Description:     Bit-accurate Python reference simulation reproducing the
                 exact cycle-by-cycle RTL behavior of the Verilog Adaptive
                 Noise Reduction System and Testbench.
=============================================================================
"""

import math

def simulate():
    NUM_TAPS = 16
    DATA_WIDTH = 16
    STEP_SHIFT = 10
    MU = 32
    TOTAL_SAMPLES = 2000
    FS = 16000.0
    PI = 3.14159265358979323846

    # DUT internal registers
    w_acc = [0] * NUM_TAPS      # 32-bit signed
    x_pipe = [0] * NUM_TAPS     # 16-bit signed
    
    # Acoustic plant delay line
    x_d1 = 0.0
    x_d2 = 0.0
    x_d3 = 0.0

    init_noise_sq = 0.0
    final_noise_sq = 0.0
    init_count = 0
    final_count = 0

    print("=" * 80)
    print(" AEROSPACE LIFE-SUPPORT ELECTRONICS - BIO-ACOUSTIC ADAPTIVE NOISE REDUCTION ")
    print(f" Python Bit-Accurate Verification Model | Fs = {FS/1000.0:.1f} kHz | Taps = {NUM_TAPS}")
    print(f" LMS Step Size (mu) = {MU} | Step Shift = {STEP_SHIFT}")
    print("=" * 80)
    print(f"{'Sample #':>9} | {'Time(ms)':>8} | {'PrimMic d(n)':>12} | {'RefNoise x(n)':>13} | {'NoiseEst y(n)':>13} | {'Cleaned e(n)':>12}")
    print("-" * 80)

    for n in range(TOTAL_SAMPLES):
        t = n / FS

        # 1. Acoustic Scene Synthesis
        # 440 Hz Speech tone
        speech_real = 4500.0 * math.sin(2.0 * PI * 440.0 * t)
        speech_int = int(round(speech_real))

        # MagLev switching (2400 Hz) + Coolant pump (3600 Hz)
        noise_ref_real = 8000.0 * math.sin(2.0 * PI * 2400.0 * t) + 4500.0 * math.sin(2.0 * PI * 3600.0 * t)
        noise_ref_int = int(round(noise_ref_real))

        # Acoustic coupling H(z) = 0.50 + 0.25*z^-1 - 0.15*z^-2 + 0.08*z^-3
        noise_prim_real = 0.50 * noise_ref_real + 0.25 * x_d1 - 0.15 * x_d2 + 0.08 * x_d3
        x_d3 = x_d2
        x_d2 = x_d1
        x_d1 = noise_ref_real

        primary_int = int(round(speech_real + noise_prim_real))

        # Clamp inputs to 16-bit signed Q1.15
        primary_int = max(-32768, min(32767, primary_int))
        noise_ref_int = max(-32768, min(32767, noise_ref_int))

        # 2. RTL Shift Delay Line
        for k in range(NUM_TAPS - 1, 0, -1):
            x_pipe[k] = x_pipe[k - 1]
        x_pipe[0] = noise_ref_int

        # 3. FIR Parallel Multiplication (Q1.15 * Q1.15 = Q2.30)
        # Weight slice is w_acc[k][31:16]
        mult_stage = []
        for k in range(NUM_TAPS):
            w_16 = w_acc[k] >> 16
            mult_stage.append(x_pipe[k] * w_16)

        # 4. Adder Tree Accumulation with Half-LSB Rounding
        y_acc = sum(mult_stage)
        y_round = y_acc + 0x4000
        y_scaled = y_round >> 15

        # Saturation to 16-bit signed
        y_sat = max(-32768, min(32767, y_scaled))

        # 5. Error Subtraction & Saturation: e(n) = d(n) - y(n)
        e_raw = primary_int - y_sat
        e_sat = max(-32768, min(32767, e_raw))

        # 6. LMS Weight Update Engine: w_acc[k] <= sat32(w_acc[k] + delta_w)
        for k in range(NUM_TAPS):
            grad_prod = e_sat * x_pipe[k]
            scaled_grad = MU * grad_prod
            delta_w = scaled_grad >> STEP_SHIFT
            new_w = w_acc[k] + delta_w
            w_acc[k] = max(-2147483648, min(2147483647, new_w))

        # 7. Collect Verification Metrics
        resid_noise = e_sat - speech_int
        if 10 <= n < 110:
            init_noise_sq += resid_noise * resid_noise
            init_count += 1
        if n >= TOTAL_SAMPLES - 200:
            final_noise_sq += resid_noise * resid_noise
            final_count += 1

        if n in (0, 100, 250, 500, 1000, 1500, 1999):
            print(f"{n:9d} | {t*1000.0:8.2f} | {primary_int:12d} | {noise_ref_int:13d} | {y_sat:13d} | {e_sat:12d}")

    init_rms = math.sqrt(init_noise_sq / init_count)
    final_rms = math.sqrt(final_noise_sq / final_count)
    attenuation_db = 20.0 * math.log10(init_rms / final_rms)

    print("-" * 80)
    print("\n" + "=" * 80)
    print("                        CONVERGENCE & PERFORMANCE REPORT                       ")
    print("=" * 80)
    print(f"  Total Samples Simulated      : {TOTAL_SAMPLES}")
    print(f"  Initial Ambient Noise RMS    : {init_rms:8.2f} (Q1.15 LSBs)")
    print(f"  Steady-State Residual RMS    : {final_rms:8.2f} (Q1.15 LSBs)")
    print(f"  Total Noise Attenuation      : {attenuation_db:8.2f} dB")
    print("=" * 80)

    if attenuation_db >= 18.0:
        print(f">>> TEST PASSED: Adaptive filter achieved {attenuation_db:.2f} dB attenuation (Target >= 18.0 dB) <<<")
        print(">>> Vocal intelligibility restored; maglev/pump resonance successfully cancelled! <<<\n")
    else:
        print(f">>> TEST FAILED: Insufficient noise reduction ({attenuation_db:.2f} dB < 18.0 dB) <<<\n")

if __name__ == "__main__":
    simulate()

