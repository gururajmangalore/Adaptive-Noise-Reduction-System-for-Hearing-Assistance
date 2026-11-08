#!/usr/bin/env python3
"""
Executes the exact simulation logic of tb_adaptive_noise_canceller.v
and outputs the identical console output.
"""
import math

def main():
    CLK_PERIOD = 20
    NUM_TAPS = 8
    DATA_WIDTH = 16
    COEFF_WIDTH = 32
    STEP_SHIFT = 8
    TOTAL_SAMPLES = 1200
    FS = 16000.0
    PI = 3.14159265358979323846
    MU = 64

    # DUT registers
    x_delay = [0] * NUM_TAPS
    w_acc = [0] * NUM_TAPS
    primary_d1 = 0

    x_d1 = 0.0
    x_d2 = 0.0
    x_d3 = 0.0

    init_noise_sq_sum = 0.0
    final_noise_sq_sum = 0.0
    init_sample_count = 0
    final_sample_count = 0

    print("==========================================================================================")
    print("   AEROSPACE LIFE-SUPPORT ELECTRONICS: ADAPTIVE NOISE REDUCTION SIMULATION               ")
    print("   Scenario: Simulated Microgravity Environment / Active Magnetic Levitation Noise Test   ")
    print(f"   Clock: 50 MHz | Fs: 16.0 kHz | Taps: {NUM_TAPS} | Format: Q1.15 | mu: {MU} | shift: {STEP_SHIFT}         ")
    print("==========================================================================================")
    print("\n[T = 300000 ps] Reset released. Beginning audio sample ingestion and adaptive filtering...")
    print("------------------------------------------------------------------------------------------")
    print(" Sample# | Time(ms) | PrimMic d(n) | RefNoise x(n) | NoiseEst y(n) | Cleaned e(n) | ResidNoise | Filter w0..w3")
    print("------------------------------------------------------------------------------------------")

    for sample_idx in range(TOTAL_SAMPLES):
        time_sec = sample_idx / FS

        # 1. 440 Hz Speech
        speech_real = 3500.0 * math.sin(2.0 * PI * 440.0 * time_sec)

        # 2. MagLev Hum (2000 Hz) + Coolant Pump (3000 Hz)
        noise_ref_real = 7000.0 * math.sin(2.0 * PI * 2000.0 * time_sec) + 4000.0 * math.sin(2.0 * PI * 3000.0 * time_sec)

        # 3. Acoustic Plant H(z)
        noise_prim_real = 0.45 * noise_ref_real + 0.25 * x_d1 - 0.15 * x_d2 + 0.08 * x_d3
        x_d3 = x_d2
        x_d2 = x_d1
        x_d1 = noise_ref_real

        prim_mic_real = speech_real + noise_prim_real

        d_in = max(-32768, min(32767, int(round(prim_mic_real))))
        x_in = max(-32768, min(32767, int(round(noise_ref_real))))

        # RTL Stage 1: Shift & Mult
        primary_d1 = d_in

        for k in range(NUM_TAPS - 1, 0, -1):
            x_delay[k] = x_delay[k - 1]
        x_delay[0] = x_in

        mults = []
        for k in range(NUM_TAPS):
            w_16 = w_acc[k] >> 16
            mults.append(x_delay[k] * w_16)

        # RTL Stage 2: Accumulate + Round + Saturate
        y_acc = sum(mults)
        y_round = y_acc + 0x4000
        y_scaled = y_round >> 15
        y_sat = max(-32768, min(32767, y_scaled))

        e_raw = primary_d1 - y_sat
        e_sat = max(-32768, min(32767, e_raw))

        # Weight update
        for k in range(NUM_TAPS):
            grad_prod = e_sat * x_delay[k]
            scaled_grad = MU * grad_prod
            delta_w = scaled_grad >> STEP_SHIFT
            sum_w = w_acc[k] + delta_w
            w_acc[k] = max(-2147483648, min(2147483647, sum_w))

        residual_noise = e_sat - speech_real

        if 10 <= sample_idx < 60:
            init_noise_sq_sum += residual_noise * residual_noise
            init_sample_count += 1

        if sample_idx >= TOTAL_SAMPLES - 100:
            final_noise_sq_sum += residual_noise * residual_noise
            final_sample_count += 1

        if sample_idx in (0, 25, 50, 100, 200, 400, 600, 800, 1000, 1199):
            w0 = (w_acc[0] >> 16) / 32768.0
            w1 = (w_acc[1] >> 16) / 32768.0
            w2 = (w_acc[2] >> 16) / 32768.0
            w3 = (w_acc[3] >> 16) / 32768.0
            print(f" {sample_idx:7d} | {time_sec*1000.0:8.2f} | {d_in:12d} | {x_in:13d} | {y_sat:13d} | {e_sat:12d} | {residual_noise:10.0f} | [{w0:0.3f}, {w1:0.3f}, {w2:0.3f}, {w3:0.3f}]")

    init_rms = math.sqrt(init_noise_sq_sum / init_sample_count)
    final_rms = math.sqrt(final_noise_sq_sum / final_sample_count)
    attenuation_db = 20.0 * math.log10(init_rms / final_rms)

    print("------------------------------------------------------------------------------------------")
    print("\n==========================================================================================")
    print("                         LMS ADAPTIVE FILTER CONVERGENCE REPORT                           ")
    print("==========================================================================================")
    print(f"  Total Audio Samples Processed : {TOTAL_SAMPLES}")
    print(f"  Initial Noise RMS (Unadapted) : {init_rms:8.2f} (Q1.15 LSBs)")
    print(f"  Final Residual Noise RMS      : {final_rms:8.2f} (Q1.15 LSBs)")
    print(f"  Total Noise Attenuation       : {attenuation_db:8.2f} dB")
    print(f"  Learned Weights w[0..3]       : [{(w_acc[0]>>16)/32768.0:0.4f}, {(w_acc[1]>>16)/32768.0:0.4f}, {(w_acc[2]>>16)/32768.0:0.4f}, {(w_acc[3]>>16)/32768.0:0.4f}]")
    print("  Target Plant H[0..3]          : [0.4500, 0.2500, -0.1500, 0.0800]")
    print("==========================================================================================")

    if attenuation_db >= 18.0:
        print(f">>> TEST PASSED: Adaptive filter achieved {attenuation_db:0.2f} dB attenuation (Specification >= 18.0 dB) <<<")
        print(">>> Ambient MagLev switching hum & pump acoustic resonance successfully cancelled! <<<\n")
    else:
        print(f">>> TEST FAILED: Insufficient noise reduction ({attenuation_db:0.2f} dB < 18.0 dB) <<<\n")

if __name__ == "__main__":
    main()

