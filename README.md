# Adaptive Noise Reduction System for Hearing Assistance

An LMS (Least Mean Squares) adaptive filter designed in Verilog HDL for real-time acoustic noise cancellation in hearing assistance applications.

## 📌 Project Overview
Hearing aid devices often struggle to isolate speech signals in noisy ambient environments. This project implements an 8-tap LMS adaptive filter architecture in fixed-point Q1.15 arithmetic. The system dynamically adapts its filter coefficients to cancel background acoustic interference and reconstruct a clear speech signal.

## 🏗️ Hardware Architecture & Features
- **Algorithm:** Least Mean Squares (LMS) Adaptive Filtering Engine
- **Data Precision:** 16-bit Signed Q1.15 Fixed-Point Arithmetic
- **Filter Taps:** 8 Configurable Taps (`w[0]` through `w[7]`)
- **Step Size ($\mu$):** Bit-shift parameterization (`MU_SHIFT = 5`) for efficient hardware multiplier scaling
- **Overflow Protection:** Built-in dynamic clamping and saturation logic to prevent signal wrap-around

## 🛠️ Simulation & EDA Toolchain
- **RTL Language:** Verilog-2012
- **Compiler:** Icarus Verilog (`iverilog`)
- **Simulation Engine:** `vvp`
- **Waveform Viewer:** GTKWave (`gtkwave`)

---

## 🚀 Getting Started

### 1. Prerequisites
Ensure you have `iverilog` and `gtkwave` installed on your system.

**Ubuntu / WSL:**
```bash
sudo apt update && sudo apt install -y iverilog gtkwave
