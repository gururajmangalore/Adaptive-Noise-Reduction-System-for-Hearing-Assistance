Adaptive Noise Reduction System for Hearing Assistance

An edge-ready, real-time digital signal processing (DSP) pipeline engineered to improve speech clarity and attenuate dynamic background noise for hearing assistance devices. The system estimates real-time Signal-to-Noise Ratio (SNR) and adapts filtering spectral masks on a per-frame basis to minimize distortion while maximizing speech intelligibility.

Key Features

Adaptive Noise Suppression: Dynamically tracks non-stationary and stationary noise sources (e.g., street noise, ambient crowd chatter, wind).

Ultra-Low Latency: Optimized frame-by-frame STFT buffer pipeline running at $<15\text{ ms}$ processing delay, meeting standard hearing aid requirements.

Hybrid Frequency Filtering: Combines spectral subtraction, Wiener filtering, and psychoacoustic masking curves to suppress noise while preserving formant frequencies.

Voice Activity Detection (VAD): Energy- and entropy-based VAD module to prevent speech degradation during active talking phases.

Embedded-Ready Architecture: Lightweight memory footprint designed for deployment on edge microcontrollers and SBCs (e.g., Raspberry Pi, STM32, ESP32).

System Architecture

                 ┌───────────────────────────┐
                 │    Raw Audio Input (Mic)  │
                 └─────────────┬─────────────┘
                               │
                               ▼
               ┌──────────────────────────────┐
               │ Windowing & STFT Processing  │
               └───────────────┬──────────────┘
                               │
                               ▼
            ┌────────────────────────────────────┐
            │   Spectral Analysis & VAD Engine   │
            └──────────────────┬─────────────────┘
                               │
                ┌──────────────┴──────────────┐
                ▼                             ▼
   ┌───────────────────────────┐ ┌───────────────────────────┐
   │ SNR & Noise Floor Tracking│ │ Spectral Gain Estimator   │
   └────────────┬──────────────┘ └────────────┬──────────────┘
                └──────────────┬──────────────┘
                               │
                               ▼
                ┌─────────────────────────────┐
                │ Adaptive Filter Engine      │
                │ (Wiener / Spectral Masking) │
                └──────────────┬──────────────┘
                               │
                               ▼
                ┌─────────────────────────────┐
                │ Inverse STFT & Overlap-Add  │
                └──────────────┬──────────────┘
                               │
                               ▼
                 ┌───────────────────────────┐
                 │ Enhanced Speech Output    │
                 └───────────────────────────┘


Project Structure

Adaptive-Noise-Reduction-System/
├── data/                  # Sample audio files (clean and noisy test vectors)
├── docs/                  # System block diagrams and technical design notes
├── src/
│   ├── __init__.py
│   ├── audio_io.py        # Micro-buffer streaming and audio file I/O
│   ├── vad.py             # Voice Activity Detection module
│   ├── snr_estimator.py   # Online SNR estimation routines
│   └── adaptive_filter.py # Spectral subtraction & Wiener filter core
├── tests/                 # Unit tests and benchmarking scripts
├── main.py                # Pipeline entry point for real-time/file execution
├── requirements.txt       # Dependencies
├── LICENSE                # License details
└── README.md              # Project documentation


Getting Started

Prerequisites

Python: 3.8+

System Libraries: portaudio (required for real-time microphone stream input)

Linux / Debian

sudo apt-get install portaudio19-dev python3-pyaudio


macOS

brew install portaudio


Installation

Clone the repository:

git clone https://github.com/your-username/Adaptive-Noise-Reduction-System-for-Hearing-Assistance.git
cd Adaptive-Noise-Reduction-System-for-Hearing-Assistance


Create a virtual environment and activate it:

python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate


Install required Python packages:

pip install -r requirements.txt


Usage

1. Process an Audio File

Enhance speech clarity on a pre-recorded .wav file:

python main.py --input data/noisy_sample.wav --output data/enhanced_sample.wav


2. Live Microphone Stream

Run real-time noise reduction using your default microphone input:

python main.py --realtime


3. Run Benchmark Tests

Evaluate algorithm processing latency and performance:

pytest tests/


Evaluation & Metrics

The system algorithm performance is benchmarked using objective speech quality metrics:

PESQ (Perceptual Evaluation of Speech Quality): Assesses overall voice clarity post-enhancement.

STOI (Short-Time Objective Intelligibility): Measures speech intelligibility improvement in high-noise environments.

Processing Latency: Monitored per frame buffer (e.g., 256–512 samples at $16\text{ kHz}$) to ensure zero audible delay.

Tech Stack

Language: Python / C++ (Signal processing acceleration)

DSP Core: NumPy, SciPy, librosa

Audio I/O: PyAudio, SoundFile

Contributing

Contributions are welcome! To contribute:

Fork the Project.

Create your Feature Branch (git checkout -b feature/AmazingFeature).

Commit your Changes (git commit -m 'Add some AmazingFeature').

Push to the Branch (git push origin feature/AmazingFeature).


