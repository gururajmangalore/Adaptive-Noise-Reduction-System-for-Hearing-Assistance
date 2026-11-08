# ==============================================================================
# Makefile: Adaptive Noise Reduction System for Hearing Assistance
# Toolchain: Icarus Verilog (iverilog), VVP runtime engine, GTKWave
# Compatible with both Windows (PowerShell/CMD) and Linux (WSL)
# ==============================================================================

# Project files and variables
TOP_MODULE = adaptive_noise_reduction
RTL_SRC    = $(TOP_MODULE).v
TB_SRC     = tb_$(TOP_MODULE).v
SIM_BIN    = $(TOP_MODULE)_sim.vvp
VCD_FILE   = waveform.vcd
GTKW_FILE  = waveform.gtkw

# EDA Tools
IVERILOG   = iverilog
VVP        = vvp
GTKWAVE    = gtkwave

# Verilog compiler flags: 2012 standard, all warnings enabled, suppress timescale inheritance
VFLAGS     = -g2012 -Wall -Wno-timescale

# Cross-platform environment handling
ifeq ($(OS),Windows_NT)
    LAUNCH_WAVE = cmd /c "start \"\" $(GTKWAVE) $(VCD_FILE) $(GTKW_FILE)"
    CLEAN_CMD   = cmd /c "del /f /q $(SIM_BIN) $(VCD_FILE) *.vvp *.vcd 2>nul"
else
    LAUNCH_WAVE = $(GTKWAVE) $(VCD_FILE) $(GTKW_FILE) &
    CLEAN_CMD   = rm -f $(SIM_BIN) $(VCD_FILE) *.vvp *.vcd
endif

.PHONY: all compile simulate wave clean

# Default: compile and run simulation
all: simulate

# Compile synthesizable RTL and testbench
compile: $(SIM_BIN)

$(SIM_BIN): $(RTL_SRC) $(TB_SRC)
	$(IVERILOG) $(VFLAGS) -o $(SIM_BIN) $(TB_SRC)

# Execute the simulation using the VVP runtime engine
simulate: compile
	$(VVP) $(SIM_BIN)

# Open the generated VCD waveform in GTKWave (non-blocking)
wave: simulate
	$(LAUNCH_WAVE)

# Remove generated simulation artifacts
clean:
	$(CLEAN_CMD)
