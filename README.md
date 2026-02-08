# Guardian Audio Gate - nRF52840 Implementation

Physics-based audio gate for always-on wakeword detection on nRF52840. Uses IIR resonator banks and correlation features to filter noise before running TinyML models, achieving 30% power savings.

## Project Status

**Week 1-2: Complete**
- 4-channel IIR resonator bank implemented with CMSIS-DSP
- Processing time: 297 µs per 320-sample frame (target: < 5000 µs)
- 16.8x faster than required

## Repository Structure
```
guardian-audio-gate/
├── firmware/
│   ├── src/
│   │   └── main.c                              # Main application
│   ├── lib/guardian_dsp/
│   │   ├── src/
│   │   │   └── resonator_df2t.c               # IIR filter implementation
│   │   └── include/
│   │       ├── guardian/
│   │       │   └── resonator_df2t.h           # Public API
│   │       └── resonator_coefs_cmsis.h        # Filter coefficients
│   ├── prj.conf                                # Zephyr configuration
│   └── CMakeLists.txt                          # Build configuration
├── tools/
│   └── coefficient_generation/
│       └── generate_cmsis_resonator_coefs.py  # Coefficient generator
├── zephyr/
│   └── module.yml                              # Zephyr module metadata
└── README.md
```

## Hardware Requirements

- nRF52840 Development Kit (nRF52840-DK)
- USB cable
- J-Link debugger (built into DK)

## Software Requirements

### WSL/Linux Environment
- Ubuntu 20.04 or later
- Zephyr SDK 0.16.5
- West build tool
- Python 3.8+

### Installation

1. Install Zephyr dependencies:
```bash
sudo apt update
sudo apt install -y git cmake ninja-build gperf ccache dfu-util \
  device-tree-compiler wget python3-dev python3-pip python3-setuptools \
  python3-wheel xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev
```

2. Install West:
```bash
pip3 install --user -U west
echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
source ~/.bashrc
```

3. Setup Zephyr workspace:
```bash
mkdir ~/zephyrproject && cd ~/zephyrproject
west init
west update
west zephyr-export
pip3 install -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

4. Install Zephyr SDK:
```bash
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.16.5_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.5
./setup.sh
```

5. Clone this repository:
```bash
cd ~/projects
git clone https://github.com/minoverse/guardian-audio-gate.git
```

6. Link to Zephyr workspace:
```bash
cd ~/zephyrproject
# Edit zephyr/west.yml and add:
# projects:
#   - name: guardian-audio-gate
#     url: https://github.com/minoverse/guardian-audio-gate
#     path: modules/lib/guardian
#     revision: main

west update
```

## Build Instructions

### Generate Filter Coefficients (One-time)
```bash
cd ~/projects/guardian-audio-gate/tools/coefficient_generation
python3 generate_cmsis_resonator_coefs.py
```

This generates `resonator_coefs_cmsis.h` with optimized Q15 fixed-point coefficients for 4 bandpass filters (300Hz, 800Hz, 1500Hz, 2500Hz).

### Build Firmware
```bash
cd ~/zephyrproject
west build -b nrf52840dk/nrf52840 -s ~/projects/guardian-audio-gate/firmware
```

Clean build (if needed):
```bash
cd ~/zephyrproject
rm -rf build
west build -b nrf52840dk/nrf52840 -s ~/projects/guardian-audio-gate/firmware
```

### Flash to Board

1. Connect nRF52840-DK via USB
2. Flash firmware:
```bash
cd ~/zephyrproject
west flash --runner jlink
```

### View Serial Output
```bash
screen /dev/ttyACM0 115200
```

Exit screen: `Ctrl+A` then `K` then `Y`

## Expected Output
```
*** Booting Zephyr OS build v4.3.0 ***
Guardian Audio Gate - Week 1-2 Timing Test
Resonator initialized
Resonator timing: 297 us (target: < 5000 us)
PASS: Meets timing requirement
```

## Problems Encountered & Solutions

### Problem 1: MPU Fault - Data Access Violation

**Symptoms:**
```
***** MPU FAULT *****
Data Access Violation
MMFAR Address: 0x20000f00
```

**Attempts:**
1. Wrong coefficient format (5 values instead of 6)
2. Incorrect coefficient signs (negated a coefficients)
3. State buffer size mismatch (2 values instead of 4)

**Root Cause:**
Large struct (2640 bytes) allocated on stack caused overflow. Stack pointer went out of bounds when CMSIS-DSP init function tried to write to the struct.

**Solution:**
Changed from stack allocation to static allocation:
```c
// Before (WRONG):
int main(void) {
    resonator_bank_df2t_t bank;  // Stack allocation
    ...
}

// After (CORRECT):
static resonator_bank_df2t_t bank;  // Static allocation

int main(void) {
    resonator_bank_df2t_init(&bank);
    ...
}
```

### Problem 2: Wrong CMSIS-DSP Coefficient Format

**Symptoms:**
MPU fault during `arm_biquad_cascade_df1_init_q15()` call.

**What i Tried:**
1. Standard IIR format: `{b0, b1, b2, a1, a2}` (5 values)
2. DF2T format: `{b0, b1, b2, -a1, -a2}` (5 values)
3. Various sign combinations

**Actual Format Required:**
Zephyr CMSIS-DSP uses: `{b0, 0, b1, b2, a1, a2}` (6 values)

The zero is SIMD padding for ARM Cortex-M4 optimization. From the CMSIS-DSP source:
```c
// Coefficient order:
{b10, 0, b11, b12, a11, a12, b20, 0, b21, b22, a21, a22, ...}
//    ^ SIMD padding for 16-bit parallel processing
```

**Solution:**
Updated coefficient generator to output 6 values per biquad stage with proper padding.

### Problem 3: CMSIS-DSP Functions Not Linked

**Symptoms:**
```
undefined reference to `arm_biquad_cascade_df1_init_q15'
```

**Root Cause:**
CMSIS-DSP filtering module not enabled in Zephyr config.

**Solution:**
Added to `prj.conf`:
```ini
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_FILTERING=y
```

CMSIS-DSP has modular compilation - each submodule must be explicitly enabled.

### Problem 4: Timing Functions Not Available

**Symptoms:**
```
undefined reference to `timing_init'
undefined reference to `timing_counter_get'
```

**Solution:**
Added to `prj.conf`:
```ini
CONFIG_TIMING_FUNCTIONS=y
```

## Key Implementation Details

### Filter Architecture

- 4 bandpass IIR filters (biquad DF1 topology)
- Center frequencies: 300Hz, 800Hz, 1500Hz, 2500Hz
- Q factor: 8.0 (narrow bandwidth)
- Frame size: 320 samples (20ms at 16kHz)

### CMSIS-DSP Usage

Uses ARM CMSIS-DSP library for SIMD-optimized filtering:
- `arm_biquad_cascade_df1_init_q15()` - Initialize filter state
- `arm_biquad_cascade_df1_q15()` - Process audio samples

Q15 fixed-point format used throughout for efficiency (no floating-point operations).

### Memory Layout

**Static allocation:**
- Bank structure: 48 bytes (4 channels × 12 bytes)
- State buffers: 32 bytes (4 channels × 4 values × 2 bytes)
- Output buffers: 2560 bytes (4 channels × 320 samples × 2 bytes)
- Total: ~2.6 KB

**Rationale:** nRF52840 stack is limited (~2KB). Large buffers must be static/global to avoid overflow.

## Performance Metrics

| Metric | Value |
|--------|-------|
| Processing time | 297 µs |
| Frame period | 20,000 µs |
| CPU utilization | 1.5% |
| Sample rate | 16 kHz |
| Frame size | 320 samples |
| Channels | 4 |
| Flash usage | 26,724 bytes (2.55%) |
| RAM usage | 4,992 bytes (1.90%) |

## Development Notes

### Why Python + C Split?

**Python (PC):** Filter design requires complex math (matrix operations, square roots, trigonometry). scipy.signal does this efficiently.

**C (nRF52):** Real-time processing needs only multiply-add operations with pre-computed coefficients. Much faster on embedded hardware.

### Why CMSIS-DSP?

Manual implementation: 2500-3000 µs
CMSIS-DSP: 297 µs (8-10x faster)

CMSIS-DSP uses:
- ARM Cortex-M4 SIMD instructions
- Hand-optimized assembly
- Parallel processing (2 samples at once)

### Code Style Decisions

- Static allocation for large buffers (avoid stack overflow)
- Q15 fixed-point (no FPU needed, faster than float)
- Minimal printk() calls (reduce overhead)
- Direct CMSIS-DSP calls (no abstraction layers)

# Week 3-4 Problems & Solutions
### Goal
Implement 5 physics-based features with target: **< 2000 µs**
### Final Result
```
Processing time: 1546 µs (target: < 2000 µs)
PASS: Meets timing requirement

Total pipeline: 1843 µs (297 resonator + 1546 features)
CPU usage: 9.2% of frame budget
```

### Features Implemented

| Feature | Purpose | Time (µs) |
|---------|---------|-----------|
| Energy (RMS) | Signal strength per channel | ~100 |
| Correlation | Inter-channel similarity (d=1.83 discriminator) | ~800 |
| Coherence | Pitch periodicity detection | ~600 |
| Zero-Crossing Rate | Frequency estimation | ~30 |
| Spectral Flux | Frame-to-frame change | ~16 |

### Optimization Journey

| Attempt | Method | Time (µs) | Result |
|---------|--------|-----------|--------|
| 1st | Original (lag 50-400, manual loop) | 5511 | Too slow |
| 2nd | `arm_correlate_q15()` (full correlation) | 7014 |  Worse |
| 3rd | Reduced lag 80-250 + `arm_dot_prod_q15()` | 2446 |  Close |
| **Final** | **Lag 80-250 step-by-2** | **1546** | ** PASS** |

**Total speedup:** 3.6x faster
![Week 3-4 Timing Result]20260208_131113.jpg)
### Key Optimizations

1. **Coherence Feature (Main Bottleneck)**
   - Original: Nested loops 350 lags × 270 samples = 94,500 iterations
   - Reduced lag range: 80-250 (covers 64-200Hz adult speech)
   - Step-by-2: 85 iterations instead of 170
   - Used CMSIS-DSP `arm_dot_prod_q15()` for inner loop

2. **Energy Feature**
   - Used CMSIS-DSP `arm_rms_q15()` instead of manual calculation
   - Required `CONFIG_CMSIS_DSP_STATISTICS=y`

3. **Correlation Feature**
   - Manual Pearson correlation with Q15 fixed-point
   - 3 channel pairs: 0-1, 1-2, 2-3
## Problem 1: Undefined Reference to Feature Functions

**Error:**
```
undefined reference to `extract_energy_features'
undefined reference to `extract_correlation_features'
undefined reference to `extract_coherence_features'
undefined reference to `compute_zcr'
undefined reference to `compute_spectral_flux'
```

**Root Cause:**
Feature source files (.c) not added to CMakeLists.txt, so they weren't being compiled and linked.

**Solution:**
Updated `firmware/lib/guardian_dsp/CMakeLists.txt`:
```cmake
zephyr_library_sources(
    src/resonator_df2t.c
    src/features/energy.c
    src/features/correlation.c
    src/features/coherence.c
    src/features/zcr.c
    src/features/flux.c
)
```

---

## Problem 2: Unknown Type 'size_t'

**Error:**
```
coherence.h:11:56: error: unknown type name 'size_t'
zcr.c:3:56: error: unknown type name 'size_t'
```

**Root Cause:**
Missing `#include <stddef.h>` in header files. `size_t` is defined in stddef.h, not stdint.h.

**Solution:**
Added `#include <stddef.h>` to coherence.h and zcr.h headers.

---

## Problem 3: Undefined Reference to arm_rms_q15

**Error:**
```
energy.c:11: undefined reference to `arm_rms_q15'
```

**Root Cause:**
CMSIS-DSP statistics module not enabled. `arm_rms_q15()` is part of statistics functions, not filtering.

**Solution:**
Added to `prj.conf`:
```ini
CONFIG_CMSIS_DSP_STATISTICS=y
```

CMSIS-DSP has modular compilation - must enable each submodule separately:
- CONFIG_CMSIS_DSP_FILTERING (for filters)
- CONFIG_CMSIS_DSP_STATISTICS (for RMS, mean, variance)

---

### Goal
Implement 5 physics-based features with target: **< 2000 µs**

### Final Result
```
Processing time: 1546 µs (target: < 2000 µs)
PASS: Meets timing requirement

Total pipeline: 1843 µs (297 resonator + 1546 features)
CPU usage: 9.2% of frame budget
```

### Features Implemented

| Feature | Purpose | Time (µs) |
|---------|---------|-----------|
| Energy (RMS) | Signal strength per channel | ~100 |
| Correlation | Inter-channel similarity (d=1.83 discriminator) | ~800 |
| Coherence | Pitch periodicity detection | ~600 |
| Zero-Crossing Rate | Frequency estimation | ~30 |
| Spectral Flux | Frame-to-frame change | ~16 |

### Optimization Journey

| Attempt | Method | Time (µs) | Result |
|---------|--------|-----------|--------|
| 1st | Original (lag 50-400, manual loop) | 5511 | ❌ Too slow |
| 2nd | `arm_correlate_q15()` (full correlation) | 7014 | ❌ Worse |
| 3rd | Reduced lag 80-250 + `arm_dot_prod_q15()` | 2446 | ❌ Close |
| **Final** | **Lag 80-250 step-by-2** | **1546** | **✅ PASS** |

**Total speedup:** 3.6x faster

![Week 3-4 Timing Result](docs/images/week3-4-timing.png)

### Key Optimizations

1. **Coherence Feature (Main Bottleneck)**
   - Original: Nested loops 350 lags × 270 samples = 94,500 iterations
   - Reduced lag range: 80-250 (covers 64-200Hz adult speech)
   - Step-by-2: 85 iterations instead of 170
   - Used CMSIS-DSP `arm_dot_prod_q15()` for inner loop

2. **Energy Feature**
   - Used CMSIS-DSP `arm_rms_q15()` instead of manual calculation
   - Required `CONFIG_CMSIS_DSP_STATISTICS=y`

3. **Correlation Feature**
   - Manual Pearson correlation with Q15 fixed-point
   - 3 channel pairs: 0-1, 1-2, 2-3
## Key Lessons

1. **CMake must list all source files** - C files not in CMakeLists.txt won't be compiled
2. **Include proper headers** - stddef.h for size_t, stdlib.h for abs()
3. **Enable CMSIS-DSP modules explicitly** - statistics, filtering, etc. are separate
4. **Profile before optimizing** - coherence feature was 73% of total time
5. **Algorithmic complexity matters** - O(n²) loops don't scale on embedded systems

---

## Files Modified
```
firmware/prj.conf                           # Added CONFIG_CMSIS_DSP_STATISTICS=y
firmware/lib/guardian_dsp/CMakeLists.txt    # Added feature source files
firmware/lib/guardian_dsp/include/guardian/features/*.h  # Added stddef.h
firmware/src/main.c                         # Feature extraction test
```

---

## Next Steps

1. Confirm 4 features meet < 2000 µs target
2. Optimize coherence feature or make it optional
3. Test with real audio (speech vs noise discrimination)
4. Implement gate decision logic using feature thresholds

## License

MIT License

## Author

Built as part of embedded audio DSP coursework.

## References

- CMSIS-DSP Documentation: https://arm-software.github.io/CMSIS-DSP/
- Zephyr RTOS: https://docs.zephyrproject.org/
- nRF52840 Product Specification: https://infocenter.nordicsemi.com/
