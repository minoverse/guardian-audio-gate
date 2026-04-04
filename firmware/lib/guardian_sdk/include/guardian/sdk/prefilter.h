/* guardian/sdk/prefilter.h — Guardian Physics Pre-Filter SDK public API
 *
 * Physics-based IIR bandpass filter bank for always-on audio preprocessing.
 * Targets speech-detection front-ends in TWS earbuds and hearable devices.
 *
 * Key properties:
 *   • Four-channel resonator bank tuned to speech formant regions
 *     (300 / 800 / 1500 / 2500 Hz at fs = 16 000 Hz by default)
 *   • DC-bias removal: 1st-order IIR HPF at 20 Hz (removes PDM/ADC bias)
 *   • AGC: fast-attack (10 ms) / slow-release (500 ms) per-frame normalisation
 *   • Spectral Flatness Measure (SFM) output — distinguishes tonal speech
 *     from broadband noise without FFT
 *   • Direct Form I topology — no limit cycles; stable over device lifetime
 *   • Pure C99 — no platform, OS, CMSIS, or RTOS dependencies
 *   • No heap allocation — caller provides state; suitable for MCU stacks
 *
 * Minimum platform requirements:
 *   • IEEE 754 float32 (hardware FPU recommended for <2 µs/frame on M4F)
 *   • C99 compiler + <stdint.h>, <stddef.h>, <math.h> (sqrtf, logf, expf)
 *
 * Quick start:
 *   guardian_pf_config_t cfg = GUARDIAN_PF_CONFIG_DEFAULT;
 *   guardian_pf_t        pf;
 *   guardian_pf_init(&pf, &cfg);
 *
 *   // each 20 ms frame of int16 PCM @ 16 kHz:
 *   guardian_pf_process(&pf, frame, GUARDIAN_PF_FRAME_SAMPLES);
 *   const int16_t *ch0 = guardian_pf_get_channel(&pf, 0);   // 300 Hz band
 *   float sfm = guardian_pf_get_sfm(&pf);   // 0=tonal speech, 1=flat noise
 *
 * Copyright (c) 2026 Guardian Audio. All rights reserved.
 * SPDX-License-Identifier: Proprietary
 * ─────────────────────────────────────────────────────────────────────── */

#ifndef GUARDIAN_SDK_PREFILTER_H
#define GUARDIAN_SDK_PREFILTER_H

#include <stdint.h>
#include <stddef.h>
#include "biquad_df1.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Version ────────────────────────────────────────────────────────── */
#define GUARDIAN_SDK_VERSION_MAJOR  1
#define GUARDIAN_SDK_VERSION_MINOR  0
#define GUARDIAN_SDK_VERSION_PATCH  0

/* ── Compile-time limits ────────────────────────────────────────────── */
#define GUARDIAN_PF_NUM_CHANNELS    4    /* resonator channels                */
#define GUARDIAN_PF_FRAME_SAMPLES   320  /* max samples per process() call    */

/* ── Coefficient type ───────────────────────────────────────────────── */
/* One biquad stage in CMSIS storage convention: {b0, b1, b2, -a1, -a2}
 * Compatible with arm_biquad_cascade_df1_f32 coefficient tables.       */
typedef struct {
    float coef[5];
} guardian_pf_coef_t;

/* ── Configuration ──────────────────────────────────────────────────── */
typedef struct {
    /* Resonator coefficient table.
     * NULL → use built-in 300/800/1500/2500 Hz defaults (fs=16000 Hz). */
    const guardian_pf_coef_t *coefs;   /* [GUARDIAN_PF_NUM_CHANNELS] or NULL */

    /* Center frequencies corresponding to coefs[] — used for metadata.
     * NULL → use built-in {300, 800, 1500, 2500}.                      */
    const uint16_t *center_freqs_hz;   /* [GUARDIAN_PF_NUM_CHANNELS] or NULL */

    /* Sample rate.  Must match the coefficient design rate.             */
    uint32_t sample_rate_hz;

    /* DC-removal HPF cutoff.  Set to 0.0 to disable DC removal.
     * Recommended: 20 Hz — removes PDM/ADC DC bias, preserves speech.  */
    float dc_hpf_hz;

    /* AGC target RMS in normalised [0.0, 1.0] range.
     * 0.25 = −12 dBFS (leaves 12 dB headroom for transients).
     * Set to 0.0 to disable AGC (pass through at cal_gain only).       */
    float agc_target_rms;
    float agc_gain_min;   /* hard floor on applied gain (e.g. 0.25 = −12 dB) */
    float agc_gain_max;   /* hard ceiling on applied gain (e.g. 4.0 = +12 dB)*/

    /* Static calibration gain applied before AGC.
     * Load from persistent storage (NVMC, NVM, etc.) at boot.
     * Compensates for ±3 dB unit-to-unit microphone sensitivity spread. */
    float cal_gain;
} guardian_pf_config_t;

/* Default configuration — 300/800/1500/2500 Hz, fs=16000, AGC −12 dBFS */
#define GUARDIAN_PF_CONFIG_DEFAULT {        \
    .coefs            = NULL,               \
    .center_freqs_hz  = NULL,               \
    .sample_rate_hz   = 16000U,             \
    .dc_hpf_hz        = 20.0f,              \
    .agc_target_rms   = 0.25f,              \
    .agc_gain_min     = 0.25f,              \
    .agc_gain_max     = 4.0f,               \
    .cal_gain         = 1.0f,               \
}

/* ── State (caller-allocated — no heap required) ────────────────────── */
/* Fields prefixed with underscore are private — do not access directly. */
typedef struct {
    gd_biquad_df1_t  _filters[GUARDIAN_PF_NUM_CHANNELS];

    /* DC removal: 1st-order IIR HPF y[n] = α·y[n−1] + x[n] − x[n−1]  */
    float   _dc_alpha;
    float   _dc_x_prev;
    float   _dc_y_prev;
    int     _dc_enabled;

    /* AGC envelope tracker + gain smoother                              */
    float   _agc_target;
    float   _agc_min;
    float   _agc_max;
    float   _agc_envelope;
    float   _agc_gain;
    int     _agc_enabled;

    /* Static calibration multiplier                                     */
    float   _cal_gain;

    /* Processed (DC-removed + AGC) Q15 output per channel              */
    int16_t _outputs[GUARDIAN_PF_NUM_CHANNELS][GUARDIAN_PF_FRAME_SAMPLES];

    /* Channel metadata                                                  */
    uint16_t _center_freqs[GUARDIAN_PF_NUM_CHANNELS];

    /* Last-processed SFM (computed inside process())                   */
    float   _sfm;

    /* Last frame sample count (for get_sfm validity check)             */
    size_t  _last_len;
} guardian_pf_t;

/* ── API ────────────────────────────────────────────────────────────── */

/* Initialise the pre-filter.  Must be called once before process().
 * Returns 0 on success, −1 on invalid config (NULL pf, bad sample_rate).*/
int guardian_pf_init(guardian_pf_t *pf, const guardian_pf_config_t *cfg);

/* Process one frame of int16 PCM samples (Q15, range [−32768, 32767]).
 * len must be ≤ GUARDIAN_PF_FRAME_SAMPLES.
 *
 * Pipeline per frame:
 *   1. Apply cal_gain + DC removal (HPF @ dc_hpf_hz)
 *   2. AGC: update envelope, smooth gain, apply total gain
 *   3. Run each resonator channel on preprocessed samples
 *   4. Compute SFM from channel energies
 *
 * State (DC history, AGC envelope, filter memories) persists across calls. */
void guardian_pf_process(guardian_pf_t *pf, const int16_t *in, size_t len);

/* Get Q15 output buffer for one resonator channel (0 … NUM_CHANNELS−1).
 * Valid after each guardian_pf_process() call.
 * Buffer length = len passed to the most recent process() call.         */
const int16_t *guardian_pf_get_channel(const guardian_pf_t *pf, int ch);

/* Spectral Flatness Measure computed from channel band energies.
 * Range [0.0, 1.0]:
 *   0.0 → highly tonal (speech, whistling)
 *   1.0 → flat noise (white/pink noise, silence)
 * Valid after each guardian_pf_process() call.
 * A threshold of ~0.60 separates speech from broadband noise.           */
float guardian_pf_get_sfm(const guardian_pf_t *pf);

/* Center frequency of a channel in Hz (metadata, set at init).         */
uint16_t guardian_pf_get_center_freq(const guardian_pf_t *pf, int ch);

/* Reset all filter state and AGC to initial conditions.
 * Configuration (coefficients, gains) is preserved.
 * Call after audio discontinuities or stream restarts.                  */
void guardian_pf_reset(guardian_pf_t *pf);

/* Update calibration gain at runtime (e.g. after NVMC write).
 * Takes effect on the next guardian_pf_process() call.                 */
void guardian_pf_set_cal_gain(guardian_pf_t *pf, float cal_gain);

#ifdef __cplusplus
}
#endif

#endif /* GUARDIAN_SDK_PREFILTER_H */
