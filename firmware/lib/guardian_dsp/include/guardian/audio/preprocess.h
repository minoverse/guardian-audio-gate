#ifndef GUARDIAN_PREPROCESS_H
#define GUARDIAN_PREPROCESS_H

#include <stdint.h>
#include <stddef.h>

/* ── Audio front-end preprocessor ────────────────────────────────────────────
 * Applied to every raw PDM frame BEFORE the resonator bank.
 * Does NOT modify the DMA ring buffer — writes to a separate output buffer.
 * The preroll ring (used by TinyML) stays raw; EI SDK does its own MFCC.
 *
 * Pipeline (per frame):
 *   1. Apply mic calibration gain (NVMC-loaded, compensates ±3dB sensitivity)
 *   2. DC removal: 1st-order IIR HPF @ 20 Hz (removes PDM DC bias)
 *   3. AGC: fast-attack (10ms) / slow-release (500ms) normalization
 *
 * State persists across frames — init once at startup, call every frame.
 * ─────────────────────────────────────────────────────────────────────────── */

typedef struct {
    /* DC removal state — IIR HPF: y[n] = α*y[n-1] + x[n] - x[n-1]
     * α = 0.99215 (pole at 20 Hz, fs=16kHz)                                  */
    float dc_x_prev;   /* x[n-1]: last raw input sample                       */
    float dc_y_prev;   /* y[n-1]: last HPF output (float for precision)        */

    /* AGC state — envelope tracker + gain smoother
     * envelope: RMS estimate (fast-attack IIR, slow-release IIR)
     * gain    : current applied gain, smoothed to avoid zipper noise          */
    float agc_envelope; /* RMS envelope estimate (frame-rate IIR)             */
    float agc_gain;     /* current gain applied to samples                    */

    /* Mic calibration gain (1.0 = nominal, loaded from NVMC at boot)         */
    float cal_gain;
} audio_frontend_t;

/* ── Constants ──────────────────────────────────────────────────────────────
 * AGC_TARGET_RMS : -12 dBFS = 0.25 (leaves 12 dB headroom for transients)
 * AGC_GAIN_MIN   : 0.25× = -12 dB  (don't amplify already-loud signals)
 * AGC_GAIN_MAX   : 4.0×  = +12 dB  (max boost for quiet whisper)            */
#define AUDIO_FRONTEND_AGC_TARGET  0.25f
#define AUDIO_FRONTEND_AGC_MIN     0.25f
#define AUDIO_FRONTEND_AGC_MAX     4.0f

void audio_frontend_init(audio_frontend_t *fe, float cal_gain);

/* Process one frame: raw Q15 input → preprocessed Q15 output.
 * in and out must point to different buffers (not in-place).
 *
 * Returns 0 on success, -EINVAL if any pointer is NULL or len is 0.
 * The error path is a programmer error — should never fire in production.
 * Saturation uses __SSAT (single ARM SSAT instruction) — no UB on overflow. */
int audio_frontend_process(audio_frontend_t *fe,
                           const int16_t *in,
                           int16_t *out,
                           size_t len);

#endif /* GUARDIAN_PREPROCESS_H */
