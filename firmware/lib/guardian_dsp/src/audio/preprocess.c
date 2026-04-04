#include "guardian/audio/preprocess.h"
#include <arm_math.h>
#include <zephyr/sys/util.h>
#include <errno.h>

/* ── DC removal coefficients ────────────────────────────────────────────────
 * 1st-order IIR highpass: y[n] = α*y[n-1] + x[n] - x[n-1]
 * α = 1 - 2π*fc/fs = 1 - 2π*20/16000 = 0.992154
 *
 * Derived from bilinear transform of analog HPF H(s) = s / (s + 2π*20):
 *   H(z) = (1 - z^-1) / (1 - 0.9922*z^-1)
 *
 * At 20 Hz  : |H| = 0.707 (-3 dB) — DC bias removed
 * At 100 Hz : |H| = 0.981 (-0.16 dB) — speech fundamental preserved
 * At 300 Hz : |H| = 0.997 (-0.03 dB) — negligible attenuation           */
#define DC_ALPHA 0.992154f

/* ── AGC time constants ─────────────────────────────────────────────────────
 * Frame rate = 50 frames/s (20ms per frame, 320 samples @ 16kHz).
 * α_frame = 1 - exp(-dt/τ) for dt=20ms.
 *
 * Attack  τ = 10ms  → α = 1 - exp(-20/10)  = 0.8647 (fast: prevents clipping)
 * Release τ = 500ms → α = 1 - exp(-20/500) = 0.0392 (slow: no pumping)
 * Gain smoother τ = 50ms → α = 1 - exp(-20/50) = 0.3297 (avoids zipper)    */
#define AGC_ALPHA_ATTACK   0.8647f
#define AGC_ALPHA_RELEASE  0.0392f
#define AGC_ALPHA_SMOOTH   0.3297f

void audio_frontend_init(audio_frontend_t *fe, float cal_gain)
{
    fe->dc_x_prev    = 0.0f;
    fe->dc_y_prev    = 0.0f;
    fe->agc_envelope = AUDIO_FRONTEND_AGC_TARGET; /* start at target, not 0 */
    fe->agc_gain     = 1.0f;
    /* Clamp cal_gain to mic_cal spec [0.5, 2.0].
     *
     * We use the explicit range form !(x >= lo && x <= hi) → fallback, NOT
     * CLAMP().  Reason: Zephyr's CLAMP(NaN, lo, hi) returns NaN because NaN
     * comparisons are always false, so MIN/MAX both propagate NaN.  The
     * inverted-AND form correctly catches NaN (NaN >= 0.5 is false → rejects).
     *
     * mic_cal_is_valid() already applies this check before saving to NVMC,
     * but a second layer here is free (one comparison at init time) and
     * prevents total_gain = NaN from ever reaching the per-sample hot loop.
     * Fallback is unity gain — same as "no calibration programmed".          */
    fe->cal_gain = (cal_gain >= 0.5f && cal_gain <= 2.0f) ? cal_gain : 1.0f;
}

int audio_frontend_process(audio_frontend_t *fe,
                           const int16_t *in,
                           int16_t *out,
                           size_t len)
{
    /* Zephyr-style guard: programmer error if pointers are NULL or len is 0.
     * In production (GATE_MODE 2 normal path) this never fires — all three
     * arguments are statically allocated in main().                          */
    if (!fe || !in || !out || len == 0U) {
        return -EINVAL;
    }

    /* ── Step 1: compute frame RMS for AGC envelope update ───────────────────
     * Done first on raw input so the gain applied this frame is based on the
     * current frame's actual level — not the previous frame's estimate.
     * arm_rms_q15 returns value in [0, 32767]; normalize to [0.0, 1.0].     */
    int16_t rms_q15;
    arm_rms_q15(in, (uint32_t)len, &rms_q15);
    float frame_rms = (float)rms_q15 / 32767.0f;

    /* Fast attack if signal is louder than envelope, slow release otherwise  */
    float alpha = (frame_rms > fe->agc_envelope) ? AGC_ALPHA_ATTACK
                                                  : AGC_ALPHA_RELEASE;
    fe->agc_envelope = alpha * frame_rms + (1.0f - alpha) * fe->agc_envelope;

    /* Desired gain to reach target RMS. Clamp to [MIN, MAX].                */
    float desired_gain = (fe->agc_envelope > 1e-6f)
                         ? (AUDIO_FRONTEND_AGC_TARGET / fe->agc_envelope)
                         : AUDIO_FRONTEND_AGC_MAX;
    desired_gain = CLAMP(desired_gain,
                         AUDIO_FRONTEND_AGC_MIN,
                         AUDIO_FRONTEND_AGC_MAX);

    /* Smooth gain to avoid zipper noise on rapid level changes              */
    fe->agc_gain = AGC_ALPHA_SMOOTH * desired_gain
                 + (1.0f - AGC_ALPHA_SMOOTH) * fe->agc_gain;

    /* Combined scale: calibration × AGC gain                                */
    float total_gain = fe->cal_gain * fe->agc_gain;

    /* ── Steps 2+3: DC removal + gain, sample by sample ─────────────────────
     * y[n] = α*y[n-1] + x[n] - x[n-1]    (IIR HPF)
     * Scaled by total_gain before converting back to Q15.
     * The loop is small (320 samples) and Cortex-M4F FPU handles it in
     * ~320×8 = 2560 cycles ≈ 40µs, well within the 20ms frame budget.      */
    float dc_x_prev = fe->dc_x_prev;
    float dc_y_prev = fe->dc_y_prev;

    for (size_t i = 0; i < len; i++) {
        float x = (float)in[i];

        /* DC removal: HPF difference equation                               */
        float y = DC_ALPHA * dc_y_prev + x - dc_x_prev;
        dc_x_prev = x;
        dc_y_prev = y;

        /* Apply combined gain and convert to Q15 with saturation.
         *
         * __SSAT(val, 16) clamps val to [-32768, 32767] — one ARM SSAT
         * instruction (1 cycle) vs. two float comparisons + conditional
         * assigns (~6 cycles with mispredicts on Cortex-M4F).
         *
         * Safety of (int32_t) cast: |y| ≤ 2 × 32768 = 65536 (HPF peak
         * gain ≤ 2× for a step from +32767 to −32768).
         * total_gain = cal_gain × agc_gain ≤ 2.0 × 4.0 = 8.0.
         * Worst case: 65536 × 8 = 524,288 ≪ INT32_MAX (2.1 × 10^9).
         * The cast is well-defined; no UB.
         *
         * NaN guard: cal_gain is clamped in audio_frontend_init(); agc_gain
         * evolves from 1.0f via bounded EMA — cannot become NaN from valid
         * Q15 input.  No NaN can reach total_gain.                          */
        out[i] = (int16_t)__SSAT((int32_t)(y * total_gain), 16U);
    }

    /* Write back DC state — persists to next frame                          */
    fe->dc_x_prev = dc_x_prev;
    fe->dc_y_prev = dc_y_prev;

    return 0;
}
