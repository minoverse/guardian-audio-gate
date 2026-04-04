/* prefilter.c — Guardian Physics Pre-Filter SDK implementation
 *
 * Pure C99 + math.h.  No platform, OS, CMSIS, or RTOS dependencies.
 * Compiles on Cortex-M4F (nRF52840, STM32), QCC51xx (Qualcomm Bluetooth SoC),
 * x86-64 (Linux/macOS host tests), and any platform with IEEE 754 float32.
 *
 * Copyright (c) 2026 Guardian Audio. All rights reserved.
 * SPDX-License-Identifier: Proprietary                                  */

#include "guardian/sdk/prefilter.h"
#include "resonator_coefs_default.h"
#include <math.h>    /* sqrtf, logf, expf — IEEE 754, FPU-accelerated on M4F */
#include <string.h>  /* memset */

/* ── AGC time constants ──────────────────────────────────────────────────
 * Frame rate = 50 frames/s (20 ms per frame, 320 samples @ 16 kHz).
 * α_frame = 1 − exp(−dt/τ) for dt = 20 ms:
 *
 *   Attack  τ = 10 ms   → α = 1 − exp(−20/10)   = 0.8647  (fast: no clip)
 *   Release τ = 500 ms  → α = 1 − exp(−20/500)  = 0.0392  (slow: no pump)
 *   Smoother τ = 50 ms  → α = 1 − exp(−20/50)   = 0.3297  (no zipper)
 *
 * Derivation: bilinear-transform HPF at fc Hz, fs = sample_rate_hz:
 *   α = 1 − 2π·fc / fs  (small-angle approximation, valid for fc ≪ fs/2) */
#define AGC_ALPHA_ATTACK   0.8647f
#define AGC_ALPHA_RELEASE  0.0392f
#define AGC_ALPHA_SMOOTH   0.3297f

/* Clamp helper — pure C99, no dependencies */
static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ── guardian_pf_init ──────────────────────────────────────────────── */
int guardian_pf_init(guardian_pf_t *pf, const guardian_pf_config_t *cfg)
{
    if (!pf || !cfg || cfg->sample_rate_hz == 0U) return -1;

    memset(pf, 0, sizeof(*pf));

    /* ── Resonator bank ─────────────────────────────────────────────── */
    const guardian_pf_coef_t *coefs =
        cfg->coefs ? cfg->coefs : GUARDIAN_PF_DEFAULT_COEFS;
    const uint16_t *freqs =
        cfg->center_freqs_hz ? cfg->center_freqs_hz : GUARDIAN_PF_DEFAULT_FREQS;

    for (int ch = 0; ch < GUARDIAN_PF_NUM_CHANNELS; ch++) {
        gd_biquad_df1_init(&pf->_filters[ch], coefs[ch].coef);
        pf->_center_freqs[ch] = freqs[ch];
    }

    /* ── DC removal ─────────────────────────────────────────────────── */
    if (cfg->dc_hpf_hz > 0.0f) {
        /* α = 1 − 2π·fc / fs  (1st-order IIR HPF pole)               */
        pf->_dc_alpha   = 1.0f - (2.0f * 3.14159265f * cfg->dc_hpf_hz)
                                  / (float)cfg->sample_rate_hz;
        pf->_dc_enabled = 1;
    }

    /* ── AGC ─────────────────────────────────────────────────────────── */
    if (cfg->agc_target_rms > 0.0f) {
        pf->_agc_target  = cfg->agc_target_rms;
        pf->_agc_min     = cfg->agc_gain_min;
        pf->_agc_max     = cfg->agc_gain_max;
        pf->_agc_envelope = cfg->agc_target_rms;  /* start at target, not 0 */
        pf->_agc_gain    = 1.0f;
        pf->_agc_enabled = 1;
    } else {
        pf->_agc_gain    = 1.0f;
    }

    /* ── Calibration gain ───────────────────────────────────────────── */
    pf->_cal_gain = (cfg->cal_gain > 0.0f) ? cfg->cal_gain : 1.0f;
    pf->_sfm      = 1.0f;

    return 0;
}

/* ── Internal: compute frame RMS (Q15 input, returns [0.0, 1.0]) ──── */
static float frame_rms_q15(const int16_t *in, size_t len)
{
    float sum_sq = 0.0f;
    for (size_t i = 0; i < len; i++) {
        float s = (float)in[i];
        sum_sq += s * s;
    }
    /* normalise by 32767² so result is in [0.0, 1.0]                  */
    return sqrtf(sum_sq / ((float)len * 32767.0f * 32767.0f));
}

/* ── Internal: compute channel energy (sum of squared Q15 samples) ── */
static uint32_t channel_energy(const int16_t *ch, size_t len)
{
    uint32_t e = 0;
    for (size_t i = 0; i < len; i++) {
        int32_t s = ch[i];
        e += (uint32_t)(s * s) >> 10;   /* right-shift prevents uint32 overflow
                                          * for 320 samples at full scale:
                                          * 320 × 32767² >> 10 ≈ 334M < 2³²   */
    }
    return e;
}

/* ── Internal: spectral flatness from 4-channel energies ──────────── */
static float compute_sfm(const uint32_t e[GUARDIAN_PF_NUM_CHANNELS])
{
    /* Convert to float; +1 avoids log(0) on silent channels.          */
    float ef[GUARDIAN_PF_NUM_CHANNELS];
    float arith_sum = 0.0f;
    float log_sum   = 0.0f;

    for (int i = 0; i < GUARDIAN_PF_NUM_CHANNELS; i++) {
        ef[i]      = (float)e[i] + 1.0f;
        arith_sum += ef[i];
        log_sum   += logf(ef[i]);
    }

    float arith_mean = arith_sum / (float)GUARDIAN_PF_NUM_CHANNELS;

    /* Near-silence: all channels essentially zero.
     * Return 1.0 (flat/noise) — silent frames must not pass a speech gate. */
    if (arith_mean < 2.0f) return 1.0f;

    float geom_mean = expf(log_sum / (float)GUARDIAN_PF_NUM_CHANNELS);
    float sfm = geom_mean / arith_mean;

    return clampf(sfm, 0.0f, 1.0f);
}

/* ── guardian_pf_process ───────────────────────────────────────────── */
void guardian_pf_process(guardian_pf_t *pf, const int16_t *in, size_t len)
{
    if (!pf || !in || len == 0 || len > GUARDIAN_PF_FRAME_SAMPLES) return;

    /* ── Step 1: AGC envelope update ───────────────────────────────── */
    if (pf->_agc_enabled) {
        float rms = frame_rms_q15(in, len);
        float alpha = (rms > pf->_agc_envelope) ? AGC_ALPHA_ATTACK
                                                 : AGC_ALPHA_RELEASE;
        pf->_agc_envelope = alpha * rms + (1.0f - alpha) * pf->_agc_envelope;

        float desired = (pf->_agc_envelope > 1e-6f)
                        ? (pf->_agc_target / pf->_agc_envelope)
                        : pf->_agc_max;
        desired = clampf(desired, pf->_agc_min, pf->_agc_max);

        pf->_agc_gain = AGC_ALPHA_SMOOTH * desired
                      + (1.0f - AGC_ALPHA_SMOOTH) * pf->_agc_gain;
    }

    /* ── Step 2: DC removal + combined gain, float working buffer ─── */
    float total_gain = pf->_cal_gain * pf->_agc_gain;

    /* Reuse _outputs[0] as a float-sized scratch? No — outputs are int16.
     * Use a static float buffer; acceptable for non-reentrant embedded use. */
    static float s_preprocessed[GUARDIAN_PF_FRAME_SAMPLES];

    if (pf->_dc_enabled) {
        float alpha   = pf->_dc_alpha;
        float x_prev  = pf->_dc_x_prev;
        float y_prev  = pf->_dc_y_prev;

        for (size_t i = 0; i < len; i++) {
            float x = (float)in[i];
            float y = alpha * y_prev + x - x_prev;
            x_prev = x;
            y_prev = y;
            /* apply gain and normalise to [-1, 1] for resonator        */
            s_preprocessed[i] = (y * total_gain) / 32768.0f;
        }

        pf->_dc_x_prev = x_prev;
        pf->_dc_y_prev = y_prev;
    } else {
        for (size_t i = 0; i < len; i++) {
            s_preprocessed[i] = ((float)in[i] * total_gain) / 32768.0f;
        }
    }

    /* ── Step 3: resonator bank ─────────────────────────────────────── */
    static float s_ch_out[GUARDIAN_PF_FRAME_SAMPLES];
    uint32_t energies[GUARDIAN_PF_NUM_CHANNELS];

    for (int ch = 0; ch < GUARDIAN_PF_NUM_CHANNELS; ch++) {
        gd_biquad_df1_block(&pf->_filters[ch], s_preprocessed, s_ch_out, len);

        /* float → Q15 with saturation; accumulate energy               */
        uint32_t e = 0;
        for (size_t i = 0; i < len; i++) {
            float v = s_ch_out[i] * 32768.0f;
            if      (v >  32767.0f) v =  32767.0f;
            else if (v < -32768.0f) v = -32768.0f;
            int16_t q = (int16_t)v;
            pf->_outputs[ch][i] = q;
            int32_t s = q;
            e += (uint32_t)(s * s) >> 10;
        }
        energies[ch] = e;
    }

    /* ── Step 4: spectral flatness ───────────────────────────────────── */
    pf->_sfm      = compute_sfm(energies);
    pf->_last_len = len;
}

/* ── Accessors ──────────────────────────────────────────────────────── */
const int16_t *guardian_pf_get_channel(const guardian_pf_t *pf, int ch)
{
    if (!pf || ch < 0 || ch >= GUARDIAN_PF_NUM_CHANNELS) return NULL;
    return pf->_outputs[ch];
}

float guardian_pf_get_sfm(const guardian_pf_t *pf)
{
    return pf ? pf->_sfm : 1.0f;
}

uint16_t guardian_pf_get_center_freq(const guardian_pf_t *pf, int ch)
{
    if (!pf || ch < 0 || ch >= GUARDIAN_PF_NUM_CHANNELS) return 0;
    return pf->_center_freqs[ch];
}

void guardian_pf_reset(guardian_pf_t *pf)
{
    if (!pf) return;
    for (int ch = 0; ch < GUARDIAN_PF_NUM_CHANNELS; ch++) {
        gd_biquad_df1_reset(&pf->_filters[ch]);
    }
    pf->_dc_x_prev    = 0.0f;
    pf->_dc_y_prev    = 0.0f;
    pf->_agc_envelope = pf->_agc_target > 0.0f ? pf->_agc_target : 0.25f;
    pf->_agc_gain     = 1.0f;
    pf->_sfm          = 1.0f;
}

void guardian_pf_set_cal_gain(guardian_pf_t *pf, float cal_gain)
{
    if (!pf || cal_gain <= 0.0f) return;
    pf->_cal_gain = cal_gain;
}
