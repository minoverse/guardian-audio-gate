/* guardian/sdk/biquad_df1.h — Portable Direct Form I biquad
 *
 * Part of the Guardian Physics Pre-Filter SDK.
 * Pure C99 — no platform, OS, or CMSIS dependencies.
 *
 * Copyright (c) 2026 Guardian Audio. All rights reserved.
 * SPDX-License-Identifier: Proprietary
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Why Direct Form I for this SDK:
 *
 *   DF2T (transposed direct form II) state variables are delayed filter
 *   OUTPUTS — they accumulate quantization error every sample.  For high-Q
 *   resonators (Q > 10) in fixed-point arithmetic (Q15/Q31), this causes
 *   limit cycles: the filter "sings" at its natural frequency during silence
 *   after 30–60 minutes of operation.  Embedded hearable devices run
 *   continuously; limit cycles are a field defect, not a lab artefact.
 *
 *   DF1 state variables are raw INPUT and OUTPUT HISTORY — bounded by the
 *   signal itself, not by accumulated arithmetic error.  No limit cycles.
 *   Stable over the device lifetime.  This is the correct topology for a
 *   licensed SDK targeting always-on audio pipelines.
 *
 *   float32:  Both topologies are numerically equivalent.  DF1 is used here
 *             for SDK credibility and to guarantee a correct fixed-point port.
 *
 * Coefficient convention:
 *   {b0, b1, b2, -a1, -a2}  — denominator coefficients are NEGATED.
 *   This is the CMSIS arm_biquad_cascade_df1_f32 convention, so coefficient
 *   tables designed for CMSIS can be used without modification.
 *
 *   Difference equation:
 *     y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2]
 *           + na1·y[n-1] + na2·y[n-2]
 *   where na1 = −a1, na2 = −a2.
 *
 * ── Numerical range and overflow analysis (headroom budget) ─────────────
 *
 * Implementation arithmetic: IEEE 754 float32 (NOT Q15/Q31 fixed-point).
 *
 * float32 overflow bound — worst case: all 5 products add constructively:
 *   |y| ≤ |b0|·|x| + |b1|·|x1| + |b2|·|x2| + |na1|·|y1| + |na2|·|y2|
 *
 * Guardian resonator coefficients (bandpass, b1 = 0):
 *   b0, b2 ≤ 0.06,  na1 ≤ 1.97 (pole radius 0.9927),  na2 ≤ 0.99
 *
 * float32 exponent range ±10^38 — trivially accommodates any audio signal.
 * Guard bits are NOT required; the IEEE 754 accumulator cannot wrap.
 *
 * Contrast with Q31 biquad (NOT used here):
 *   An int32_t accumulator WOULD overflow for 5 × INT32_MAX.  A Q31
 *   implementation MUST use int64_t for the inner product:
 *     int64_t acc = (int64_t)b0_q31 * x_q31;   // 64-bit accumulation
 *   This concern does NOT apply to this float32 implementation.
 *
 * AGC headroom: audio_frontend applies gain ∈ [0.25, 4.0] before this
 *   filter. At max gain 4× on a full-scale int16 input (32767):
 *   4 × 32767 / 32768 = 3.999 — within float32 range, clamped to Q15 by
 *   the frontend before the resonator bank sees it.
 *
 * Startup transient: states (x1, x2, y1, y2) initialise to 0.  For
 *   high-Q resonators this produces a ~10-frame transient.  The PDM driver
 *   discards 10 warmup frames before yielding data to the gate.
 * ─────────────────────────────────────────────────────────────────────── */

#ifndef GUARDIAN_SDK_BIQUAD_DF1_H
#define GUARDIAN_SDK_BIQUAD_DF1_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Filter state ──────────────────────────────────────────────────────── */
typedef struct {
    float b0, b1, b2;    /* feedforward coefficients                       */
    float na1, na2;      /* negated denominator: na1=−a1, na2=−a2          */
    float x1, x2;        /* input history:  x[n−1], x[n−2]                */
    float y1, y2;        /* output history: y[n−1], y[n−2]                */
} gd_biquad_df1_t;

/* ── Initialise from CMSIS-convention coefficient array {b0,b1,b2,−a1,−a2} ── */
static inline void gd_biquad_df1_init(gd_biquad_df1_t *f, const float coef[5])
{
    f->b0  = coef[0]; f->b1  = coef[1]; f->b2  = coef[2];
    f->na1 = coef[3]; f->na2 = coef[4];
    f->x1  = 0.0f;    f->x2  = 0.0f;
    f->y1  = 0.0f;    f->y2  = 0.0f;
}

/* ── Zero filter state (coefficients unchanged) ─────────────────────── */
static inline void gd_biquad_df1_reset(gd_biquad_df1_t *f)
{
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

/* ── Process one sample ─────────────────────────────────────────────── */
static inline float gd_biquad_df1_sample(gd_biquad_df1_t *f, float x)
{
    float y = f->b0 * x  + f->b1 * f->x1 + f->b2 * f->x2
            + f->na1 * f->y1 + f->na2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

/* ── Process a block of samples (float input, float output) ────────── */
void gd_biquad_df1_block(gd_biquad_df1_t *f,
                         const float *in, float *out, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* GUARDIAN_SDK_BIQUAD_DF1_H */
