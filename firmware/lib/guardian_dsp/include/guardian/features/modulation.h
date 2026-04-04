#ifndef GUARDIAN_MODULATION_H
#define GUARDIAN_MODULATION_H

#include <stdint.h>
#include <stdbool.h>

/* ── Energy Modulation Index (syllabic rate detector) ───────────────────────
 * Human speech has 3-5 syllables/second (natural 4Hz modulation of energy).
 * Continuous noise (HVAC, traffic) has stationary energy — little modulation.
 *
 * Metric: Coefficient of Variation (CV) = σ / μ of the energy envelope.
 *   CV = sqrt(E[(x-μ)²]) / E[x]
 *
 * Computed via exponential moving average (EMA) over ~500ms of frame history.
 * Cross-frame state — must be allocated by the caller and passed each frame.
 *
 * Typical values (frame energy = total_energy from energy_features_t):
 *   Speech (continuous)  : CV = 0.25 – 0.70   ← on/off syllabic pattern
 *   HVAC / fan noise     : CV = 0.02 – 0.10   ← stationary
 *   Transient (door slam): CV spike then drops — hold_frames prevents false
 *
 * Time constants:
 *   τ_mean = 500ms (25 frames @ 20ms) — tracks syllabic envelope
 *   τ_var  = 500ms — tracks variance at same rate
 *   α_frame = 1 - exp(-20ms/500ms) = 0.0392
 * ─────────────────────────────────────────────────────────────────────────── */

typedef struct {
    float mean_ema;   /* EMA of frame energy — long-term level estimate       */
    float var_ema;    /* EMA of (energy - mean)² — variance estimate          */
    bool  valid;      /* false until enough frames have been seen (warmup)    */
    int   warmup_n;   /* warmup frame counter                                 */
} modulation_state_t;

#define MODULATION_WARMUP_FRAMES 25   /* 500ms before CV estimate is reliable */

/* Initialize state — call once at startup.                                   */
void modulation_init(modulation_state_t *state);

/* Update state with this frame's total energy (Q15) and return current CV.
 * Returns 0.0f during warmup (first MODULATION_WARMUP_FRAMES calls).        */
float modulation_update(modulation_state_t *state, int16_t frame_energy_q15);

/* Reset state — call after PDM restart so stale history doesn't pollute CV. */
void modulation_reset(modulation_state_t *state);

#endif /* GUARDIAN_MODULATION_H */
