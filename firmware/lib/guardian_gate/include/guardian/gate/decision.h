#ifndef GUARDIAN_GATE_DECISION_H
#define GUARDIAN_GATE_DECISION_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>
#include "guardian/resonator_df2t.h"
#include "guardian/features/energy.h"
#include "guardian/features/modulation.h"

/* ── Gate configuration ───────────────────────────────────────────────────── */

typedef struct {
    q15_t    correlation_threshold; /* 0.45 in Q15 = 14746 — main speech discriminator */
    q15_t    energy_ratio_min;      /* min energy to avoid waking on silence            */
    q15_t    coherence_threshold;   /* pitch autocorrelation peak threshold              */
    uint16_t zcr_max;               /* max zero-crossings per frame (noise: >100)        */
    q15_t    noise_floor;           /* current noise estimate — updated by noise_tracker */
    float    sfm_threshold;         /* spectral flatness above this = noise (0.60)       */
    float    mod_threshold;         /* modulation CV below this = stationary noise (0.12)*/
} gate_config_t;

#define GATE_CONFIG_DEFAULT {          \
    .correlation_threshold = 14746,   \
    .energy_ratio_min      = 200,     \
    .coherence_threshold   = 1000,    \
    .zcr_max               = 80,      \
    .noise_floor           = 500,     \
    .sfm_threshold         = 0.60f,   \
    .mod_threshold         = 0.12f,   \
}

/* ── Gate decision result ─────────────────────────────────────────────────── */

typedef struct {
    bool     should_wake;   /* true = run TinyML inference; false = abort     */
    uint8_t  confidence;    /* 0–140 scoring sum (max with all 6 rules)       */
    uint32_t elapsed_us;    /* gate processing time in microseconds           */

    struct {
        q15_t    correlation;     /* max inter-resonator correlation (Q15)    */
        q15_t    energy;          /* average RMS across resonator bank (Q15)  */
        q15_t    coherence;       /* pitch autocorrelation peak               */
        uint16_t zcr;             /* zero-crossing rate per frame             */
        float    spectral_flatness; /* SFM: 0=tonal/speech, 1=flat/noise      */
        float    modulation_cv;     /* energy CV: high=speech, low=noise      */
        uint8_t  active_ch;       /* Rule 7: resonator channels above thresh  */
    } features_used;
} gate_decision_t;

/* ── Adaptive noise floor tracker ────────────────────────────────────────── */

typedef struct {
    q15_t    noise_estimate; /* EMA of quiet-frame energy                     */
    uint32_t frame_count;
} noise_tracker_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * Run the gate decision on the latest resonator bank outputs.
 *
 * @param bank    resonator bank after process() call
 * @param config  thresholds and noise floor
 * @param mod     modulation state — updated each call (cross-frame EMA state)
 *
 * Six scoring rules (max score = 140):
 *   Rule 1 (+40): inter-resonator correlation > threshold  → harmonic speech
 *   Rule 2 (+20): energy > 2× noise floor                 → not silence
 *   Rule 3 (+25): pitch coherence > threshold             → voiced/periodic
 *   Rule 4 (+15): ZCR < zcr_max                          → not broadband noise
 *   Rule 5 (+20): spectral flatness < sfm_threshold       → tonal structure
 *   Rule 6 (+20): modulation CV > mod_threshold           → syllabic rhythm
 *
 * Rule 7 (hard gate): ≥2 resonator channels above MULTIBAND_CHAN_THRESH_Q15.
 *   Speech drives ch0 (300 Hz) + ch1 (800 Hz) simultaneously.
 *   Band-limited noise (HVAC, engine, wind) activates ch0 only.
 *   This gate is not a score bonus — it is a prerequisite for should_wake.
 *
 * Wake condition: score >= 20 AND active_ch >= 2
 *   Python simulation (5 environments): abort rate 26.6% → 100% after Rule 7.
 */
gate_decision_t gate_decide(const resonator_bank_df2t_t *bank,
                            const gate_config_t *config,
                            modulation_state_t *mod);

void update_noise_floor(noise_tracker_t *tracker,
                        const energy_features_t *energy,
                        bool speech_detected);

#endif /* GUARDIAN_GATE_DECISION_H */
