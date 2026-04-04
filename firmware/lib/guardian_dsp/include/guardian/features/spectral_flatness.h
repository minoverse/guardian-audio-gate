#ifndef GUARDIAN_SPECTRAL_FLATNESS_H
#define GUARDIAN_SPECTRAL_FLATNESS_H

#include <stdint.h>
#include "guardian/features/energy.h"

/* ── Spectral Flatness Measure (SFM) ─────────────────────────────────────────
 * Wiener entropy: ratio of geometric mean to arithmetic mean of sub-band RMS.
 *
 *   SFM = (∏ energy[i])^(1/N) / ((∑ energy[i]) / N)
 *
 * Range [0.0, 1.0]:
 *   SFM → 0.0 : tonal / harmonic (speech, pure tone)
 *   SFM → 1.0 : spectrally flat (white noise, broadband)
 *
 * Typical values:
 *   Speech (voiced)    : 0.05 – 0.35   ← harmonic formant structure
 *   Traffic / HVAC     : 0.65 – 0.90   ← broadband, flat
 *   White noise        : 0.90 – 1.00
 *   Silence / very low : undefined (returns 1.0 — treated as non-speech)
 *
 * Implementation: float (4 values, called once per frame — negligible cost).
 * ─────────────────────────────────────────────────────────────────────────── */

float compute_spectral_flatness(const energy_features_t *energy);

#endif /* GUARDIAN_SPECTRAL_FLATNESS_H */
