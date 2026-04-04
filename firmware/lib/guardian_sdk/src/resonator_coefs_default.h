/* resonator_coefs_default.h — Default bandpass resonator coefficients
 *
 * Private SDK header — not part of the public API.
 * Pure C99; no arm_math.h or CMSIS types.
 *
 * Four second-order bandpass resonators targeting speech formant regions.
 * Design: H(z) = b0·(1 − z^{−2}) / (1 − 2r·cos(ω₀)·z^{−1} + r²·z^{−2})
 *
 * Storage convention: {b0, b1, b2, −a1, −a2}  (CMSIS-compatible)
 *
 * fs = 16 000 Hz.  Pole radii give increasing Q for lower frequencies
 * (lower bands require higher Q to stay narrow relative to Nyquist).
 *
 *   Channel │  f₀ (Hz) │   r      │ 2r·cos(ω₀) │   b0
 *   ────────┼──────────┼──────────┼────────────┼───────
 *      0    │   300    │ 0.9927   │  1.9717    │ 0.0073
 *      1    │   800    │ 0.9806   │  1.8650    │ 0.0187
 *      2    │  1500    │ 0.9639   │  1.6029    │ 0.0355
 *      3    │  2500    │ 0.9404   │  1.0451    │ 0.0578
 *
 * b0 set to half-unity-gain (b0 = (1-r)/2 * normalisation) to avoid
 * saturation on full-scale inputs.
 *
 * Note on Q15 range failure (documented for future fixed-point port):
 *   For f₀ < 4 000 Hz, 2r·cos(ω₀) > 1.0.  Q15 range is [−1, +1].
 *   Storing these coefficients in Q15 silently clips to 32767 (1.0),
 *   placing all resonators at ~2650 Hz regardless of label.  This SDK
 *   uses float32 throughout; a fixed-point port must use Q31 for a1/a2.
 *
 * Copyright (c) 2026 Guardian Audio. All rights reserved.
 * SPDX-License-Identifier: Proprietary                                  */

#ifndef GUARDIAN_SDK_RESONATOR_COEFS_DEFAULT_H
#define GUARDIAN_SDK_RESONATOR_COEFS_DEFAULT_H

#include "guardian/sdk/prefilter.h"

static const guardian_pf_coef_t GUARDIAN_PF_DEFAULT_COEFS[GUARDIAN_PF_NUM_CHANNELS] = {
    {{ 0.0073f, 0.0f, -0.0073f,  1.9717f, -0.9855f }},  /* 300 Hz  */
    {{ 0.0187f, 0.0f, -0.0187f,  1.8650f, -0.9616f }},  /* 800 Hz  */
    {{ 0.0355f, 0.0f, -0.0355f,  1.6029f, -0.9291f }},  /* 1500 Hz */
    {{ 0.0578f, 0.0f, -0.0578f,  1.0451f, -0.8844f }},  /* 2500 Hz */
};

static const uint16_t GUARDIAN_PF_DEFAULT_FREQS[GUARDIAN_PF_NUM_CHANNELS] = {
    300, 800, 1500, 2500
};

#endif /* GUARDIAN_SDK_RESONATOR_COEFS_DEFAULT_H */
