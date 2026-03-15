#ifndef RESONATOR_COEFS_CMSIS_H
#define RESONATOR_COEFS_CMSIS_H
#include <arm_math.h>

#define NUM_RESONATORS 4
#define NUM_STAGES     1

/*
 * Bandpass resonator coefficients for arm_biquad_cascade_df2T_f32.
 * Format per stage: {b0, b1, b2, -a1, -a2}
 *
 * H(z) = b0*(1 - z^{-2}) / (1 - 2r*cos(w0)*z^{-1} + r^2*z^{-2})
 *
 * CMSIS stores a coefficients negated:
 *   stored -a1 = +2r*cos(w0)   (positive)
 *   stored -a2 = -r^2          (negative)
 *
 * Root cause of previous bug: Q15 range is [-1,+1]. For f0 < 4000 Hz,
 * 2r*cos(w0) > 1.0 and cannot be stored in Q15 — it was silently clipped
 * to 32767 (1.0), placing all resonators at ~2650 Hz regardless of label.
 * Float has no such range constraint.
 *
 * fs = 16000 Hz. Pole radii match original design intent (higher Q for
 * lower frequencies). b0 set to half unity-gain to avoid saturation.
 *
 *   300 Hz : r=0.9927, 2r*cos(w0)=1.9717, b0=0.0036
 *   800 Hz : r=0.9806, 2r*cos(w0)=1.8650, b0=0.0093
 *  1500 Hz : r=0.9639, 2r*cos(w0)=1.6029, b0=0.0177
 *  2500 Hz : r=0.9404, 2r*cos(w0)=1.0451, b0=0.0289
 */

static const float32_t RESONATOR_0_COEFS[5] = {
    0.0073f, 0.0f, -0.0073f,  1.9717f, -0.9855f   /* 300 Hz */
};
static const float32_t RESONATOR_1_COEFS[5] = {
    0.0187f, 0.0f, -0.0187f,  1.8650f, -0.9616f   /* 800 Hz */
};
static const float32_t RESONATOR_2_COEFS[5] = {
    0.0355f, 0.0f, -0.0355f,  1.6029f, -0.9291f   /* 1500 Hz */
};
static const float32_t RESONATOR_3_COEFS[5] = {
    0.0578f, 0.0f, -0.0578f,  1.0451f, -0.8844f   /* 2500 Hz */
};

static const float32_t * const RESONATOR_COEFS[NUM_RESONATORS] = {
    RESONATOR_0_COEFS, RESONATOR_1_COEFS, RESONATOR_2_COEFS, RESONATOR_3_COEFS
};

static const uint16_t RESONATOR_CENTER_FREQS[NUM_RESONATORS] = {300, 800, 1500, 2500};

#endif /* RESONATOR_COEFS_CMSIS_H */
