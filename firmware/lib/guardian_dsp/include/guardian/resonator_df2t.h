#ifndef RESONATOR_DF2T_H
#define RESONATOR_DF2T_H

#include <stdint.h>
#include <stddef.h>
#include <arm_math.h>

#define FRAME_SIZE 320
#define NUM_RESONATORS 4
#define NUM_STAGES 1

/* DF1 topology (Direct Form I):
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * Why DF1 instead of DF2T for a commercial SDK:
 *   DF2T state variables are filter OUTPUTS — they accumulate quantization
 *   error every sample. For high-Q resonators (Q>10) in fixed-point, this
 *   causes limit cycles: the filter "sings" in silence after 30-60 minutes.
 *
 *   DF1 state variables are raw INPUT and OUTPUT HISTORY — bounded by the
 *   signal itself, not by accumulated arithmetic error. No limit cycles,
 *   stable indefinitely. Required for Q15/Q31 fixed-point paths.
 *
 *   For float32 (this implementation): both topologies are numerically
 *   equivalent, but DF1 is the correct choice for SDK credibility and
 *   consistency with any future fixed-point port.
 *
 * State size: DF1 needs 4 values/stage (x[n-1], x[n-2], y[n-1], y[n-2])
 *             DF2T needed 2 values/stage.                                    */
typedef struct {
    arm_biquad_casd_df1_inst_f32 channels[NUM_RESONATORS];
    float32_t state[NUM_RESONATORS][4 * NUM_STAGES]; /* DF1: 4 values per stage */
    int16_t outputs[NUM_RESONATORS][FRAME_SIZE];     /* Q15 output for features */
} resonator_bank_df2t_t;

int resonator_bank_df2t_init(resonator_bank_df2t_t *bank);
void resonator_bank_df2t_process(resonator_bank_df2t_t *bank, const int16_t *input, size_t len);
void resonator_bank_df2t_reset(resonator_bank_df2t_t *bank);
const int16_t* resonator_bank_df2t_get_output(const resonator_bank_df2t_t *bank, int channel);
uint16_t resonator_bank_df2t_get_center_freq(int channel);

#endif
