#ifndef RESONATOR_DF2T_H
#define RESONATOR_DF2T_H

#include <stdint.h>
#include <stddef.h>
#include <arm_math.h>

#define FRAME_SIZE 320
#define NUM_RESONATORS 4
#define NUM_STAGES 1

typedef struct {
    arm_biquad_cascade_df2T_instance_f32 channels[NUM_RESONATORS];
    float32_t state[NUM_RESONATORS][2 * NUM_STAGES]; /* DF2T needs 2 values per stage */
    int16_t outputs[NUM_RESONATORS][FRAME_SIZE];     /* Q15 output for feature functions */
} resonator_bank_df2t_t;

int resonator_bank_df2t_init(resonator_bank_df2t_t *bank);
void resonator_bank_df2t_process(resonator_bank_df2t_t *bank, const int16_t *input, size_t len);
void resonator_bank_df2t_reset(resonator_bank_df2t_t *bank);
const int16_t* resonator_bank_df2t_get_output(const resonator_bank_df2t_t *bank, int channel);
uint16_t resonator_bank_df2t_get_center_freq(int channel);

#endif
