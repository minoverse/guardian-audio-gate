#include "guardian/resonator_df2t.h"
#include "resonator_coefs_cmsis.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <arm_math.h>

/* Static conversion buffers — avoids large stack allocations */
static float32_t s_float_input[FRAME_SIZE];
static float32_t s_float_output[FRAME_SIZE];

int resonator_bank_df2t_init(resonator_bank_df2t_t *bank)
{
    if (bank == NULL) return -EINVAL;
    memset(bank, 0, sizeof(resonator_bank_df2t_t));

    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        arm_biquad_cascade_df2T_init_f32(
            &bank->channels[ch],
            NUM_STAGES,
            RESONATOR_COEFS[ch],
            bank->state[ch]
        );
    }
    printk("Resonator initialized\n");
    return 0;
}

void resonator_bank_df2t_process(resonator_bank_df2t_t *bank, const int16_t *input, size_t len)
{
    /* Q15 → float [-1.0, 1.0] */
    for (size_t i = 0; i < len; i++) {
        s_float_input[i] = (float32_t)input[i] / 32768.0f;
    }

    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        arm_biquad_cascade_df2T_f32(
            &bank->channels[ch], s_float_input, s_float_output, (uint32_t)len);

        /* float → Q15, clip to [-32768, 32767] */
        for (size_t i = 0; i < len; i++) {
            float32_t v = s_float_output[i] * 32768.0f;
            if      (v >  32767.0f) v =  32767.0f;
            else if (v < -32768.0f) v = -32768.0f;
            bank->outputs[ch][i] = (int16_t)v;
        }
    }
}

void resonator_bank_df2t_reset(resonator_bank_df2t_t *bank)
{
    memset(bank->state, 0, sizeof(bank->state));
}

const int16_t* resonator_bank_df2t_get_output(const resonator_bank_df2t_t *bank, int channel)
{
    return bank->outputs[channel];
}

uint16_t resonator_bank_df2t_get_center_freq(int channel)
{
    return RESONATOR_CENTER_FREQS[channel];
}
