#include "guardian/resonator_df2t.h"
#include "resonator_coefs_cmsis.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <arm_math.h>

static int16_t state_buffers[NUM_RESONATORS][4];

int resonator_bank_df2t_init(resonator_bank_df2t_t *bank)
{
    if (bank == NULL) return -EINVAL;
    memset(bank, 0, sizeof(resonator_bank_df2t_t));
    memset(state_buffers, 0, sizeof(state_buffers));
    
    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        arm_biquad_cascade_df1_init_q15(
            &bank->channels[ch],
            NUM_STAGES,
            (int16_t*)RESONATOR_COEFS[ch],
            state_buffers[ch],
            2
        );
    }
    printk("Resonator initialized\n");
    return 0;
}

void resonator_bank_df2t_process(resonator_bank_df2t_t *bank, const int16_t *input, size_t len)
{
    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        arm_biquad_cascade_df1_q15(&bank->channels[ch], (int16_t*)input, bank->outputs[ch], len);
    }
}

void resonator_bank_df2t_reset(resonator_bank_df2t_t *bank) { memset(state_buffers, 0, sizeof(state_buffers)); }
const int16_t* resonator_bank_df2t_get_output(const resonator_bank_df2t_t *bank, int channel) { return bank->outputs[channel]; }
uint16_t resonator_bank_df2t_get_center_freq(int channel) { return RESONATOR_CENTER_FREQS[channel]; }
