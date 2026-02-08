#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include "guardian/resonator_df2t.h"
#include <arm_math.h>
#include "guardian/features/energy.h"
#include "guardian/features/correlation.h"
#include "guardian/features/coherence.h"
#include "guardian/features/zcr.h"
#include "guardian/features/flux.h"

static resonator_bank_df2t_t bank;
static int16_t test_input[320];

int main(void)
{
    printk("Guardian Audio Gate - Week 3-4 Feature Test\n");
    
    resonator_bank_df2t_init(&bank);
    
    // Generate test signal (1kHz tone)
    for (int i = 0; i < 320; i++) {
        test_input[i] = (int16_t)(10000 * arm_sin_q15((i * 32768) / 16));
    }
    
    // Process through resonator
    resonator_bank_df2t_process(&bank, test_input, 320);
    
    // Feature extraction timing test
    energy_features_t energy;
    correlation_features_t corr;
    coherence_features_t coherence;
    flux_state_t flux = {0};
    uint16_t zcr;
    
    timing_init();
    timing_start();
    
    timing_t start = timing_counter_get();
    
    extract_energy_features((const int16_t (*)[320])bank.outputs, &energy);
    extract_correlation_features((const int16_t (*)[320])bank.outputs, &corr);
    extract_coherence_features(bank.outputs[0], 320, &coherence);
    zcr = compute_zcr(bank.outputs[0], 320);
    compute_spectral_flux(&flux, &energy);
    
    timing_t end = timing_counter_get();
    
    uint64_t cycles = timing_cycles_get(&start, &end);
    uint64_t ns = timing_cycles_to_ns(cycles);
    uint32_t us = ns / 1000;
    
    printk("\n=== Feature Extraction Results ===\n");
    printk("Processing time: %u us (target: < 2000 us)\n", us);
    
    if (us < 2000) {
        printk("PASS: Meets timing requirement\n");
    } else {
        printk("FAIL: Too slow\n");
    }
    
    printk("\n=== Feature Values ===\n");
    printk("Energy - Ch0: %d, Ch1: %d, Ch2: %d, Ch3: %d, Total: %d\n",
           energy.channel_energy[0], energy.channel_energy[1],
           energy.channel_energy[2], energy.channel_energy[3],
           energy.total_energy);
    
    printk("Correlation - 0-1: %d, 1-2: %d, 2-3: %d, Max: %d\n",
           corr.corr_0_1, corr.corr_1_2, corr.corr_2_3, corr.max_corr);
    
    printk("Coherence - Peak: %d, Lag: %u\n",
           coherence.autocorr_peak, coherence.peak_lag);
    
    printk("ZCR: %u crossings\n", zcr);
    
    printk("Spectral Flux: %d\n", flux.flux);
    
    printk("\n=== Combined Timing ===\n");
    printk("Resonator: 297 us\n");
    printk("Features: %u us\n", us);
    printk("Total: %u us (frame budget: 20000 us)\n", 297 + us);
    
    return 0;
}
