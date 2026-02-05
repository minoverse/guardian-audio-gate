#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include "guardian/resonator_df2t.h"

static resonator_bank_df2t_t bank;
static int16_t test_input[320];

int main(void)
{
    printk("Guardian Audio Gate - Week 1-2 Timing Test\n");
    
    resonator_bank_df2t_init(&bank);
    
    // Generate test signal
    for (int i = 0; i < 320; i++) {
        test_input[i] = (i % 100) * 100;
    }
    
    timing_init();
    timing_start();
    
    timing_t start = timing_counter_get();
    resonator_bank_df2t_process(&bank, test_input, 320);
    timing_t end = timing_counter_get();
    
    uint64_t cycles = timing_cycles_get(&start, &end);
    uint64_t ns = timing_cycles_to_ns(cycles);
    uint32_t us = ns / 1000;
    
    printk("Resonator timing: %u us (target: < 5000 us)\n", us);
    
    if (us < 5000) {
        printk("PASS: Meets timing requirement\n");
    } else {
        printk("FAIL: Too slow\n");
    }
    
    return 0;
}
