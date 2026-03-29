# Week 5 Debugging Log

## Issue Timeline

### Day 1: PDM Timeout
**Symptom:** `dma_pdm_read()` returns `-ETIMEDOUT`, no callbacks firing

**Debug steps:**
1. Added callback counter - confirmed handler never executes
2. Verified IRQ registration with `IRQ_CONNECT()` - correct
3. Checked PDM init return value - success (0)
4. Inspected `pdm_instance.p_reg` in debugger - **found NULL**

**Root cause:** `NRFX_PDM_INSTANCE(0)` macro interprets 0 as register address, not instance index. All PDM register writes went to address 0x0.

**Fix:**
```c
// Before
static nrfx_pdm_t pdm_instance = NRFX_PDM_INSTANCE(0);

// After
static nrfx_pdm_t pdm_instance = NRFX_PDM_INSTANCE(NRF_PDM_BASE);
```

### Day 2: 1.18 mA Flat Current
**Symptom:** PPK2 shows 1.18 mA, no periodic spikes, code appears running

**Analysis:** nRF52840 boots on HFINT (RC oscillator). PDM peripheral requires accurate HFXO for sample clock generation. Without HFXO:
- `NRF_PDM->PDMCLKCTRL` divider produces wrong frequency
- Microphone receives incorrect clock
- No DATA output → PDM DMA never triggers

**Fix:** Explicit HFXO start before PDM init
```c
static void hfclk_start(void)
{
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    while (!nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK)) {
        /* Wait for crystal stabilization (~400µs) */
    }
}
```

### Day 3: Division by Zero Fault
**Symptom:** `USAGE FAULT - Division by zero` at `r15/pc: 0x000070fc`

**Backtrace:** Fault occurred during `compute_correlation_q15()` execution

**Root cause:** Overflow in Q15 fixed-point variance calculation:
```c
int32_t var_x = 0;
for (...) {
    var_x += (dx * dx) >> 15;  // Can overflow int32_t
}
int32_t denom_sq = (var_x * var_y) >> 15;
int16_t denom;
arm_sqrt_q15((int16_t)denom_sq, &denom);  // Cast truncates → zero
return (cov << 15) / denom;  // Division by zero
```

**Fix:** Switch to `int64_t` accumulators + `float` for final step
```c
int64_t var_x = 0, var_y = 0;
for (...) {
    var_x += (int64_t)dx * dx;  // No overflow
}
float denom = sqrtf((float)var_x * (float)var_y);
if (denom < 1.0f) return 0;
float r = (float)cov / denom;
return (int16_t)(r * 32767.0f);
```

**Trade-off:** +7KB flash for floating-point math library. Acceptable given 1MB total.

### Day 4: nrfx API Mismatch
**Symptom:** Build errors on struct member access

**Changes in nrfx 3.x:**
1. `nrfx_pdm_irq_handler()` → `nrfx_pdm_irq_handler(nrfx_pdm_t *)`
2. `pdm_config.clock_freq` → `pdm_config.prescalers.clock_freq`
3. `pdm_config.ratio` → `pdm_config.prescalers.ratio`

**Lesson:** Always check nrfx version-specific API docs when porting code.

## Lessons Learned

1. **Macro arguments matter:** `NRFX_PDM_INSTANCE()` expects register base, not index
2. **Clock domains are critical:** PDM needs HFXO, not HFINT
3. **Fixed-point overflow is silent:** Use wider types for intermediate calculations
4. **Development boards have limitations:** SB40 prevents clean power measurement
5. **Real-time verification beats theory:** Actual audio testing found correlation overflow

## Measurement Limitation

**SB40 Issue:** nRF52840-DK routes USB power through solder bridge SB40 directly to VDD rail. When connected:
- USB supplies nRF VDD
- PPK2 cannot measure current (bypassed)
- Removing SB40 requires soldering tools

**Workaround attempted:** Li-Po mode (SW9 switch)
- Disconnects USB power
- But also disables UART (no code verification)
- J-Link must be disconnected (reflashing requires VDD mode)

**Decision:** Code verification > power number. Implemented and tested DMA architecture, deferred precision measurement.

