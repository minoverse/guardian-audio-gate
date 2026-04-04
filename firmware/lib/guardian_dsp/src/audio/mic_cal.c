#include "guardian/audio/mic_cal.h"
#include <nrfx_nvmc.h>
#include <string.h>
#include <zephyr/sys/printk.h>

/* ── CRC16-CCITT (poly=0x1021, init=0xFFFF) ──────────────────────────────────
 * Detects all 1-bit and 2-bit errors, all burst errors ≤16 bits.
 * Replaces the v1 XOR checksum which could not detect byte-swaps or
 * systematic bit patterns (e.g., gain=0.0 with magic=0x00000000 XOR=0).    */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Checksum covers magic + gain (first 8 bytes of mic_cal_t).
 * Stored as uint32_t (upper 16 bits zero) for natural word alignment.      */
static uint32_t cal_checksum(const mic_cal_t *c)
{
    return (uint32_t)crc16_ccitt((const uint8_t *)c, 8U);
}

bool mic_cal_is_valid(void)
{
    const mic_cal_t *cal = (const mic_cal_t *)MIC_CAL_FLASH_ADDR;

    if (cal->magic != MIC_CAL_MAGIC) {
        return false;   /* unprogrammed (0xFFFFFFFF) or garbage               */
    }
    if (cal->checksum != cal_checksum(cal)) {
        return false;   /* single-bit flip or partial write detected          */
    }
    /* Sanity-check gain range: ±6dB covers any reasonable mic variance.
     * Gains outside [0.5, 2.0] indicate corrupt data.
     *
     * NOTE: Use !(x >= lo && x <= hi) rather than (x < lo || x > hi).
     * NaN comparisons are always false in IEEE 754, so the OR form would
     * pass NaN silently.  The AND form correctly rejects NaN because
     * !(false && false) = true → returns false (invalid).                   */
    if (!(cal->gain >= 0.5f && cal->gain <= 2.0f)) {
        return false;
    }
    return true;
}

bool mic_cal_load(float *gain_out)
{
    if (!mic_cal_is_valid()) {
        printk("MIC_CAL: no valid calibration — using unity gain\n");
        *gain_out = 1.0f;
        return false;
    }
    const mic_cal_t *cal = (const mic_cal_t *)MIC_CAL_FLASH_ADDR;
    *gain_out = cal->gain;
    printk("MIC_CAL: loaded gain=%.4f from 0x%05X\n",
           (double)cal->gain, MIC_CAL_FLASH_ADDR);
    return true;
}

bool mic_cal_save(float gain)
{
    /* Validate before writing — bad gain would produce a corrupt unit        */
    if (gain < 0.5f || gain > 2.0f) {
        printk("MIC_CAL: gain %.4f out of range [0.5, 2.0] — rejected\n",
               (double)gain);
        return false;
    }

    mic_cal_t cal;
    cal.magic    = MIC_CAL_MAGIC;
    cal.gain     = gain;
    cal.checksum = cal_checksum(&cal);

    /* nRF52840 NVMC: must erase (set all bits to 1) before writing.
     * nrfx_nvmc_page_erase blocks until erase completes (~89ms for 4KB).    */
    nrfx_nvmc_page_erase(MIC_CAL_FLASH_ADDR);

    /* Write struct word by word — nrfx_nvmc_word_write blocks per word.     */
    const uint32_t *src  = (const uint32_t *)&cal;
    uint32_t        addr = MIC_CAL_FLASH_ADDR;
    for (size_t i = 0; i < sizeof(mic_cal_t) / sizeof(uint32_t); i++) {
        nrfx_nvmc_word_write(addr, src[i]);
        addr += sizeof(uint32_t);
    }

    /* Verify readback                                                        */
    if (!mic_cal_is_valid()) {
        printk("MIC_CAL: write verification FAILED\n");
        return false;
    }
    printk("MIC_CAL: saved gain=%.4f to 0x%05X\n",
           (double)gain, MIC_CAL_FLASH_ADDR);
    return true;
}
