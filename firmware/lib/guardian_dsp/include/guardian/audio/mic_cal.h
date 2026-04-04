#ifndef GUARDIAN_MIC_CAL_H
#define GUARDIAN_MIC_CAL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Microphone sensitivity calibration ──────────────────────────────────────
 * Problem: PDM mics have ±3dB sensitivity tolerance part-to-part.
 *   Without calibration: keyword detection accuracy varies ±15% unit-to-unit.
 *   With calibration: <3% variation → customer-acceptable consistency.
 *
 * Factory procedure (once per unit):
 *   1. Play 1kHz sine at 94 dB SPL (0 dB re 20µPa) from reference speaker
 *   2. Measure ADC output RMS over 1 second
 *   3. cal_gain = NOMINAL_RMS_AT_94DBSPL / measured_rms
 *   4. Write cal_gain to NVMC via mic_cal_save()
 *
 * Runtime: mic_cal_load() reads gain at boot → applied by audio_frontend.
 *
 * Storage: Second-to-last 4KB flash page (0xFE000).
 *   Far from application image (~0x20000), never overwritten by flashing.
 *   Survives firmware updates unless flash is fully erased.
 * ─────────────────────────────────────────────────────────────────────────── */

#define MIC_CAL_FLASH_ADDR  0xFE000U   /* nRF52840: 1MB flash, page 254 */
/* v2 magic: CRC16-CCITT replaces v1 XOR checksum.
 * Bumping magic ensures old XOR-checksummed flash data is rejected at boot
 * (mic_cal_is_valid() returns false → unity gain fallback, safe default).    */
#define MIC_CAL_MAGIC       0xCA1BFE02U

typedef struct {
    uint32_t magic;     /* MIC_CAL_MAGIC — validates format version            */
    float    gain;      /* Calibration gain: 1.0 nominal, range [0.5, 2.0]    */
    uint32_t checksum;  /* CRC16-CCITT over first 8 bytes (magic + gain)       */
} mic_cal_t;

/* Load calibration from flash. Returns true if valid data found.
 * On failure (not programmed or corrupted): sets *gain_out = 1.0f (unity).  */
bool mic_cal_load(float *gain_out);

/* Save calibration to flash. Erases page first, then writes.
 * Call from a factory test CLI command — not from the audio loop.           */
bool mic_cal_save(float gain);

/* Returns true if calibration data is present and checksum is valid.        */
bool mic_cal_is_valid(void);

#endif /* GUARDIAN_MIC_CAL_H */
