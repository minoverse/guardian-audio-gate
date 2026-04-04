#ifndef GUARDIAN_DMA_PDM_H
#define GUARDIAN_DMA_PDM_H

#include <stdint.h>
#include <stdbool.h>

#define DMA_PDM_BUFFER_SIZE 320

/* ── PPI batch frame count ────────────────────────────────────────────────────
 * The ring path (dma_pdm_read_ring) wakes the CPU once every BATCH_N_FRAMES
 * PDM frames instead of every frame.  This reduces CPU wakeup frequency
 * from 50 Hz to 50/BATCH_N_FRAMES Hz, cutting HFXO startup overhead by
 * (BATCH_N_FRAMES-1)/BATCH_N_FRAMES × N_transitions × 4µJ per transition.
 *
 * Hardware chain (zero CPU cycles per PDM frame within a batch):
 *   PDM EVENTS_END ──[PPI CH0]──► TIMER1 TASKS_COUNT
 *   TIMER1 CC[0]=BATCH_N ──[PPI CH1]──► EGU0 TASKS_TRIGGER[0]
 *   EGU0_IRQn: k_sem_give(batch_sem) × BATCH_N_FRAMES
 *
 * Default 4 = 80ms window: TinyML latency budget unchanged.
 * Must be ≥1.  Increasing beyond 8 risks ring overruns if gate processing
 * falls behind (ring holds PREROLL_FRAMES = 50 frames = 1000ms capacity).  */
#define DMA_PDM_BATCH_N_FRAMES  4

int      dma_pdm_init(void);
int      dma_pdm_start(void);
int      dma_pdm_read(int16_t **buffer);       /* DMA mode: CPU sleeps between frames */
int      dma_pdm_read_poll(int16_t **buffer);  /* Polling mode: CPU busy-waits */
uint32_t dma_pdm_get_irq_timestamp_ms(void);   /* IRQ fire time for latency measurement */

/* ── Zero-copy ring buffer API ───────────────────────────────────────────────
 * True zero-copy: DMA writes directly into the caller's ring buffer slots.
 * No memcpy at all — the PDM hardware IS the ring writer.
 *
 * Usage:
 *   dma_pdm_set_ring(preroll_ring[0], PREROLL_FRAMES, FRAME_SIZE);
 *   dma_pdm_start();
 *   ...
 *   int16_t *frame; uint32_t head;
 *   dma_pdm_read_ring(&frame, &head);   // frame points into ring — no copy
 * ─────────────────────────────────────────────────────────────────────────── */
void     dma_pdm_set_ring(int16_t *ring_base, uint32_t n_frames, uint32_t frame_size);
int      dma_pdm_read_ring(int16_t **frame_ptr, uint32_t *head_out);

/* ── Fault tolerance API ─────────────────────────────────────────────────────
 * Call dma_pdm_needs_restart() each frame — returns true when the PDM ISR
 * detected a hardware overflow and flagged recovery.  Then call
 * dma_pdm_restart() from the main thread to stop/start the peripheral.
 * dma_pdm_get_error_count() / dma_pdm_get_overrun_count() are lifetime
 * counters for health logging.                                               */
bool     dma_pdm_needs_restart(void);
int      dma_pdm_restart(void);
uint32_t dma_pdm_get_error_count(void);
uint32_t dma_pdm_get_overrun_count(void);
#ifdef FAULT_INJECT_ENABLED
void     dma_pdm_inject_error(void);   /* test only: simulate ISR error event */
void     dma_pdm_stop_for_test(void);  /* test only: stop PDM + drain semaphore */
#endif

#endif
