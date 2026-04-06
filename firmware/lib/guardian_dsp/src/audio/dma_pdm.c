#include <zephyr/kernel.h>
#include <nrfx_pdm.h>
#include <hal/nrf_clock.h>
#include <zephyr/irq.h>
#include "guardian/audio/dma_pdm.h"

/* ── PPI + TIMER1 + EGU0 includes ────────────────────────────────────────────
 * Direct CMSIS register access — no nrfx_timer or nrfx_ppi dependency.
 * NRF_TIMER1, NRF_EGU0, NRF_PPI come from <mdk/nrf.h> via nrfx_pdm.h →
 * nrfx.h → nrfx_bsp.h → <mdk/nrf.h>.  No additional include needed.
 * TIMER1 must not be claimed by Zephyr's COUNTER or PWM drivers.
 * On nRF52840 with default Zephyr config (no SoftDevice), TIMER1-4 are free.
 * IRQ allocation: PDM=3, EGU0_batch=4 (lower priority — less time-critical). */

/* ── PPI channel assignments ─────────────────────────────────────────────────
 * Channels 0-19 are software-configurable; 20-31 are fixed pre-programmed.
 * No other driver in this project uses PPI, so 0 and 1 are safe.
 * In a multi-driver system, use nrfx_ppi_channel_alloc() instead.          */
#define BATCH_PPI_PDM_CH   0   /* PDM EVENTS_END → TIMER1 TASKS_COUNT        */
#define BATCH_PPI_TMR_CH   1   /* TIMER1 EVENTS_COMPARE[0] → EGU0 TRIGGER[0] */

#define NUM_DMA_BUFFERS  4   /* legacy ping-pong slots */
#define DMA_PDM_SEM_MAX 16   /* semaphore ceiling — covers 320ms burst without
                              * dropping wakeup signals during brief stalls.
                              * Was 4: after a stall, only 4 wakeups queued → consumer
                              * missed pending frames. 16 >> 3-frame stall watchdog. */

/* ── Legacy ping-pong buffers (used when zero-copy ring is NOT configured) ── */
static int16_t dma_buffers[NUM_DMA_BUFFERS][DMA_PDM_BUFFER_SIZE] __aligned(4);
static uint8_t write_idx;
static uint8_t read_idx;
static struct k_sem buffer_sem;

/* ── Batch wakeup semaphore (ring path only) ─────────────────────────────────
 * Given BATCH_N_FRAMES times by EGU0 ISR (triggered by PPI TIMER1 compare).
 * Main thread drains batch_sem immediately — no API change needed.           */
static struct k_sem batch_sem;

static nrfx_pdm_t pdm_instance = NRFX_PDM_INSTANCE(NRF_PDM_BASE);

/* IRQ timestamp — set in ISR, read by main thread to measure interrupt latency */
static volatile uint32_t irq_timestamp_ms;

/* ── Fault tolerance counters ────────────────────────────────────────────────
 * pdm_error_count    : PDM hardware overflow events (NRFX_PDM_ERROR_OVERFLOW).
 * pdm_overrun_count  : Ring buffer laps — DMA outran the consumer, oldest
 *                      frame silently dropped (correct graceful degradation).
 * pdm_restart_requested: set in ISR on error; main thread checks and restarts. */
static volatile uint32_t pdm_error_count      = 0;
static volatile uint32_t pdm_overrun_count     = 0;
static volatile bool     pdm_restart_requested = false;

/* ── Zero-copy ring buffer state ─────────────────────────────────────────────
 * When ring_base != NULL, DMA writes directly into ring slots.
 * ring_write_head: next slot the DMA will fill (advanced in buffer_requested).
 * ring_read_head : last slot the DMA finished filling (set in buffer_released,
 *                  read by dma_pdm_read_ring — no copy ever touches the data).
 * Both are indices mod ring_n_frames.                                         */
static int16_t   *ring_base       = NULL;
static uint32_t   ring_n_frames   = 0;
static uint32_t   ring_frame_size = 0;
static volatile uint32_t ring_write_head    = 0;  /* advanced in ISR (buffer_requested) */
static volatile uint32_t ring_read_head     = 0;  /* last slot DMA completed (ISR) */
static          uint32_t ring_consumer_head = 0;  /* last slot main thread read (non-ISR) */

uint32_t dma_pdm_get_irq_timestamp_ms(void)
{
    return irq_timestamp_ms;
}

/* ── EGU0 ISR: fires every DMA_PDM_BATCH_N_FRAMES via PPI chain ──────────────
 * Triggered by: PDM EVENTS_END → PPI CH0 → TIMER1 COUNT → COMPARE[0] →
 *               PPI CH1 → EGU0 TRIGGER[0] → this ISR.
 *
 * Gives DMA_PDM_BATCH_N_FRAMES counts to batch_sem in one shot.  The main
 * thread drains them immediately (each dma_pdm_read_ring call takes one count),
 * then blocks on the next batch — waking every BATCH_N × 20ms = 80ms instead
 * of every 20ms.  This cuts CPU wakeup transitions from 50/s to 12.5/s.     */
static void batch_egu_isr(const void *arg)
{
    ARG_UNUSED(arg);
    NRF_EGU0->EVENTS_TRIGGERED[0] = 0;   /* clear interrupt source */
    for (int i = 0; i < DMA_PDM_BATCH_N_FRAMES; i++) {
        k_sem_give(&batch_sem);
    }
}

/* Configure zero-copy ring — call before dma_pdm_start(). */
void dma_pdm_set_ring(int16_t *base, uint32_t n_frames, uint32_t frame_size)
{
    ring_base           = base;
    ring_n_frames       = n_frames;
    ring_frame_size     = frame_size;
    ring_write_head     = 0;
    ring_read_head      = 0;
    ring_consumer_head  = 0;
}

void pdm_dma_event_handler(nrfx_pdm_evt_t const *evt)
{
    /* ── Fault: PDM hardware error (overflow) ────────────────────────────────
     * NRFX_PDM_ERROR_OVERFLOW fires when the DMA FIFO overflows — typically
     * caused by the main thread stalling too long between buffer_requested
     * events.  Flag for recovery in main thread; do not call stop/start here
     * (re-entrant into nrfx_pdm_irq_handler is unsafe).                      */
    if (evt->error) {
        pdm_error_count++;
        pdm_restart_requested = true;
        return;   /* don't process buffers on error frame */
    }

    if (evt->buffer_released) {
        irq_timestamp_ms = k_uptime_get_32();

        if (ring_base != NULL) {
            /* Zero-copy ring path: record completed slot.
             * Do NOT call k_sem_give here — the PPI→TIMER1→EGU0 chain fires
             * batch_egu_isr() every DMA_PDM_BATCH_N_FRAMES to wake the main
             * thread.  This ISR stays at ~2µs (pointer update only).         */
            ring_read_head = ring_write_head;
        } else {
            /* Legacy ping-pong path: wake CPU every frame as before.         */
            write_idx = (write_idx + 1) % NUM_DMA_BUFFERS;
            k_sem_give(&buffer_sem);
        }
    }

    if (evt->buffer_requested) {
        if (ring_base != NULL) {
            /* Zero-copy path: point DMA at next ring slot directly.         */
            uint32_t next = (ring_write_head + 1) % ring_n_frames;
            ring_write_head = next;
            nrfx_pdm_buffer_set(&pdm_instance,
                                &ring_base[next * ring_frame_size],
                                ring_frame_size);
        } else {
            /* Legacy path: use internal ping-pong buffers */
            nrfx_pdm_buffer_set(&pdm_instance,
                                dma_buffers[write_idx],
                                DMA_PDM_BUFFER_SIZE);
        }
    }
}

static void pdm_isr(const void *arg)
{
    ARG_UNUSED(arg);
    nrfx_pdm_irq_handler(&pdm_instance);
}

static void hfclk_start(void)
{
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    while (!nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, NULL)) {
        /* poll HFCLKSTAT */
    }
}

int dma_pdm_init(void)
{
    hfclk_start();

    k_sem_init(&buffer_sem, 0, DMA_PDM_SEM_MAX);
    k_sem_init(&batch_sem,  0, DMA_PDM_SEM_MAX);

    IRQ_CONNECT(PDM_IRQn, 3, pdm_isr, NULL, 0);
    irq_enable(PDM_IRQn);

    /* ── PPI batch counter: TIMER1 in counter mode ───────────────────────────
     * TIMER1 counts PDM buffer-end events via PPI (zero CPU cycles per count).
     * After DMA_PDM_BATCH_N_FRAMES counts, COMPARE[0] fires → PPI → EGU0 →
     * batch_egu_isr() wakes the main thread with N sem counts.
     *
     * Register constants (from nRF52840 Product Specification Table 196):
     *   MODE.Counter = 2, BITMODE.16Bit = 0
     *   SHORTS.COMPARE0_CLEAR = bit 0 (auto-reset after each batch)          */
    NRF_TIMER1->TASKS_STOP    = 1;
    NRF_TIMER1->TASKS_CLEAR   = 1;
    NRF_TIMER1->MODE          = 2U;    /* Counter mode (not Timer mode)       */
    NRF_TIMER1->BITMODE       = 0U;    /* 16-bit width                        */
    NRF_TIMER1->CC[0]         = DMA_PDM_BATCH_N_FRAMES;
    NRF_TIMER1->SHORTS        = 1U;    /* COMPARE0_CLEAR: reset to 0 on match */
    NRF_TIMER1->TASKS_START   = 1;

    /* PPI CH0: PDM EVENTS_END → TIMER1 TASKS_COUNT
     * PDM EVENTS_END (buffer released) offset = 0x108 from PDM base         */
    NRF_PPI->CH[BATCH_PPI_PDM_CH].EEP = (uint32_t)&NRF_PDM->EVENTS_END;
    NRF_PPI->CH[BATCH_PPI_PDM_CH].TEP = (uint32_t)&NRF_TIMER1->TASKS_COUNT;

    /* PPI CH1: TIMER1 EVENTS_COMPARE[0] → EGU0 TASKS_TRIGGER[0]             */
    NRF_PPI->CH[BATCH_PPI_TMR_CH].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[BATCH_PPI_TMR_CH].TEP = (uint32_t)&NRF_EGU0->TASKS_TRIGGER[0];

    /* Enable both PPI channels (set bits in CHENSET) */
    NRF_PPI->CHENSET = (1u << BATCH_PPI_PDM_CH) | (1u << BATCH_PPI_TMR_CH);

    /* EGU0 interrupt: priority 4 (lower than PDM at 3 — less time-critical) */
    NRF_EGU0->INTENSET = EGU_INTENSET_TRIGGERED0_Msk;
    IRQ_CONNECT(SWI0_EGU0_IRQn, 4, batch_egu_isr, NULL, 0);
    irq_enable(SWI0_EGU0_IRQn);

    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(26, 25);
    pdm_config.prescalers.clock_freq = NRF_PDM_FREQ_1280K;
    pdm_config.prescalers.ratio = NRF_PDM_RATIO_80X;

    int err = nrfx_pdm_init(&pdm_instance, &pdm_config, pdm_dma_event_handler);
    return err;
}

int dma_pdm_start(void)
{
    return nrfx_pdm_start(&pdm_instance);
}

int dma_pdm_read(int16_t **buffer)
{
    if (k_sem_take(&buffer_sem, K_MSEC(25)) != 0) {
        return -ETIMEDOUT;
    }

    *buffer = dma_buffers[read_idx];
    read_idx = (read_idx + 1) % NUM_DMA_BUFFERS;

    return DMA_PDM_BUFFER_SIZE;
}

/* Zero-copy ring read — returns pointer directly into ring slot, no memcpy.
 * head_out receives the ring slot index that was just filled by DMA.
 * Caller uses head_out to track the circular buffer position for TinyML.     */
int dma_pdm_read_ring(int16_t **frame_ptr, uint32_t *head_out)
{
    if (ring_base == NULL) {
        return -EINVAL;   /* ring not configured — call dma_pdm_set_ring() first */
    }
    /* Timeout = (BATCH_N + 1) × 20ms frame period → allows full batch window
     * plus one extra frame of margin before declaring a stall.
     * Main thread's stall watchdog still fires at 3 consecutive timeouts:
     *   3 × (BATCH_N+1) × 20ms = 3 × 100ms = 300ms < 500ms WDT window.     */
    if (k_sem_take(&batch_sem,
                   K_MSEC((DMA_PDM_BATCH_N_FRAMES + 1) * 20)) != 0) {
        return -ETIMEDOUT;
    }

    /* Sequential consumption: advance consumer by one slot per call.
     * DMA fills ring slots continuously; this function reads them in order.
     * Each of the BATCH_N calls per batch reads the next slot in sequence,
     * so the caller sees frames N, N+1, N+2, N+3 — not N+3 four times.
     *
     * Overrun guard: if DMA has nearly lapped the consumer (lag ≥ N-1),
     * skip forward to avoid reading a slot DMA is currently filling.
     * NOTE: with modular ring indices, a real lap (ring_n_frames frames
     * consumed without a read) is indistinguishable from a normal small lag.
     * The WDT (500 ms) fires before any real lap on a 50-frame (1000 ms) ring,
     * so this guard is a conservative last-resort safety check only.          */
    uint32_t next   = (ring_consumer_head + 1) % ring_n_frames;
    uint32_t latest = ring_read_head;
    uint32_t lag    = (latest + ring_n_frames - next) % ring_n_frames;
    if (lag >= ring_n_frames - 1) {
        /* DMA nearly lapped consumer — skip to a safe position */
        pdm_overrun_count++;
        next = (latest + ring_n_frames - 1) % ring_n_frames;
    }
    ring_consumer_head = next;

    *frame_ptr = &ring_base[next * ring_frame_size];
    *head_out  = next;
    return (int)ring_frame_size;
}

/* ── Fault tolerance API ─────────────────────────────────────────────────────*/

/* Returns true if the ISR flagged a hardware error and restart is required.
 * Main thread must call dma_pdm_restart() then clear the flag.               */
bool dma_pdm_needs_restart(void)
{
    return pdm_restart_requested;
}

uint32_t dma_pdm_get_error_count(void)   { return pdm_error_count; }
uint32_t dma_pdm_get_overrun_count(void) { return pdm_overrun_count; }

/* ── Test-only helpers — excluded from production binary ─────────────────────
 * Compiled only when -DFAULT_INJECT=ON is passed to west build.
 * Production build: FAULT_INJECT_ENABLED undefined → linker never sees these
 * symbols → zero flash cost.                                                  */
#ifdef FAULT_INJECT_ENABLED
void dma_pdm_inject_error(void)
{
    pdm_error_count++;
    pdm_restart_requested = true;
}

void dma_pdm_stop_for_test(void)
{
    nrfx_pdm_stop(&pdm_instance);
    while (k_sem_take(&buffer_sem, K_NO_WAIT) == 0) { /* drain */ }
}
#endif /* FAULT_INJECT_ENABLED */

/* Soft-reset the PDM peripheral from the main thread.
 * Safe to call after a timeout or after dma_pdm_needs_restart() returns true.
 * Resets ring indices so the consumer starts fresh on the next filled slot.   */
int dma_pdm_restart(void)
{
    nrfx_pdm_stop(&pdm_instance);
    /* Reset ring state — main thread will re-sync preroll tracking vars */
    ring_write_head       = 0;
    ring_read_head        = 0;
    ring_consumer_head    = 0;
    pdm_restart_requested = false;
    /* Reset TIMER1 counter so the next batch starts from zero, preventing a
     * partial-batch ghost wakeup from a stale count carried over the restart. */
    NRF_TIMER1->TASKS_CLEAR = 1;
    /* Drain any stale batch_sem counts accumulated before the restart.       */
    while (k_sem_take(&batch_sem, K_NO_WAIT) == 0) { /* drain */ }
    return nrfx_pdm_start(&pdm_instance);
}

/* Polling variant — CPU never sleeps, burns cycles waiting for DMA.
 * Use this to measure power cost of busy-wait vs k_sem sleep (DMA mode above).
 * Expected: noticeably higher active current than dma_pdm_read(). */
int dma_pdm_read_poll(int16_t **buffer)
{
    /* Spin until ISR advances write_idx (buffer released), signalling new data.
     * write_idx == read_idx means no new buffer yet. */
    volatile uint8_t *p_write = (volatile uint8_t *)&write_idx;
    while (*p_write == read_idx) {
        /* busy-wait — keeps CPU in active state the entire inter-frame period */
    }

    /* Drain the semaphore counter so it does not overflow if mode is switched */
    k_sem_take(&buffer_sem, K_NO_WAIT);

    *buffer = dma_buffers[read_idx];
    read_idx = (read_idx + 1) % NUM_DMA_BUFFERS;

    return DMA_PDM_BUFFER_SIZE;
}
