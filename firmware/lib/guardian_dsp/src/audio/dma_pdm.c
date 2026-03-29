#include <zephyr/kernel.h>
#include <nrfx_pdm.h>
#include <hal/nrf_clock.h>
#include <zephyr/irq.h>
#include "guardian/audio/dma_pdm.h"

#define NUM_DMA_BUFFERS 4

static int16_t dma_buffers[NUM_DMA_BUFFERS][DMA_PDM_BUFFER_SIZE] __aligned(4);
static uint8_t write_idx;
static uint8_t read_idx;
static struct k_sem buffer_sem;

static nrfx_pdm_t pdm_instance = NRFX_PDM_INSTANCE(NRF_PDM_BASE);

/* IRQ timestamp — set in ISR, read by main thread to measure interrupt latency */
static volatile uint32_t irq_timestamp_ms;

uint32_t dma_pdm_get_irq_timestamp_ms(void)
{
    return irq_timestamp_ms;
}

void pdm_dma_event_handler(nrfx_pdm_evt_t const *evt)
{
    if (evt->buffer_released) {
        irq_timestamp_ms = k_uptime_get_32(); /* stamp IRQ fire time */
        write_idx = (write_idx + 1) % NUM_DMA_BUFFERS;
        k_sem_give(&buffer_sem);
    }
    
    if (evt->buffer_requested) {
        nrfx_pdm_buffer_set(&pdm_instance, dma_buffers[write_idx], DMA_PDM_BUFFER_SIZE);
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
    
    k_sem_init(&buffer_sem, 0, NUM_DMA_BUFFERS);
    
    IRQ_CONNECT(PDM_IRQn, 3, pdm_isr, NULL, 0);
    irq_enable(PDM_IRQn);
    
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

/* Week 5 Day 5: polling variant — CPU never sleeps, burns cycles waiting for DMA.
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
