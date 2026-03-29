#ifndef GUARDIAN_DMA_PDM_H
#define GUARDIAN_DMA_PDM_H

#include <stdint.h>

#define DMA_PDM_BUFFER_SIZE 320

int      dma_pdm_init(void);
int      dma_pdm_start(void);
int      dma_pdm_read(int16_t **buffer);       /* DMA mode: CPU sleeps between frames */
int      dma_pdm_read_poll(int16_t **buffer);  /* Polling mode: CPU busy-waits */
uint32_t dma_pdm_get_irq_timestamp_ms(void);   /* IRQ fire time for latency measurement */

#endif
