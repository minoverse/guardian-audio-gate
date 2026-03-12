#ifndef GUARDIAN_DMA_PDM_H
#define GUARDIAN_DMA_PDM_H

#include <stdint.h>

#define DMA_PDM_BUFFER_SIZE 320

int dma_pdm_init(void);
int dma_pdm_start(void);
int dma_pdm_read(int16_t **buffer);

#endif
