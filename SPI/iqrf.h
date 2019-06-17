/*
 * Name: iqrf.h
 * Author: Martin Stankus
 *
 */

#ifndef _IQRF_H_
#define _IQRF_H_

#include <stdint.h>

#include "iqrf.h"

#define IQRF_SENS_CNT		64u

typedef struct __attribute__ ((packed)) {
	uint16_t chan_a;
	uint16_t chan_b;
} IQRF_FRC_DATA;

typedef struct __attribute__ ((packed))  {
	uint16_t naddr;
	uint8_t pnum;
	uint8_t pcmd;
	uint16_t hwpid;
	uint8_t errn;
	uint8_t dpa_val;
	uint16_t chan_a;
	uint16_t chan_b;
} IQRF_POLL_DATA;

void iqrf_init(uint8_t expt_pri_spi, uint8_t dma_chan_rx, uint8_t dma_chan_tx);

uint8_t iqrf_en(void);
void iqrf_dis(void);

uint8_t iqrf_rd_frc_uf(uint64_t *urgent_flags); // , uint64_t *battery_flags

uint8_t iqrf_rd_frc_data(IQRF_FRC_DATA data[], uint64_t *valid_flags);
uint8_t iqrf_rd_poll_data(IQRF_POLL_DATA data[], uint64_t *valid_flags);

void iqrf_TEST();

#endif /* _IQRF_H_ */
