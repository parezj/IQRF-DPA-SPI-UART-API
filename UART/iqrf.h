/*
 * Name:    iqrf.c
 * Author:  Jakub Parez
 * Date:    23.1.2019
 * Version: 0.2.1
 *
 */

#ifndef _IQRF_H_
#define _IQRF_H_

#include <stdint.h>

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

void iqrf_init(void);

uint8_t iqrf_en(void);
void iqrf_dis(void);

uint8_t iqrf_rd_frc_uf(uint64_t *urgent_flags, uint64_t *battery_flags);
uint8_t iqrf_rd_frc_data(IQRF_FRC_DATA data[], uint64_t *valid_flags);
uint8_t iqrf_rd_poll_data(IQRF_POLL_DATA data[], uint64_t *valid_flags);

void iqrf_test();

#endif /* _IQRF_H_ */
