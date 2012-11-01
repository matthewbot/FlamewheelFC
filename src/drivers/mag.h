#ifndef FC_DRIVERS_MAG_H
#define FC_DRIVERS_MAG_H

#include <stdint.h>

void mag_init();

uint8_t mag_readreg(uint8_t addr);
void mag_writereg(uint8_t addr, uint8_t val);

#endif
