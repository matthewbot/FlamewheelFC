#ifndef FC_DRIVERS_MPU_H
#define FC_DRIVERS_MPU_H

#include <stdint.h>

void mpu_init();
uint8_t mpu_read_reg(uint8_t reg);
void mpu_write_reg(uint8_t reg, uint8_t val);

void mpu_ss(bool ss);
uint8_t mpu_spi(uint8_t out);

#endif
