#ifndef FC_DRIVERS_MPU_H
#define FC_DRIVERS_MPU_H

#include <stdint.h>

void mpu_init();

enum class AccelFS : uint8_t { FS2G, FS4G, FS8G, FS16G };
enum class GyroFS : uint8_t { FS200DS, FS500DS, FS1000DS, FS2000DS };

void mpu_reset(AccelFS accel_fs, GyroFS gyro_fs, uint8_t dlpf, uint8_t samplerate_div);

struct MPUSample {
	uint32_t num;
	int16_t accel[3];
	int16_t temp;
	int16_t gyro[3];
};

MPUSample mpu_sample();

#endif
