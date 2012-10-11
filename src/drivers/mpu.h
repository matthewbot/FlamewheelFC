#ifndef FC_DRIVERS_MPU_H
#define FC_DRIVERS_MPU_H

#include <stdint.h>

struct MPUSample {
	uint32_t samplenum;
	int16_t accel[3];
	int16_t temp;
	int16_t gyro[3];
};

void mpu_init();

enum class AccelFS : uint8_t { FS2G, FS4G, FS8G, FS16G };
enum class GyroFS : uint8_t { FS200DS, FS500DS, FS1000DS, FS2000DS };

void mpu_reset(AccelFS accel_fs, GyroFS gyro_fs, uint8_t dlpf, uint8_t samplerate_div);
int mpu_get_accel_scale();
int mpu_get_gyro_scale();
int mpu_get_sample_rate();

MPUSample mpu_sample(uint32_t samplenum=0);

uint8_t readreg(uint8_t reg);

#endif
