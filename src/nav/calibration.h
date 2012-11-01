#ifndef FC_NAV_CALIBRATION_H
#define FC_NAV_CALIBRATION_H

#include "math/matrix.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include <stdint.h>

VectorF<3> calibration_accel(const int16_t (&sample)[3]);
VectorF<3> calibration_gyro(const int16_t (&sample)[3]);
VectorF<3> calibration_mag(const int16_t (&sample)[3]);

#endif
