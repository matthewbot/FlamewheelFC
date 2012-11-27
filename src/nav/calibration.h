#ifndef FC_NAV_CALIBRATION_H
#define FC_NAV_CALIBRATION_H

#include "math/matrix.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "drivers/spektrum.h"
#include "drivers/alt.h"
#include <stdint.h>

struct AltCalibrated {
    float temp;
    float pressure; // in Pa
    float alt;
};

VectorF<3> calibration_accel(const int16_t (&sample)[3]);
VectorF<3> calibration_gyro(const int16_t (&sample)[3]);
VectorF<3> calibration_mag(const int16_t (&sample)[3]);
void calibration_esc(const VectorF<4> &thrust, uint16_t (&pwms)[4], uint16_t minpwm);
VectorF<4> calibration_spektrum(const SpektrumSample &sample);
AltCalibrated calibration_alt(const AltSample &sample);
float calibration_sonar(uint16_t sample);

#endif
