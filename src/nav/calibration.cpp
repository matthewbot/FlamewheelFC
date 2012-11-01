#include "calibration.h"

VectorF<3> calibration_accel(const int16_t (&sample)[3]) {
    return { static_cast<float>(sample[0]), static_cast<float>(-sample[1]), static_cast<float>(-sample[2]) };
}

VectorF<3> calibration_gyro(const int16_t (&sample)[3]) {
    return { static_cast<float>(sample[0]), static_cast<float>(sample[1]), static_cast<float>(sample[2]) };
}

static const int16_t magoffsets[] = {90, 149, 213};

VectorF<3> calibration_mag(const int16_t (&sample)[3]) {
    VectorF<3> ret;
    for (int i=0; i<3; i++)
        ret[i] = static_cast<float>(-sample[i] - magoffsets[i]);
    return ret;
}
