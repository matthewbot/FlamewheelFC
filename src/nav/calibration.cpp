#include "nav/calibration.h"
#include "math/orientation.h"

VectorF<3> calibration_accel(const int16_t (&sample)[3]) {
    VectorF<3> v = { static_cast<float>(-sample[0]), static_cast<float>(-sample[1]), static_cast<float>(sample[2]) };
    return static_cast<float>(.98/8192*9.8)*v;
}

VectorF<3> calibration_gyro(const int16_t (&sample)[3]) {
    VectorF<3> v = { static_cast<float>(-sample[0]), static_cast<float>(-sample[1]), static_cast<float>(sample[2]) };
    return static_cast<float>(.97/65.5/180*pi)*v;
}

static const int16_t magoffsets[] = {23, -83, 149};
static const float magscales[] = {-121.95, 121.95, -121.95};

VectorF<3> calibration_mag(const int16_t (&sample)[3]) {
    VectorF<3> ret;
    for (int i=0; i<3; i++)
        ret[i] = magscales[i]*static_cast<float>(sample[i] - magoffsets[i]);
    return ret;
}
