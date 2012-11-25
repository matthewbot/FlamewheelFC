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

static const float mag_shifts[] = {28.1579, 95.8592, 177.2704};
static const float mag_correction[] = {1.9659e-3, -1.6244e-5, -2.1436e-5,
                                       -1.6244e-5, 2.0544e-3, 8.7991e-5,
                                       -2.1436e-5, 8.7991e-5, 2.0941e-3};

VectorF<3> calibration_mag(const int16_t (&sample)[3]) {
    VectorF<3> vec = {(float)-sample[0], (float)sample[1], (float)-sample[2]};
    return ConstMatrix<float, 3, 3>(mag_correction) * (vec + ConstMatrix<float, 3, 1>(mag_shifts));
}

void calibration_esc(const VectorF<4> &thrust, uint16_t (&pwms)[4], uint16_t minpwm) {
    for (int i=0; i<4; i++) {
        float t = thrust[i];
        uint16_t pwm;
        if (t > 1) {
            pwm = 1000;
        } else if (t < 0) {
            pwm = 0;
        } else {
            pwm = static_cast<uint16_t>(t*1000);
        }

        if (pwm < minpwm)
            pwm = minpwm;

        pwms[i] = pwm;
    }
}

static float spektrum_to_float(uint16_t chan, uint16_t min, uint16_t max, float deadzone) {
    float val = -2*(chan - min)/static_cast<float>(max - min) + 1;
    if (val > 1)
        val = 1;
    else if (val < -1)
        val = -1;
    else if (fabs(val) < deadzone)
        val = 0;
    return val;
}

static float spektrum_to_unsigned_float(uint16_t chan, uint16_t min, uint16_t max) {
    float val = (chan - min)/static_cast<float>(max - min);
    if (val > 1)
        val = 1;
    else if (val < 0)
        val = 0;
    return val;
}

VectorF<4> calibration_spektrum(const SpektrumSample &sample) {
    VectorF<4> out;
    out[0] = spektrum_to_float(sample.channel[1], 170, 830, .02);
    out[1] = spektrum_to_float(sample.channel[2], 145, 860, .02);
    out[2] = spektrum_to_float(sample.channel[3], 200, 850, .05);
    out[3] = spektrum_to_unsigned_float(sample.channel[0], 200, 845);
    return out;
}
