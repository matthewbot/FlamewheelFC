#include "nav/calibration.h"
#include "math/orientation.h"

static const float accel_shifts[] = {.28, .28, 0};

VectorF<3> calibration_accel(const int16_t (&sample)[3]) {
    VectorF<3> v = { static_cast<float>(-sample[0]), static_cast<float>(-sample[1]), static_cast<float>(sample[2]) };
    return static_cast<float>(.98/8192*9.8)*v + ConstMatrix<float, 3, 1>(accel_shifts);
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
    out[0] = spektrum_to_float(sample.channel[1], 170, 830, .05);
    out[1] = spektrum_to_float(sample.channel[2], 145, 860, .05);
    out[2] = spektrum_to_float(sample.channel[3], 160, 830, .05);
    out[3] = spektrum_to_unsigned_float(sample.channel[0], 200, 845);
    return out;
}

AltCalibrated calibration_alt(const AltSample &sample) {
    const AltEEPROM &eeprom = alt_get_eeprom();

    int X1 = ((sample.ut - (int)eeprom.AC6) * (int)eeprom.AC5) >> 15;
    int X2 = (eeprom.MC << 11) / (X1 + eeprom.MD);
    int B5 = X1 + X2;
    int T = (B5 + 8) >> 4;

    int B6 = B5 - 4000;
    X1 = (eeprom.B2 * (B6 * (B6 >> 12))) >> 11;
    X2 = (eeprom.AC2 * B6) >> 11;
    int X3 = X1 + X2;
    int B3 = ((eeprom.AC1*4+X3)+2)/4;
    X1 = (eeprom.AC3 * B6) >> 13;
    X2 = (eeprom.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    unsigned int B4 = (eeprom.AC4 * (unsigned int)(X3 + 32768)) >> 15;
    unsigned int B7 = ((unsigned int)sample.up - B3) * 50000;
    int p;
    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    AltCalibrated cal;
    cal.temp = T / 10.0f;
    cal.pressure = p;
    cal.alt = 44330 * (1 - powf(p / 101325.0f, 1 / 5.255f));
    return cal;
}

float calibration_sonar(uint16_t sample) {
    return sample / 5800.0f;
}
