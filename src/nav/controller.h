#ifndef FC_NAV_CONTROLLER_H
#define FC_NAV_CONTROLLER_H

#include "math/orientation.h"

void controller_init();
void controller_reset();

void controller_start();
void controller_stop();

enum class ControllerMode { RATE, ATTITUDE, ALTITUDE };

struct ControllerSetpoint {
    ControllerMode mode;
    VectorF<3> rate_d;
    Quaternion att_d;
    float thrust_d;
    float altitude_d;
};

void controller_set(const ControllerSetpoint &setpoint);

struct ControllerDebug {
    VectorF<4> pout;
    VectorF<4> dout;
    VectorF<4> out;
    VectorF<4> motors;
};

ControllerDebug controller_get_debug();

struct ControllerGains {
    VectorF<3> p;
    VectorF<3> d;
};

void controller_set_gains(const ControllerGains &gains);

#endif
