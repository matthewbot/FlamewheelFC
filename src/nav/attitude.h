#ifndef FC_NAV_ATTITUDE_H
#define FC_NAV_ATTITUDE_H

#include "math/matrix.h"
#include "math/orientation.h"

void attitude_init();

void attitude_start_triad();
void attitude_start();
void attitude_stop();
void attitude_reset();
bool attitude_running();

struct AttitudeState {
    Quaternion quat;
    VectorF<3> rate;
};

AttitudeState attitude_get_state();

struct AttitudeDebugState {
    Quaternion quat;
    VectorF<3> rate;
    VectorF<3> bias_gyro;
    VectorF<3> bias_accel;
    bool acc_en;
};

AttitudeDebugState attitude_get_debug_state();


#endif

