#ifndef FC_NAV_ATTITUDE_H
#define FC_NAV_ATTITUDE_H

#include "math/matrix.h"
#include "math/orientation.h"

void attitude_init();

void attitude_start_from_triad();
void attitude_start();
void attitude_stop();

struct AttitudeState {
    Quaternion quat;
    VectorF<3> rate;
};

AttitudeState attitude_get_state();

struct AttitudeDebugState {
    Quaternion quat;
    VectorF<3> rate;
    VectorF<3> b_g;
    VectorF<3> b_a;
    bool acc_en;
};

AttitudeDebugState attitude_get_debug_state();


#endif

