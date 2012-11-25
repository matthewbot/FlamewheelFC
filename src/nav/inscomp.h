#ifndef FC_NAV_ATTITUDE_H
#define FC_NAV_ATTITUDE_H

#include "math/matrix.h"
#include "math/orientation.h"

void inscomp_init();

void inscomp_start_triad();
void inscomp_start();
void inscomp_stop();
void inscomp_reset();
bool inscomp_running();

struct INSCompState {
    Quaternion quat;
    VectorF<3> rate;
};

INSCompState inscomp_get_state();

struct INSCompDebugState {
    Quaternion quat;
    VectorF<3> rate;
    VectorF<3> bias_gyro;
    VectorF<3> bias_mag;
    float acc_norm_err;
};

INSCompDebugState inscomp_get_debug_state();


#endif

