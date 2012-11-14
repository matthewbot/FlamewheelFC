#ifndef FC_NAV_CONTROLLER_H
#define FC_NAV_CONTROLLER_H

#include "math/orientation.h"

void controller_init();

void controller_start();
void controller_stop();

void controller_set_desired(const Quaternion &attitude, const VectorF<3> &rate);

struct ControllerDebug {
    VectorF<3> e1;
    VectorF<3> e2;
    VectorF<3> out;
};

#endif
