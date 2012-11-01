#ifndef FC_NAV_INS_H
#define FC_NAV_INS_H

#include "math/matrix.h"
#include "math/orientation.h"

void ins_init();

void ins_start();
void ins_stop();

Quaternion ins_get_quaternion();
VectorF<3> ins_get_bias();

void ins_correct(const Quaternion &quat_err, const VectorF<3> &bias_err);
void ins_reset(const Quaternion &new_quat, const VectorF<3> &new_bias);

#endif
