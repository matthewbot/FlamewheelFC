#ifndef FC_NAV_INS_H
#define FC_NAV_INS_H

#include "math/matrix.h"
#include "math/orientation.h"

void ins_init();

void ins_start();
void ins_start_triad();
bool ins_running();
void ins_stop();

Quaternion ins_get_quaternion();
VectorF<3> ins_get_rate();
VectorF<3> ins_get_rate_bias();
VectorF<3> ins_get_accel();
VectorF<3> ins_get_accel_bias();

void ins_correct(const Quaternion &quat_err, const VectorF<3> &rate_bias_err, const VectorF<3> &accel_bias_err);
void ins_reset();
void ins_reset(const Quaternion &new_quat, const VectorF<3> &new_rate_bias, const VectorF<3> &new_accel_bias);

#endif
