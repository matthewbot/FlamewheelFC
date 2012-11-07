#ifndef FC_MATH_ATTITUDEEKF_H
#define FC_MATH_ATTITUDEEKF_H

#include "matrix.h"
#include "orientation.h"

struct EKFState {
    VectorF<9> x;
    MatrixF<9, 9> P;
};

EKFState attitude_ekf(const EKFState &state,
                  const VectorF<3> &y_g, const VectorF<3> &y_a, const VectorF<3> &y_m,
                  const Quaternion &q_ins, bool mag_en, bool accel_en, float dt);

#endif
