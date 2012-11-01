#ifndef FC_MATH_ORIENTATION_H
#define FC_MATH_ORIENTATION_H

#include "matrix.h"

static constexpr float pi = 3.1415926535;

MatrixF<3, 3> triad_algorithm(const VectorF<3> &accel, const VectorF<3> &mag);

inline constexpr float deg_to_rad(float deg) { return deg * (pi / static_cast<float>(180)); }
inline constexpr float rad_to_deg(float rad) { return rad * (static_cast<float>(180) / pi); }
VectorF<3> rotation_to_rpy(const MatrixF<3, 3> &rot);

using Quaternion = VectorF<4>;
Quaternion quat_mult(const Quaternion &a, const Quaternion &b);
Quaternion quat_conj(const Quaternion &q);
Quaternion quat_int(const Quaternion &q, const VectorF<3> &w, float dt);
Quaternion quat_axisangle(const VectorF<3> &axis, float angle);
VectorF<3> quat_to_rpy(const Quaternion &q);
MatrixF<3, 3> C_mat(const Quaternion &q);

#endif
