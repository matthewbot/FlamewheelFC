#ifndef FC_MATH_ORIENTATION_H
#define FC_MATH_ORIENTATION_H

#include "matrix.h"

using Quaternion = VectorF<4>;

static constexpr float pi = 3.1415926535;
inline constexpr float deg_to_rad(float deg) { return deg * (pi / static_cast<float>(180)); }
inline constexpr float rad_to_deg(float rad) { return rad * (static_cast<float>(180) / pi); }

MatrixF<3, 3> triad_algorithm(const VectorF<3> &accel, const VectorF<3> &mag);
VectorF<3> rot_to_rpy(const MatrixF<3, 3> &rot);
Quaternion rot_to_quat(const MatrixF<3, 3> &rot);

Quaternion quat_mult(const Quaternion &a, const Quaternion &b);
void quat_norm(Quaternion &a);
Quaternion quat_conj(const Quaternion &q);
Quaternion quat_int(const Quaternion &q, const VectorF<3> &w, float dt);
Quaternion quat_axisangle(const VectorF<3> &axis, float angle);
VectorF<3> quat_to_rpy(const Quaternion &q);
VectorF<3> quat_to_axisangle(const Quaternion &q, float &angle);
MatrixF<3, 3> C_mat(const Quaternion &q);

VectorF<4> motor_map(const VectorF<4> &out); // [Tx Ty Tz F]

#endif
