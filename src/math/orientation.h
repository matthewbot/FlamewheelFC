#ifndef FC_MATH_ORIENTATION_H
#define FC_MATH_ORIENTATION_H

#include "matrix.h"

static constexpr float pi = 3.1415926535;

MatrixF<3, 3> triad_algorithm(const VectorF<3> &accel, const VectorF<3> &mag);

inline constexpr float deg_to_rad(float deg) { return deg * (pi / static_cast<float>(180)); }
inline constexpr float rad_to_deg(float rad) { return rad * (static_cast<float>(180) / pi); }
VectorF<3> rotation_to_rpy(const MatrixF<3, 3> &rot);

#endif
