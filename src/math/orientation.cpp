#include "orientation.h"
#include <math.h>

MatrixF<3, 3> triad_algorithm(const VectorF<3> &accel, const VectorF<3> &mag) {
    MatrixF<3, 3> fixed = { 0, 0, -1,
                            0, 1, 0,
                            1, 0, 0 };

    VectorF<3> accel_norm = (1/norm(accel))*accel;
    VectorF<3> mag_norm = (1/norm(mag))*mag;

    MatrixF<3, 3> measured;
    measured.slice<3, 1>(0, 0) = accel_norm;

    VectorF<3> cr = cross(accel_norm, mag_norm);
    measured.slice<3, 1>(0, 1) = (1/norm(cr))*cr;
    cr = cross(measured.slice<3, 1>(0, 0), measured.slice<3, 1>(0, 1));
    measured.slice<3, 1>(0, 2) = (1/norm(cr))*cr;

    MatrixF<3, 3> rot = fixed*tr(measured);
    return rot;
}

VectorF<3> rotation_to_rpy(const MatrixF<3, 3> &rot) {
    return { atan2f(rot(2, 1), rot(2, 2)),
            atan2f(-rot(2, 0), sqrtf(rot(2, 1)*rot(2, 1) + rot(2, 2)*rot(2, 2))),
            atan2f(rot(1, 0), rot(0, 0)) };
}

Quaternion quat_mult(const Quaternion &a, const Quaternion &b) {
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] - a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
}

Quaternion quat_conj(const Quaternion &q) {
    return { q[0], -q[1], -q[2], -q[3] };
}

Quaternion quat_int(const Quaternion &q, const VectorF<3> &w, float dt) {
    MatrixF<4, 4> M = { 0,    -w[0], -w[1], -w[2],
                        w[0], 0,     w[2],  -w[1],
                        w[1], -w[2], 0,     w[0],
                        w[2], w[1],  -w[0], 0 };
    return q + (dt/2)*(M * q);
}

Quaternion quat_axisangle(const VectorF<3> &axis, float angle) {
    if (fabsf(angle) < 1e-8)
        return { 1, 0, 0, 0 };

    Quaternion ret;
    ret[0] = cosf(angle/2);
    ret.slice<3, 1>(1, 0) = (sinf(angle/2)/norm(axis))*axis;
    return ret;
}

VectorF<3> quat_to_rpy(const Quaternion &q) {
    return {atan2f(2*q[1]*q[0]-2*q[1]*q[3], 1 - 2*q[1]*q[1] - 2*q[3]*q[3]),
            atan2f(2*q[2]*q[0] - 2*q[1]*q[3], 1 - 2*q[2]*q[2] - 2*q[3]*q[3]),
            asinf(2*q[1]*q[2]+2*q[3]*q[0]) };
}

MatrixF<3, 3> C_mat(const Quaternion &q) {
    return {
        2*q[0]*q[0] + 2*q[1]*q[1] - 1, 2*q[1]*q[2] + 2*q[0]*q[3],     2*q[1]*q[3] - 2*q[0]*q[2],
        2*q[1]*q[2] - 2*q[0]*q[3],     2*q[0]*q[0] + 2*q[2]*q[2] - 1, 2*q[2]*q[3] + 2*q[0]*q[1],
        2*q[1]*q[3] + 2*q[0]*q[2],     2*q[2]*q[3] - 2*q[0]*q[1],     2*q[0]*q[0] + 2*q[3]*q[3] - 1 };
}
