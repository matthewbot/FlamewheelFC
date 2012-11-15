#include "orientation.h"
#include <math.h>

MatrixF<3, 3> triad_algorithm(const VectorF<3> &accel, const VectorF<3> &mag) {
    MatrixF<3, 3> fixed = { 0, -0.09562, -0.99542,
                            0, -0.99542, 0.09562,
                            -1, 0, 0 };

    MatrixF<3, 3> measured;
    measured.slice<3, 1>(0, 0) = (1/norm(accel))*accel;

    VectorF<3> cr = cross(accel, mag);
    measured.slice<3, 1>(0, 1) = (1/norm(cr))*cr;
    cr = cross(measured.slice<3, 1>(0, 0), measured.slice<3, 1>(0, 1));
    measured.slice<3, 1>(0, 2) = cr;

    MatrixF<3, 3> rot = fixed*tr(measured);
    return rot;
}

VectorF<3> rot_to_rpy(const MatrixF<3, 3> &rot) {
    VectorF<3> ret;
    ret[0] = atan2f(rot(2,1), rot(2,2));
    ret[1] = -asinf(rot(2,0));
    ret[2] = atan2f(rot(1,0), rot(0, 0));
    return ret;
}

Quaternion rot_to_quat(const MatrixF<3, 3> &rot) {
    Quaternion q;
    q[0] = sqrtf(1 + rot(0, 0) + rot(1, 1) + rot(2, 2))/2;
    q[1] = copysignf(sqrtf(1 + rot(0, 0) - rot(1, 1) - rot(2, 2))/2, rot(2, 1) - rot(1, 2));
    q[2] = copysignf(sqrtf(1 - rot(0, 0) + rot(1, 1) - rot(2, 2))/2, rot(0, 2) - rot(2, 0));
    q[3] = copysignf(sqrtf(1 - rot(0, 0) - rot(1, 1) + rot(2, 2))/2, rot(1, 0) - rot(0, 1));
    return q;
}

Quaternion quat_mult(const Quaternion &a, const Quaternion &b) {
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
}

void quat_norm(Quaternion &a) {
    a = (1/norm(a))*a;
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
    if (fabsf(angle) < 1e-8f)
        return { 1, 0, 0, 0 };

    Quaternion ret;
    ret[0] = cosf(angle/2);
    ret.slice<3, 1>(1, 0) = (sinf(angle/2)/norm(axis))*axis;
    return ret;
}

VectorF<3> quat_to_axisangle(const Quaternion &quat, float &angle) {
    VectorF<3> axis = quat.slice<3, 1>(1, 0);
    angle = 2*acosf(quat[0]);
    float s = sqrtf(1-quat[0]*quat[0]);
    if (s > (float)1e-5)
        axis = (1/s)*axis;
    return axis;
}

VectorF<3> quat_to_rpy(const Quaternion &q) {
    return {
        atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1 - 2*(q[1]*q[1]+q[2]*q[2])),
        asinf(2*q[0]*q[2]-q[3]*q[1]),
        atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1 - 2*(q[2]*q[2]+q[3]*q[3])) };
}

MatrixF<3, 3> C_mat(const Quaternion &q) {
    return {
        2*q[0]*q[0] + 2*q[1]*q[1] - 1, 2*q[1]*q[2] + 2*q[0]*q[3],     2*q[1]*q[3] - 2*q[0]*q[2],
        2*q[1]*q[2] - 2*q[0]*q[3],     2*q[0]*q[0] + 2*q[2]*q[2] - 1, 2*q[2]*q[3] + 2*q[0]*q[1],
        2*q[1]*q[3] + 2*q[0]*q[2],     2*q[2]*q[3] - 2*q[0]*q[1],     2*q[0]*q[0] + 2*q[3]*q[3] - 1 };
}
