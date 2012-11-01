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
    return { atan2f(rot(1, 0), rot(0, 0)),
            atan2f(-rot(2, 0), sqrtf(rot(2, 1)*rot(2, 1) + rot(2, 2)*rot(2, 2))),
            atan2f(rot(2, 1), rot(2, 2)) };
}
