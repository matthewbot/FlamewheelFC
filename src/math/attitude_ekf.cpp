#include "attitude_ekf.h"

static const float g_e[] = { 0, 0, -9.8 };
static const float m_e[] = { 2.41377e4, -2.32061e3, 4.038953e4 };
static const float R_g[] = { 1e-4, 1e-3, 1e-3 };
static const float R_a[] = { 3e-1, 3e-1, 3e-1 };
static const float R_m[] = { 3e4, 3e4, 3e4 };
static const float Q_b_g[] = { 1e-7, 1e-7, 1e-7 };
static const float Q_b_m[] = { 1e0, 1e0, 1e0 };

Quaternion EKFState::err_quat() const {
    Quaternion q;
    q[0] = sqrtf(1 - x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    q.slice<3,1>(1,0) = x.slice<3, 1>(0, 0);
    return q;
}

template <typename M>
static void make_diag(M &mat) {
    for (int r=0; r<mat.rows(); r++)
        for (int c=0; c<r; c++)
            mat(r, c) = mat(c, r);
}

bool attitude_ekf(EKFState &state,
                  const VectorF<3> &y_g, const VectorF<3> &y_a, const VectorF<3> &y_m,
                  const Quaternion &q_ins, bool mag_en, bool acc_en, float dt) {
    MatrixF<9, 9> A = ZeroMatrix<float, 9, 9>();
    A.slice<3, 3>(0, 0) = cross_mat(y_g);
    A.slice<3, 3>(0, 3) = -0.5f*IdentityMatrix<float, 3>();

    MatrixF<9, 9> Q = ZeroMatrix<float, 9, 9>();
    Q.slice<3, 3>(0, 0) = 0.25f*diag(ConstMatrix<float, 3, 1>(R_g));
    Q.slice<3, 3>(3, 3) = diag(ConstMatrix<float, 3, 1>(Q_b_g));
    Q.slice<3, 3>(6, 6) = diag(ConstMatrix<float, 3, 1>(Q_b_m));

    MatrixF<9, 9> A_d = IdentityMatrix<float, 9>() + dt*A;
    MatrixF<9, 9> Q_d = dt*Q + (0.5f*dt*dt)*(A*Q + Q*tr(A));

    state.x = VectorF<9>(A_d*state.x);
    state.P = MatrixF<9, 9>(A_d*state.P*tr(A_d) + Q_d);
    make_diag(state.P);

    if (!acc_en && !mag_en) {
        return true;
    }

    MatrixF<3, 3> C = C_mat(q_ins);
    VectorF<3> g_b = C*ConstMatrix<float, 3, 1>(g_e);
    VectorF<3> m_b = C*ConstMatrix<float, 3, 1>(m_e);

    MatrixF<6, 9> H = ZeroMatrix<float, 6, 9>();
    if (acc_en) {
        H.slice<3, 3>(0, 0) = 2*cross_mat(g_b);
    }
    if (mag_en) {
        H.slice<3, 3>(3, 0) = 2*cross_mat(m_b);
        H.slice<3, 3>(3, 6) = IdentityMatrix<float, 3>();
    }

    MatrixF<6, 6> R = ZeroMatrix<float, 6, 6>();
    R.slice<3, 3>(0, 0) = diag(ConstMatrix<float, 3, 1>(R_a));
    R.slice<3, 3>(3, 3) = diag(ConstMatrix<float, 3, 1>(R_m));

    VectorF<6> z;
    z.slice<3, 1>(0, 0) = y_a - g_b;
    z.slice<3, 1>(3, 0) = y_m - m_b;

    MatrixF<6, 9> Kt;
    bool ok = cholsolve(H*state.P*tr(H)+R, H*tr(state.P), Kt);
    if (!ok)
        return false;

    state.x = VectorF<9>(state.x + tr(Kt)*(z - H*state.x));
    state.P = MatrixF<9, 9>((IdentityMatrix<float, 9>() - tr(Kt)*H)*state.P);
    make_diag(state.P);

    return true;
}
