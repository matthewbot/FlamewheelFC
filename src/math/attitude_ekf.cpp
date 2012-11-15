#include "attitude_ekf.h"

static const float g_e[] = { 0, 0, -9.8 };
static const float m_e[] = { .512308, -0.049210, .857396 };
static const float R_g[] = { 1e-4, 1e-4, 1e-5 };
static const float R_a[] = { 1e-3, 1e-3, 1e-3 };
static const float R_m[] = { 1e-2, 1e-2, 1e-2 };
static const float Q_b_a[] = { 1e-10, 1e-10, 1e-10 };
static const float Q_b_g[] = { 1e-8, 1e-8, 1e-8 };

Quaternion EKFState::err_quat() const {
    Quaternion q;
    q[0] = sqrtf(1 - x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    q.slice<3,1>(1,0) = x.slice<3, 1>(0, 0);
    return q;
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
    Q.slice<3, 3>(6, 6) = diag(ConstMatrix<float, 3, 1>(Q_b_a));

    MatrixF<9, 9> A_d = IdentityMatrix<float, 9>() + dt*A;
    MatrixF<9, 9> Q_d = dt*Q + (0.5f*dt*dt)*(A*Q + Q*tr(A));

    state.x = VectorF<9>(A_d*state.x);
    state.P = MatrixF<9, 9>(A_d*state.P*tr(A_d) + Q_d);

    for (int r=0; r<9; r++)
        for (int c=0; c<r; c++)
            state.P(c, r) = state.P(r, c);

    if (!acc_en && !mag_en) {
        return true;
    }

    MatrixF<3, 3> C = C_mat(q_ins);
    VectorF<3> g_b = C*ConstMatrix<float, 3, 1>(g_e);
    VectorF<3> m_b = C*ConstMatrix<float, 3, 1>(m_e);

    MatrixF<6, 9> H = ZeroMatrix<float, 6, 9>();
    if (acc_en) {
        H.slice<3, 3>(0, 0) = 2*cross_mat(g_b);
        H.slice<3, 3>(0, 6) = IdentityMatrix<float, 3>();
    }
    if (mag_en) {
        H.slice<3, 3>(3, 0) = 2*cross_mat(m_b);
    }

    MatrixF<6, 6> R = ZeroMatrix<float, 6, 6>();
    R.slice<3, 3>(0, 0) = diag(ConstMatrix<float, 3, 1>(R_a));
    R.slice<3, 3>(3, 3) = diag(ConstMatrix<float, 3, 1>(R_m));

    VectorF<6> z;
    z.slice<3, 1>(0, 0) = y_a - g_b;
    z.slice<3, 1>(3, 0) = (1/norm(y_m))*y_m - m_b;

    MatrixF<6, 9> Kt;
    bool ok = cholsolve(H*state.P*tr(H)+R, H*tr(state.P), Kt);
    if (!ok)
        return false;

    state.x = VectorF<9>(state.x + tr(Kt)*(z - H*state.x));
    state.P = MatrixF<9, 9>((IdentityMatrix<float, 9>() - tr(Kt)*H)*state.P);
    for (int r=0; r<9; r++)
        for (int c=0; c<r; c++)
            state.P(c, r) = state.P(r, c);

    return true;
}
