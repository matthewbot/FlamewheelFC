#include "attitude_ekf.h"

static const float g_e[] = { 0, 0, 9.8 };
static const float m_e[] = { .512299, -0.049209, .857396 };
static const float R_g[] = { 3e-5, 3e-5, 3e-6 };
static const float R_a[] = { 1e-4, 1e-4, 1e-4 };
static const float R_m[] = { 1e-2, 1e-2, 1e-2 };
static const float Q_b_a[] = { 1e-8, 1e-8, 1e-8 };
static const float Q_b_g[] = { 1e-8, 1e-8, 1e-8 };

EKFState attitude_ekf(const EKFState &state,
                  const VectorF<3> &y_g, const VectorF<3> &y_a, const VectorF<3> &y_m,
                  const Quaternion &q_ins, bool mag_en, bool acc_en, float dt) {
    MatrixF<9, 9> A = ZeroMatrix<float, 9, 9>();
    A.slice<3, 3>(0, 0) = -cross_mat(y_g);
    A.slice<3, 3>(0, 3) = -0.5f*IdentityMatrix<float, 3>();

    MatrixF<9, 9> Q = ZeroMatrix<float, 9, 9>();
    Q.slice<3, 3>(0, 0) = 0.25f*diag(ConstMatrix<float, 3, 1>(R_g));
    Q.slice<3, 3>(3, 3) = diag(ConstMatrix<float, 3, 1>(Q_b_g));
    Q.slice<3, 3>(6, 6) = diag(ConstMatrix<float, 3, 1>(Q_b_a));

    MatrixF<9, 9> A_d = IdentityMatrix<float, 9>() + dt*A;
    auto Q_d = dt*Q + (0.5f*dt*dt)*(A*Q + Q*tr(A));

    EKFState newstate;
    newstate.x = A_d*state.x;
    newstate.P = A_d*state.P*tr(A_d) + Q_d;

    if (!acc_en && !mag_en) {
        return newstate;
    }

    MatrixF<3, 3> C = C_mat(q_ins);
    auto g_b = C*ConstMatrix<float, 3, 1>(g_e);
    auto m_b = C*ConstMatrix<float, 3, 1>(m_e);

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

    MatrixF<6, 9> Kt = cholsolve(H*newstate.P*tr(H)+R, H*tr(newstate.P));
    auto K = tr(Kt);

    newstate.x = VectorF<9>(newstate.x + K*(z - H*newstate.x));
    newstate.P = MatrixF<9, 9>((IdentityMatrix<float, 9>() - K*H)*newstate.P);
    return newstate;
}
