function [x_up, P_up, z] = ekf_ind(x, P, y_g, y_a, y_m, q_ins, g_e, m_e, R_g, R_a, R_m, Q_b_g, Q_b_a, acc_en, mag_en, dt)
  A = zeros(9);
  A(1:3, 1:3) = -cross_mat(y_g);
  A(1:3, 4:6) = -0.5*eye(3);

  Q = zeros(9);
  Q(1:3,1:3) = 0.25*R_g;
  Q(4:6,4:6) = Q_b_g;
  Q(7:9,7:9) = Q_b_a;

  A_d = eye(9) + A*dt;
  Q_d = Q*dt + .5*A*Q*dt^2 + .5*Q*A'*dt^2;

  x_pred = A_d*x;
  P_pred = A_d*P*A_d' + Q_d;

  if acc_en == 0 && mag_en == 0
    x_up = x_pred;
    P_up = P_pred;
    return;
  endif

  g_b = C_mat(q_ins)*g_e;
  m_b = C_mat(q_ins)*m_e;

  H = [2*cross_mat(g_b), zeros(3), eye(3);
       2*cross_mat(m_b), zeros(3), zeros(3)];

  if acc_en == 0
    H(1:3, :) = 0;
  elseif mag_en == 0
    H(4:6, :) = 0;
  endif

  R = [R_a, zeros(3),
       zeros(3), R_m];
  z = [y_a - g_b;
       y_m/norm(y_m) - m_b];

  ch = chol(H*P_pred*H' + R)';
  K = (ch'\(ch\(H*P_pred')))';
  x_up = x_pred + K*(z - H*x_pred);
  P_up = (eye(9)-K*H)*P_pred;

#  x_up = x_pred;
#  P_up = P_pred;

 # P_up = triu(P_up, 1) + triu(P_up, 1)' + diag(diag(P_up));
  if (sum(diag(P_up) <= 0) > 0)
    save bad.dat P P_pred P_up x x_pred x_up H P R A_d Q_d A Q y_g y_a y_m
    blarg
  endif
endfunction