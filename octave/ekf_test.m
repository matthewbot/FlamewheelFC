load sim.dat
N = Tend / dt;
T = linspace(0, Tend, N);

x = zeros(9, 1);
P = diag([1 1 1 0.1 0.1 0.1 10 10 10]);
q_ins = [1; 0; 0; 0];
q_ins /= norm(q_ins);
b_g_ins = [0; 0; 0];

x_log = zeros(9, N);
P_log = zeros(9, 9, N);
q_ins_log = zeros(4, N);
b_g_hat_log = zeros(3, N);
q_hat_log = zeros(4, N);

z_log = zeros(6, N);

reset_timer = 0;
for i=1:N
  q_ins = ins(q_ins, y_g_log(:, i), b_g_ins, dt);
  [x, P, z] = ekf_ind(x, P, y_g_log(:, i), y_a_log(:, i), y_m_log(:, i), q_ins, g_e, m_e, R_g, R_a, R_m, Q_b_g*1e1, Q_b_a*1e1, dt);
  q_hat = qmult(q_ins, [1; x(1:3)]);
  q_hat /= norm(q_hat);

  reset_timer += dt;
  if reset_timer >= 1
    q_ins = q_hat;
    b_g_ins += x(4:6);
    x(1:6) = 0;
    reset_timer = 0;
  endif

  x_log(:, i) = x;
  P_log(:, :, i) = P;
  q_ins_log(:, i) = q_ins;
  b_g_hat_log(:, i) = b_g_ins + x(4:6);
  q_hat_log(:, i) = q_hat;
  z_log(:, i) = z;
endfor

close all;

plotlog(q_hat_log, T);
title('Quaternion');

plotlog(q_hat_log - q_log, T);
title('Quaternion error');

plotlog(b_g_hat_log, T);
title('Gyro bias estimation');

plotlog(b_g_hat_log - b_g_log, T);
title('Gyro bias error');

plotlog(x_log(7:9, :), T);
title('Accel bias estimation');

plotlog(x_log(7:9, :) - b_a_log, T);
title('Accel bias error');

