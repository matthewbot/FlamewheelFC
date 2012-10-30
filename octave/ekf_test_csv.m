data = csvread('data.csv')';
data = data(1:6, 2:end);
dt = 1e-3;
N = columns(data);
Tend = N*dt;
T = linspace(0, Tend, N);

y_a_log = data(1:3, :) / 8192;
y_a_log *= .98;
y_g_log = data(4:6, :) / 65.5 / 180 * pi;
y_g_log *= .97;
y_m_log = zeros(3, N);

g_e = [0; 0; 1];
m_e = [1; 0; 0];

R_a = diag([1e-4 1e-4 1e-4]);
R_g = diag([3e-5 3e-5 3e-6]);
R_m = diag([1e-2 1e-2 1e-2]);
Q_b_a = diag([1e-8 1e-8 1e-8]);
Q_b_g = diag([1e-8 1e-8 1e-8]);

x = zeros(9, 1);
P = diag([0.1 0.1 0.1 0.0001 0.0001 0.0001 0.001 0.001 0.001]);
q_ins = [0; 1; 0; 0];
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
  [x, P, z] = ekf_ind(x, P, y_g_log(:, i), y_a_log(:, i), y_m_log(:, i), q_ins, g_e, m_e, R_g, R_a, R_m, Q_b_g*1e1, Q_b_a*1e1, 1, 0, dt);
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

plotlog(b_g_hat_log, T);
title('Gyro bias estimation');

plotlog(x_log(7:9, :), T);
title('Accel bias estimation');
