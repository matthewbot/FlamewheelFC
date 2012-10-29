Tend = 120;
dt = 1e-3;
N = Tend / dt;
T = linspace(0, Tend, N);

qinit = [1; .1; 0; 0];
qinit /= norm(qinit);
b_a_init = [.1; -.1; .05];
b_g_init = [.1; .02; -.1];

R_g = 1e-5*eye(3);
R_a = 1e-1*eye(3);
R_m = 1e-4*eye(3);
Q_b_g = 1e-4*eye(3);
Q_b_a = 1e-4*eye(3);

g_e = [0; 0; 9.8];
m_e = [1; 0; .5];

w_log = zeros(3, N);
for i=1:N
  t = i*dt;
  w_log(:, i) = [.05*cos(2*pi*t/20); .03*cos(2*pi*t/31)*sin(2*pi*t/5)+.05; .2*sin(2*pi*t/10)^2];
  if t > 30 && t < 40
    w_log(:, i) = zeros(3, 1);
  endif
endfor
a_ext_log = zeros(3, N);

q_log = qint(w_log, qinit, dt);
[y_g_log, b_g_log] = gyro_sim(w_log, b_g_init, R_g, Q_b_g, dt);
[y_a_log, b_a_log] = accel_sim(q_log, a_ext_log, g_e, b_a_init, R_a, Q_b_a, dt);
[y_m_log] = mag_sim(q_log, m_e, R_m, dt);

close all;

plotlog(w_log, T);
title('Angular rate');

plotlog(q_log, T);
title('Quaternion orientation');

plotlog(y_g_log, T);
title('Gyroscope');

plotlog(y_a_log, T);
title('Accelerometer');

plotlog(y_m_log, T);
title('Magnetometer');

save sim.dat Tend dt qinit b_a_init b_g_init R_g R_a R_m Q_b_g Q_b_a g_e m_e w_log a_ext_log q_log y_g_log b_g_log y_a_log b_a_log y_m_log;
