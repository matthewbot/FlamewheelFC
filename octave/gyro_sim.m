function [y_g_log, b_g_log] = gyro_sim(w_log, b_g_init, R_g, Q_b_g, dt)
  N = columns(w_log);
  b_g_log = cumsum([b_g_init mvnrnd(zeros(3, 1), Q_b_g, N-1)'*dt], 2);
  y_g_log = w_log .+ b_g_log .+ mvnrnd(zeros(3, 1), R_g, N)';
endfunction