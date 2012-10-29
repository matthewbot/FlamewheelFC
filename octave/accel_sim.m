function [y_a_log, b_a_log] = accel_sim(q_log, a_ext_log, g_e, b_a_init, R_a, Q_b_a, dt)
  N = columns(q_log);
  b_a_log = cumsum([b_a_init mvnrnd(zeros(3, 1), Q_b_a, N-1)'*dt], 2);
  y_a_log = b_a_log + mvnrnd(zeros(3, 1), R_a, N)';
  for i = 1:N
    y_a_log(:, i) += C_mat(q_log(:, i))*(a_ext_log(:, i) + g_e);
  endfor
endfunction