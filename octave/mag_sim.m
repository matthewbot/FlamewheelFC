function [y_m_log] = mag_sim(q_log, m_e, R_m, dt)
  N = columns(q_log);
  y_m_log = mvnrnd(zeros(3, 1), R_m, N)';
  for i = 1:N
    y_m_log(:, i) += C_mat(q_log(:, i))*m_e;
  endfor
endfunction