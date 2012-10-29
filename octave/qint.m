function q_log = qint(w_log, qinit, dt)
  N = columns(w_log);
  q = qinit;
  q_log = zeros(4, N);

  for i=1:N
    angle = norm(w_log(:, i))*dt;
    q = qmult(q, qaxisangle(angle, w_log(:, i)));
    q /= norm(q);
    q_log(:, i) = q;
  endfor
endfunction