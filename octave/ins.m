function q_ins = ins(q_ins, y_g, b_g_ins, dt)
#  x = dt/2*(y_g-b_g_ins);
#  q_ins += [-x(1)*q_ins(2)-x(2)*q_ins(3)-x(3)*q_ins(4);
#            x(1)*q_ins(1)+x(2)*q_ins(4)-x(3)*q_ins(3);
#            -x(1)*q_ins(4)+x(2)*q_ins(1)+x(3)*q_ins(2);
#            x(1)*q_ins(3)-x(2)*q_ins(2)+x(3)*q_ins(1)];
#  q_ins /= norm(q_ins);
  y_g_prime = y_g - b_g_ins;
  q_ins = qmult(q_ins, qaxisangle(norm(y_g_prime)*dt, y_g_prime));
  q_ins /= norm(q_ins);
endfunction