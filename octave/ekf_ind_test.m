g_e = single([0; 0; -9.8]);
m_e = single([.512299; -0.049209; .857396]);
R_g = single(diag([3e-5, 3e-5, 3e-6]));
R_a = single(diag([1e-4, 1e-4, 1e-4]));
R_m = single(diag([1e-2, 1e-2, 1e-2]));
Q_b_a = single(diag([1e-8, 1e-8, 1e-8]));
Q_b_g = single(diag([1e-8, 1e-8, 1e-8]));
x = single((1:9)');
P = single(ones(9));
for i=1:9
  P(i, i) = 10;
endfor
y_g = single([1; 1; 1]);
y_a = single([2; 2; 2]);
y_m = single([3; 3; 3]);
q_ins = single([1; 0; 0; 0]);
q_ins /= norm(q_ins);

[x, P] = ekf_ind(x, P, y_g, y_a, y_m, q_ins, g_e, m_e, R_g, R_a, R_m, Q_b_g, Q_b_a, 1, 1, .01)
class(x)
class(P)