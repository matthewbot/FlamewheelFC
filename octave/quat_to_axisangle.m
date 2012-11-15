function [axis, angle] = quat_to_axisangle(q)
  angle = 2*acos(q(1));
  s = sqrt(1-q(1)*q(1));
  if s < 1e-5
    axis = q(2:4);
  else
    axis = q(2:4)/s;
  endif
endfunction