function q = qaxisangle(theta, axis)
  if abs(theta) < 1e-8
    q = [1; 0; 0; 0];
    return
  endif
  axis /= norm(axis);
  q = [cos(theta/2); axis*sin(theta/2)];
endfunction