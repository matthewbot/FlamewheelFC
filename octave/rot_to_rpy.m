function rpy = rot_to_rpy(R)
  r = atan2(R(3,2), R(3,3));
  y = atan2(R(2,1), R(1,1));
  p = atan2(-R(3,1), cos(y)*R(1,1) + sin(y)*R(2,1));
  rpy = [r;p;y];
endfunction