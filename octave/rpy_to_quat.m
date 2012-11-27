function q = rpy_to_quat(rpy)
  cr = cos(rpy(1)/2);
  sr = sin(rpy(1)/2);
  cp = cos(rpy(2)/2);
  sp = sin(rpy(2)/2);
  cy = cos(rpy(3)/2);
  sy = sin(rpy(3)/2);

  q = [
       cr*cp*cy + sr*sp*sy;
       sr*cp*cy - cr*sp*sy;
       cr*sp*cy + sr*cp*sy;
       cr*cp*sy - sr*sp*cy ];
endfunction