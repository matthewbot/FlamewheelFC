function motors = motor_mapper(torque, thrust)
  M = [ 1  1 -1  1;
       -1  1  1  1;
       -1 -1 -1  1;
       1  -1  1  1];
  motors = M * [torque; thrust];
endfunction