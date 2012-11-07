function rpy=quat_to_rpy(q)
  q /= norm(q);
  rpy = [
         atan2(2*q(2)*q(1) - 2*q(2)*q(4), 1 - 2*q(2)*q(2) - 2*q(4)*q(4));
         atan2(2*q(3)*q(1) - 2*q(2)*q(4), 1 - 2*q(3)*q(3) - 2*q(4)*q(4));
         asin(2*q(2)*q(3)+2*q(4)*q(1))
         ];
endfunction
