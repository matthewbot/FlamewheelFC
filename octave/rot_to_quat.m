function q=rot_to_quat(r)
  q = zeros(4, 1);
  q(1) = sqrt(1 + r(1,1) + r(2,2) + r(3,3))/2;
  q(2) = (sqrt(1 + r(1,1) - r(2,2) - r(3,3))/2)*sign(r(3,2) - r(2,3));
  q(3) = (sqrt(1 - r(1,1) + r(2,2) - r(3,3))/2)*sign(r(1,3) - r(3,1));
  q(4) = (sqrt(1 - r(1,1) - r(2,2) + r(3,3))/2)*sign(r(2,1) - r(1,2));
endfunction
