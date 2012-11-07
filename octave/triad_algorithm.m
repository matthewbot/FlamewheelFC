# NOAA: 24,137.66 nT -2,318.67 nT 40,396.99 nT

function [A, fixed]=triad_algorithm(accel, mag)
  accel_fixed = [0; 0; -9.8];
  mag_fixed = [2.4138e+04; -2.3186e+03; 4.0397e+04];

  fixed = zeros(3);
  fixed(:, 1) = accel_fixed/norm(accel_fixed);
  cr = cross(accel_fixed, mag_fixed);
  fixed(:, 2) = cr/norm(cr);
  cr = cross(fixed(:, 1), fixed(:, 2));
  fixed(:, 3) = cr/norm(cr);

  measured = zeros(3);
  measured(:, 1) = accel/norm(accel);
  cr = cross(accel, mag);
  measured(:, 2) = cr/norm(cr);
  cr = cross(measured(:, 1), measured(:, 2));
  measured(:, 3) = cr;

  A = measured*fixed';
endfunction