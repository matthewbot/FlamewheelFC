load -ascii data.csv
N = rows(data);
board_rpy = data(:, 1:3)';
accel = data(:, 4:6)';
mag_tmp = data(:, 7:9)';

#mag = [ mag_tmp(2, :); mag_tmp(1, :); -mag_tmp(3, :) ];
mag = mag_tmp;

r_log = zeros(3, 3, N);
val_log = zeros(1, N);
norm_log = zeros(1, N);
rpy_log = zeros(3, N);

for i=1:N
  r_log(:, :, i) = triad_algorithm(accel(:, i), mag(:, i));
  val_log(i) = r_log(3,2,i)*r_log(2,1,i)+r_log(2,2,i)*r_log(2,2,i);
  norm_log(i) = norm(mag(:, i));
endfor
for i=1:N
  rpy_log(:, i) = rot_to_rpy(r_log(:, :, i));
endfor

figure();
plot(rpy_log'/pi*180)
