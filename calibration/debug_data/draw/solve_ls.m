clear all;
clc;
files_lidar = dir('../left/plane/lidar/*.csv')
files_camera = dir('../left/plane/camera/*.csv')

side_len = 1.016;
objectPts_W = [0, 0, 0, 1; 
               0, side_len, 0, 1; 
               side_len, side_len, 0, 1;
               side_len, 0, 0, 1]';
fx = 6.4372590342756985e+02;
fy = 6.4372590342756985e+02;
cx = 3.9534097290039062e+02;
cy = 3.0199901199340820e+02;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

n = length(files_lidar);
A = [];
b = [];
for i = 1:n
  csv_lidar = load((strcat('../left/plane/lidar/', files_lidar(i).name)));
  csv_camera = load((strcat('../left/plane/camera/', files_camera(i).name)));
  X_l = csv_lidar(:, 1); Y_l = csv_lidar(:, 2); Z_l = csv_lidar(:, 3);
  c_RT_w = csv_camera;
  r3 = c_RT_w(:, 3);
  tvec = c_RT_w(:, 4);
  bi = r3'*tvec;
  a1 = r3(1).*[X_l, Y_l, Z_l];
  a2 = r3(2).*[X_l, Y_l, Z_l];
  a3 = r3(3).*[X_l, Y_l, Z_l];
  a4 = r3'.*ones(size(a3));
  a = [a1, a2, a3, a4];
  A = [A;a];
  b = [b; bi*ones(size(a, 1), 1)];
end
x = A\b;
R_ = reshape(x(1:9), 3, 3)';
[U, S, V] = svd(R_);
R = U*V'
alpha = norm(R)/norm(R_)
t = alpha*x(10:12)
%t_ = x(10:12);
%tz_ = t_(3);