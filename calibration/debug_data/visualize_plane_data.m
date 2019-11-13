clear all;
close all;
clc;
files_lidar = dir('plane/lidar/*.csv')
files_camera = dir('plane/camera/*.csv')

side_len = 0.608;
objectPts_W = [0, 0, 0, 1; 
               0, side_len, 0, 1; 
               side_len, side_len, 0, 1;
               side_len, 0, 0, 1]';
fx = 6.4372590342756985e+02;
fy = 6.4372590342756985e+02;
cx = 3.9534097290039062e+02;
cy = 3.0199901199340820e+02;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
C1_T_C = [0  0 1 0; 
         -1  0 0 0; 
          0 -1 0 0; 
          0  0 0 1]; 
          
for i = 1:length(files_lidar)
  csv_lidar = load((strcat('plane/lidar/', files_lidar(i).name)));
  xP = csv_lidar(:, 1);
  yP = csv_lidar(:, 2);
  zP = csv_lidar(:, 3);
  subplot(121);
  color_no = (i-1)/length(files_lidar);
  plot3(xP, yP, zP, '.', 'MarkerSize', 10);
  hold on;
  csv_camera = load((strcat('plane/camera/', files_camera(i).name)));
  R_t = csv_camera;
  R_t = [R_t; 0 0 0 1];
  C1_T_W = C1_T_C*R_t;
  objectPts_C1 = C1_T_W*objectPts_W;
  objectPts_C1 = objectPts_C1';
  objectPts_C1 = [objectPts_C1; objectPts_C1(1, :)];
  xC = objectPts_C1(:, 1);
  yC = objectPts_C1(:, 2);
  zC = objectPts_C1(:, 3);
  subplot(122);
  plot3(xC, yC, zC, '-','LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor',[color_no, color_no, color_no]);
  hold on
  color_no
end 

subplot(121);
hold off;
grid;
axis equal;
 
subplot(122);
hold off;
grid;
axis equal;
