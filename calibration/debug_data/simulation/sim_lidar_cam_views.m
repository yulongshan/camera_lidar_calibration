clear all;
close all;
clc;
side_len = 1.016;
objectPts_W = [0, 0, 0, 1; 
               0, side_len, 0, 1; 
               side_len, side_len, 0, 1;
               side_len, 0, 0, 1];

img_ht = 592;
img_wdth = 800;
fx = 6.4372590342756985e+02;
fy = 6.4372590342756985e+02;
cx = 3.9534097290039062e+02;
cy = 3.0199901199340820e+02;
K = [fx,  0, cx;
      0, fy, cy;
      0,  0,  1];
      
X = 0.12;
Y = -0.10;
Z = -0.10;
      
C_T_L = [0 -1  0 X;
         0  0 -1 Y;
         1  0  0 Z;
         0  0  0 1];
         
C_R_L = C_T_L(1:3, 1:3);
C_t_L = C_T_L(1:3, 4);         
L_T_C = inv(C_T_L);
            
X1 = objectPts_W(1, 1:3)';
X2 = objectPts_W(2, 1:3)';
X3 = objectPts_W(3, 1:3)';
X4 = objectPts_W(4, 1:3)';

no_of_line_pts = 5;
l1_w = linspace(X4, X1, no_of_line_pts);
l1_w = [l1_w; ones(1, no_of_line_pts)];

l2_w = linspace(X1, X2, no_of_line_pts);
l2_w = [l2_w; ones(1, no_of_line_pts)];

l3_w = linspace(X2, X3, no_of_line_pts);
l3_w = [l3_w; ones(1, no_of_line_pts)];

l4_w = linspace(X3, X4, no_of_line_pts);
l4_w = [l4_w; ones(1, no_of_line_pts)];

no_of_views = 1;
for view_no = 1:no_of_views
  roll_mat = getRotX(randn*180/pi);
  pitch_mat = getRotY(randn*180/pi);
  yaw_mat = getRotZ(randn*180/pi);
  CRW = roll_mat*pitch_mat*yaw_mat;
  lim_xy = 0.5;
  lim_z = 3;
  CTW = [2*lim_xy*rand(1, 1)-lim_xy; 
         2*lim_xy*rand(1, 1)-lim_xy;
           lim_z*rand(1, 1)];

  C_RT_W = [CRW CTW; 0 0 0 1];

  l1_c = C_RT_W*l1_w;
  l2_c = C_RT_W*l2_w;
  l3_c = C_RT_W*l3_w;
  l4_c = C_RT_W*l4_w;

  l1_l = L_T_C*l1_c;
  csvwrite(strcat('l1_l', mat2str(view_no), '.csv'), l1_l');
  l2_l = L_T_C*l2_c;
  csvwrite(strcat('l2_l', mat2str(view_no), '.csv'), l2_l');
  l3_l = L_T_C*l3_c;
  csvwrite(strcat('l3_l', mat2str(view_no), '.csv'), l3_l');
  l4_l = L_T_C*l4_c;
  csvwrite(strcat('l4_l', mat2str(view_no), '.csv'), l4_l');

  plane_3d_pts_w = [];
  for i = 0:0.1:side_len
    for j = 0:0.1:side_len
      plane_3d_pts_w = [plane_3d_pts_w; i j 0 1];
    end
  end

  plane_3d_pts_c = C_RT_W*plane_3d_pts_w';
  plane_3d_pts_c = plane_3d_pts_c';

  plane_3d_pts_l = L_T_C*plane_3d_pts_c';
  plane_3d_pts_l = plane_3d_pts_l';
  csvwrite(strcat('plane_pts_lidar', mat2str(view_no), '.csv'), plane_3d_pts_l);
  
  uvw = K*C_RT_W(1:3,1:4)*objectPts_W';
  uv = uvw./uvw(end, :);
  uv = [uv, uv(:, 1)];

  pt2d_1 = uv(:, 1);
  pt2d_2 = uv(:, 2);
  pt2d_3 = uv(:, 3);
  pt2d_4 = uv(:, 4);

  l1_pt2d = linspace(pt2d_4, pt2d_1, no_of_line_pts);
  l2_pt2d = linspace(pt2d_1, pt2d_2, no_of_line_pts);
  l3_pt2d = linspace(pt2d_2, pt2d_3, no_of_line_pts);
  l4_pt2d = linspace(pt2d_3, pt2d_4, no_of_line_pts);

  r3 = CRW(:, 3);
  tvec = CTW;
  
  r3tvec = [r3', tvec'];
  csvwrite(strcat('r3tvec', mat2str(view_no), '.csv'), r3tvec);
  
  line1 = getLineEquation(pt2d_4, pt2d_1);
  line2 = getLineEquation(pt2d_1, pt2d_2);
  line3 = getLineEquation(pt2d_2, pt2d_3);
  line4 = getLineEquation(pt2d_3, pt2d_4);

  normal1 = K'*line1/norm(K'*line1);
  normal2 = K'*line2/norm(K'*line2);
  normal3 = K'*line3/norm(K'*line3);
  normal4 = K'*line4/norm(K'*line4);
  
  all_normals = [normal1';
                 normal2';
                 normal3';
                 normal4'];
  csvwrite(strcat('all_normals', mat2str(view_no), '.csv'), all_normals);
end

x_plane_lidar = plane_3d_pts_l(:,1);
y_plane_lidar = plane_3d_pts_l(:,2);
z_plane_lidar = plane_3d_pts_l(:,3);

planar_points_L = [x_plane_lidar';
                   y_plane_lidar';
                   z_plane_lidar'];                   
figure(1)
subplot(131)
plot3(l1_l(1,:), l1_l(2,:), 
      l1_l(3,:), '*');
hold on;
plot3(l2_l(1,:), l2_l(2,:), 
      l2_l(3,:), '+');
hold on;
plot3(l3_l(1,:), l3_l(2,:), 
      l3_l(3,:), '.');
hold on;
plot3(l4_l(1,:), l4_l(2,:), 
      l4_l(3,:), 'd');
hold off;
grid;
axis equal;
xlim([0, 4]);
ylim([-2, 2]);

subplot(132)
plot3(plane_3d_pts_l(:,1), plane_3d_pts_l(:,2), 
      plane_3d_pts_l(:,3), '*');
grid;
axis equal;
xlim([0, 6]);
ylim([-3, 3]);

subplot(133)
plot(l1_pt2d(1, :), -l1_pt2d(2, :), '*');
hold on;
plot(l2_pt2d(1, :), -l2_pt2d(2, :), '+');
hold on;
plot(l3_pt2d(1, :), -l3_pt2d(2, :), '.');
hold on;
plot(l4_pt2d(1, :), -l4_pt2d(2, :), 'd');
hold off;
grid;
axis equal;
xlim([0 img_wdth]);
ylim([-img_ht, 0]);



%C_R_L = getRotX(randn*180/pi)*getRotY(randn*180/pi)*getRotZ(randn*180/pi);
%C_T_L = [randn; randn; randn];

Line_dotProd1 = sum(normal1'*(C_R_L*l1_l(1:3,:)+C_t_L))
Line_dotProd2 = sum(normal2'*(C_R_L*l2_l(1:3,:)+C_t_L))
Line_dotProd3 = sum(normal3'*(C_R_L*l3_l(1:3,:)+C_t_L))
Line_dotProd4 = sum(normal4'*(C_R_L*l4_l(1:3,:)+C_t_L))
Plane_dotProduct = sum(r3'*(C_R_L*planar_points_L+C_t_L-tvec))              