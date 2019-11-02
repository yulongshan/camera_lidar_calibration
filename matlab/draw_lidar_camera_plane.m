close all;
points3dP = csvread('points3dplane.csv');
point3dPEdges = csvread('points3dplane_edges.csv');

xP = points3dP(:, 1);
yP = points3dP(:, 2);
zP = points3dP(:, 3);

xPE = point3dPEdges(:, 1);
yPE = point3dPEdges(:, 2);
zPE = point3dPEdges(:, 3);

subplot(211)
plot3(xP, yP, zP, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
hold on;
plot3(xPE, yPE, zPE, 'o', 'LineWidth',2,...
    'MarkerSize',5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[1,0,0]);
hold on;


origin = [0, 0, 0];
x =[0.2, 0, 0];
y =[0, 0.2, 0];
z =[0, 0, 0.2];

axis_pts_x = [origin; x];
axis_pts_y = [origin; y];
axis_pts_z = [origin; z];

plot3(axis_pts_x(:, 1), axis_pts_x(:, 2), axis_pts_x(:, 3), 'LineWidth', 5, 'r');
hold on;
plot3(axis_pts_y(:, 1), axis_pts_y(:, 2), axis_pts_y(:, 3), 'LineWidth', 5, 'g');
hold on;
plot3(axis_pts_z(:, 1), axis_pts_z(:, 2), axis_pts_z(:, 3), 'LineWidth', 5, 'b');
hold off;

axis equal
grid;
xlim([0, 2]);
ylim([-0.5, 1]);
set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 25);
set(gca, 'FontWeight', 'bold');
ylabel('Label Y axis')
xlabel('Label X axis')
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

%%

C_R_W = [-0.6873012813551714, 0.7211890857465928, -0.08662130943095533;
          0.7200546825526566, 0.6921555308862722, 0.04941634544901293;
          0.09559394741055835, -0.02840816191760558, -0.995014961472907];
C_t_W = [-0.0005398960451074554; -0.540902862512962; 1.498845076463349];  

C_T_W = eye(4);
C_T_W(1:3, 1:3) = C_R_W;
C_T_W(1:3, 4) = C_t_W; 
C1_T_C = [0  0 1 0; 
         -1  0 0 0; 
          0 -1 0 0; 
          0  0 0 1]; 
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
objectPts_C = C1_T_C*C_T_W*objectPts_W;
objectPts_C = objectPts_C(1:3, 1:4);

objectPts_C = objectPts_C'
objectPts_C = [objectPts_C; objectPts_C(1, :)];
x = objectPts_C(:, 1);
y = objectPts_C(:, 2);
z = objectPts_C(:, 3);
subplot(212)
plot3(x, y, z, 'LineWidth', 4, 'b');
hold on;
origin = [0, 0, 0];
x =[0.2, 0, 0];
y =[0, -0.2, 0];
z =[0, 0, -0.2];

axis_pts_x = [origin; x];
axis_pts_y = [origin; y];
axis_pts_z = [origin; z];
plot3(axis_pts_x(:, 1), axis_pts_x(:, 2), axis_pts_x(:, 3), 'LineWidth', 5, 'b');
hold on;
plot3(axis_pts_y(:, 1), axis_pts_y(:, 2), axis_pts_y(:, 3), 'LineWidth', 5, 'r');
hold on;
plot3(axis_pts_z(:, 1), axis_pts_z(:, 2), axis_pts_z(:, 3), 'LineWidth', 5, 'g');
hold on;

axis equal;
grid;
xlim([0, 2]);
ylim([-0.5, 0.6]);
zlim([-0.4, 0.6]);
xlabel('z axis');
ylabel('x axis');
zlabel('y axis');

%% 
pixel_values = K*C_T_W(1:3, 1:4)*objectPts_W;
pixel_values = pixel_values./pixel_values(3,:)

u = pixel_values(1, :);
v = pixel_values(2, :);

u = [u, u(1)];
v = [v, v(1)];
figure(3)
plot(u, -v, 'LineWidth', 5, 'b')
axis equal;
xlim([0, 800]);
ylim([0, -592]);


          

