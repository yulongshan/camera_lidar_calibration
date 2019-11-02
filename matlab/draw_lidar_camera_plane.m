close all;
points3dP = csvread('points3dplane.csv');
point3dPEdges = csvread('points3dplane_edges.csv');

xP = points3dP(:, 1);
yP = points3dP(:, 2);
zP = points3dP(:, 3);

xPE = point3dPEdges(:, 1);
yPE = point3dPEdges(:, 2);
zPE = point3dPEdges(:, 3);

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