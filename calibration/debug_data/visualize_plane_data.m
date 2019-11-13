clear all;
close all;
clc;
files = dir('plane/lidar/*.csv')
figure(1)
for i = 1:length(files)
  csv = load((strcat('plane/lidar/', files(i).name)));
  xP = csv(:, 1);
  yP = csv(:, 2);
  zP = csv(:, 3);

  plot3(xP, yP, zP, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
end  
 grid;
 axis equal;
