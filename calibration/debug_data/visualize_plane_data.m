clear all;
close all;
clc;
files = dir('plane/lidar/*.csv')

for i = 1:length(files)
  csv = load((strcat('plane/lidar/', files(i).name)));
  xP = csv(:, 1);
  yP = csv(:, 2);
  zP = csv(:, 3);
  figure(1);
  plot3(xP, yP, zP, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor',[rand, rand, rand]);
  hold on;
end  
 grid;
 axis equal;
