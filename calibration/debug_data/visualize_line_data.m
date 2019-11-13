clear all;
close all;
clc;
files1 = dir('lines/lidar/line1_*.csv')
files2 = dir('lines/lidar/line2_*.csv')
files3 = dir('lines/lidar/line3_*.csv')
files4 = dir('lines/lidar/line4_*.csv')

for i = 1:length(files1)
  csv1 = load((strcat('lines/lidar/', files1(i).name)));
  xP1 = csv1(:, 1); yP1 = csv1(:, 2); zP1 = csv1(:, 3);
  
  csv2 = load((strcat('lines/lidar/', files2(i).name)));
  xP2 = csv2(:, 1); yP2 = csv2(:, 2); zP2 = csv2(:, 3);
  
  csv3 = load((strcat('lines/lidar/', files3(i).name)));
  xP3 = csv3(:, 1); yP3 = csv3(:, 2); zP3 = csv3(:, 3);
  
  csv4 = load((strcat('lines/lidar/', files4(i).name)));
  xP4 = csv4(:, 1); yP4 = csv4(:, 2); zP4 = csv4(:, 3);
  
  figure(i)
  plot3(xP1, yP1, zP1, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  plot3(xP2, yP2, zP2, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  plot3(xP3, yP3, zP3, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  plot3(xP4, yP4, zP4, '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','c',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  grid;
  axis equal;
  hold off;
end  

