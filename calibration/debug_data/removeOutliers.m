function [points3d_filtered] = removeOutliers (points3d, dim_x, dim_y)
  mean_3dPos = mean(points3d);
  max_dist = sqrt(dim_x*dim_x + dim_y*dim_y);
  max_dist = max_dist/2;
  k = 1;
  for i=1:length(points3d)
    X = points3d(i, 1);
    Y = points3d(i, 2);
    Z = points3d(i, 3);
    X_mean = mean_3dPos(1);
    Y_mean = mean_3dPos(2);
    Z_mean = mean_3dPos(3);
    distance = (X-X_mean)*(X-X_mean)+(Y-Y_mean)*(Y-Y_mean)+(Z-Z_mean)*(Z-Z_mean);
    distance = sqrt(distance);
    if distance <= max_dist
      points3d_filtered(k,:) = points3d(i,:);
      k = k + 1;
    end
  end
end
