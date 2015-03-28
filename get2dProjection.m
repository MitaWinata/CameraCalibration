function [ points2D ] = get2dProjection( points3D, calib_matrix )
%GET2DPROJECTION Summary of this function goes here
%   Detailed explanation goes here

projection_points = calib_matrix * points3D;
s = projection_points(3,:);
sX = projection_points (1,:);
sY = projection_points(2,:);
Xu = sX ./ s;
Yu = sY ./ s;
points2D = [Xu; Yu];

end

