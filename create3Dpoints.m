function points3D = create3Dpoints( points_total,threshold_max,threshold_min)
%CREATE3DPOINTS Summary of this function goes here
%   Detailed explanation goes here

points3D = rand(3, points_total) * (threshold_max-threshold_min) + threshold_min;

end

