function eucl_dist = compute_points_distance(points2d,points2dnoise)

npoints = size(points2d,2);
eucl_dist = zeros(npoints,1);
for i = 1:npoints
    eucl_dist(i) = sqrt((points2d(1,i) - points2dnoise(1,i))^2 +....
    (points2d(1,i) - points2dnoise(1,i))^2);
end



end