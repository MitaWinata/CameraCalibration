clear all; clc; close all;
%Step 1
au = 557.0943; 
av =712.9824; 
u0 =326.3819; 
v0 =298.6679;
f =80.;
Tx =100; 
Ty =0;
Tz =1500;
Phix =0.8*pi/2; 
Phiy =-1.8*pi/2; 
Phix1 =pi/5; 
image_size = [640 480];

%Step 2
rotation_on_x = [1 0 0; 0 cos(Phix) -sin(Phix);0 sin(Phix) cos(Phix)];
rotation_on_y = [cos(Phiy) 0 sin(Phiy); 0 1 0; -sin(Phiy) 0 cos(Phiy)];
rotation_on_x1 = [1 0 0; 0 cos(Phix1) -sin(Phix1);0 sin(Phix) cos(Phix1)];
rotation_matrix =  rotation_on_x * rotation_on_y * rotation_on_x1;
translation_matrix = [Tx; Ty; Tz];
intrinsic_param = [au 0 u0 0; 0 av v0 0; 0 0 1 0];
extrinsic_param = [rotation_matrix , translation_matrix ; 0 0 0 1 ];


%Step 3- generate six 3D random points
threshold_max = 480;
threshold_min = -480;
points_total = 6;
xyz = create3Dpoints( points_total, threshold_max, threshold_min );

%Step 4 - project 3D points to 2D point
calib_param = intrinsic_param * extrinsic_param;
xyz = [xyz; ones(1,points_total)];
projection = calib_param * xyz;

s = projection(3,:);
sX = projection (1,:);
sY = projection(2,:);

Xu = sX ./ s;
Yu = sY ./ s;
p_2d = [Xu; Yu];

%Step 6 - Method of Hall
A = hall( xyz, p_2d )
    
% Step 7.
% Compare the matrix obtained in Step 6 to the one defined in step 2.
calib_matrix_normalized = calib_param / calib_param(3,4)
error = A(:) - calib_matrix_normalized(:);
error_calib_hall = norm(error,2)

%step 8 -  Add some Gaussian noise to all the 2D points
%producing discrepancies between the 
%range [-1,+1] pixels for the 95% of points
noise = randn(2,points_total)*0.5;
noisy_points = p_2d + noise;
% Again repeat step 6 with the noisy 2D points and the ones
%defined in step 3. 
% Compare the obtained matrix to the one you got in step 6 with the
% non-noisy points
A_noisy = hall( xyz, noisy_points )
error = A_noisy(:) - calib_matrix_normalized(:);
error_noisy_hall = norm(error,2);
%Now compute the 2D points with the obtained matrix
% and compare them to those obtained in step 4 (you can check accuracy 
% computing the discrepancy between points)?
%step 4
p_2d_noisy = get2dProjection( xyz, A_noisy );
eucl_dist = compute_points_distance( p_2d_noisy(),p_2d());
error_p2d_noisy_norm = mean(eucl_dist);

figure(1)
hold on

axis([0 750 0 600]);
xlabel('X'); ylabel('Y');
title('Projection points in 640X480 window');
rectangle('Position',[0,0,640,480]);hold on;scatter(p_2d(1,:),p_2d(2,:)....
,'o','b');hold on; scatter(p_2d_noisy(1,:), p_2d_noisy(2,:),'^','g')

% Step 10. Define the vector X of the method of Faugeras. Compute X 
%by using both least-squares (LS) and Singular Value Decomposition (SVD)
%by using the points of
% Step 3 and Step 4, without noise. 
X_SVD = faugeras_SVD( xyz, p_2d );
X = faugeras_LS( xyz, p_2d );

% Extract the camera parameters from both
% computations. Compare the obtained parameters with the ones defined in Step 1.
[calib_matrix_F, intrinsic_param_F, extrinsic_param_F] ....
= faugeras_calib_matrix(X);
calib_matrix_normalized_F = calib_matrix_F / calib_matrix_F(3,4)
%Step 11.
%[-1,1]
noise_11 = randn(2,points_total)*0.5;
p_2d_noise_11 = p_2d + noise_11;
X_n11 = faugeras_LS( xyz, p_2d_noise_11 );
calib_matrix_F_n11 = faugeras_calib_matrix(X_n11);
calib_matrix_normalized_F_n11 = calib_matrix_F_n11/calib_matrix_F_n11(3,4)
error_Fn11 = calib_matrix_normalized_F_n11(:) - calib_matrix_normalized(:);
error_Fn11 = norm(error_Fn11,2);

p_2d_Fn1 = get2dProjection( xyz, calib_matrix_normalized_F_n11 );
eucl_dist_1 = compute_points_distance( p_2d_Fn1,p_2d);
error_p_2d_Fn1 = mean(eucl_dist_1)

A_noisy_11 = hall( xyz, p_2d_noise_11 )
error_11 = A_noisy_11(:) - calib_matrix_normalized(:);
error_noisy_hall_11 = norm(error_11,2)

p_2d_A1 = get2dProjection( xyz, A_noisy_11 );
eucl_dist_A1 = compute_points_distance( p_2d_A1,p_2d);
error_p_2d_A1 = mean(eucl_dist_A1)


%[-2,2]
noise_22 = randn(2,points_total)*1;
p_2d_noise_22 = p_2d + noise_22;
X_n22 = faugeras_LS( xyz, p_2d_noise_22 );
calib_matrix_F_n22 = faugeras_calib_matrix(X_n22);
calib_matrix_normalized_F_n22  = calib_matrix_F_n22/calib_matrix_F_n22(3,4)
error_Fn22 = calib_matrix_normalized_F_n22(:) - calib_matrix_normalized(:);
error_Fn22 = norm(error_Fn22,2);

p_2d_Fn22 = get2dProjection( xyz, calib_matrix_normalized_F_n22 );
eucl_dist_22 = compute_points_distance( p_2d_Fn22,p_2d);
error_p_2d_Fn22 = mean(eucl_dist_22);

A_noisy_22 = hall( xyz, p_2d_noise_22 )
error_22 = A_noisy_22(:) - calib_matrix_normalized(:);
error_noisy_hall_22 = norm(error_22,2)

p_2d_A22 = get2dProjection( xyz, A_noisy_22 );
eucl_dist_A22 = compute_points_distance( p_2d_A22,p_2d);
error_p_2d_A22 = mean(eucl_dist_A22)

%[-3,3]
noise_33 = randn(2,points_total)*1.5;
p_2d_noise_33 = p_2d + noise_33;
X_n33 = faugeras_LS( xyz, p_2d_noise_33 );
calib_matrix_F_n33 = faugeras_calib_matrix(X_n33);
calib_matrix_normalized_F_n33 = calib_matrix_F_n33/calib_matrix_F_n33(3,4)
error_Fn33 = calib_matrix_normalized_F_n33(:) - calib_matrix_normalized(:);
error_Fn33 = norm(error_Fn33,2);

p_2d_Fn33 = get2dProjection( xyz, calib_matrix_normalized_F_n33 );
eucl_dist_33 = compute_points_distance( p_2d_Fn33,p_2d);
error_p_2d_Fn33 = mean(eucl_dist_33);


A_noisy_33 = hall( xyz, p_2d_noise_33 )
error_33 = A_noisy_33(:) - calib_matrix_normalized(:);
error_noisy_hall_33 = norm(error_33,2)

p_2d_A33 = get2dProjection( xyz, A_noisy_33 );
eucl_dist_A33 = compute_points_distance( p_2d_A33,p_2d);
error_p_2d_A33 = mean(eucl_dist_A33)

projection_F = calib_matrix_normalized_F * xyz;
s = projection_F(3,:);
sX = projection_F(1,:);
sY = projection_F(2,:);

Xu_F = sX ./ s;
Yu_F = sY ./ s;
p_2d_F = [Xu; Yu];

%Step 12
DrawSetting(f, intrinsic_param_F, extrinsic_param_F, xyz, p_2d_F );
