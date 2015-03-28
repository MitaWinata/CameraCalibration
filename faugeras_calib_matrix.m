function [calib_param_F, intrinsic_param_F, extrinsic_param_F ] ....
= faugeras_calib_matrix( X )
%FAUGERAS_CALIB_MATRIX Summary of this function goes here
%   Detailed explanation goes here
T1 = X(1:3); T1=T1';
T2 = X(4:6); T2=T2';
T3 = X(7:9); T3=T3';
C1 = X(10);
C2 = X(11);
%Intrinsics
u0_F = ( T1*transpose(T2) ) / (norm(T2))^2 ;
v0_F = ( T2*transpose(T3) ) / (norm(T2))^2 ;
au_F = (norm(cross(transpose(T1),transpose(T2)))) / norm(T2)^2;
av_F = (norm(cross(transpose(T2),transpose(T3)))) / norm(T2)^2;

%Extrinsics
Tx_F = (norm(T2)/(norm(cross(transpose(T1),transpose(T2)))))*(C1-u0_F);
Ty_F = (norm(T2)/(norm(cross(transpose(T2),transpose(T3)))))*(C2-v0_F);
Tz_F = 1 / norm(T2);
r1_F = (norm(T2)/(norm(cross(transpose(T1),transpose(T2)))))*(T1-u0_F*T2 );
r2_F = (norm(T2)/(norm(cross(transpose(T2),transpose(T3)))))*(T3-v0_F*T2 );
r3_F = T2 / norm(T2);

rotation_matrix_F =  [r1_F; r2_F; r3_F];
translation_matrix_F = [Tx_F; Ty_F; Tz_F];
intrinsic_param_F = [au_F 0 u0_F 0; 0 av_F v0_F 0; 0 0 1 0];
extrinsic_param_F = [rotation_matrix_F , translation_matrix_F ; 0 0 0 1 ];
calib_param_F = intrinsic_param_F * extrinsic_param_F;
%calib_matrix_normalized_F = calib_param_F / calib_param_F(3,4);

end

