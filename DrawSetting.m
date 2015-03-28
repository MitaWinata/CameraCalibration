
function [] = DrawSetting( f, intrinsic_param_F, extrinsic_param_F, xyz, p2d )

%Part 3
%Step 12 
figure(2)
hold on
grid on

%Drawing plane
xlim([-500 500]);xlabel('x');
ylim([-900 500]);ylabel('y');
zlim([-500 1300]);zlabel('z');

%World Coordinate System
%Define the points
x_orig_w=[100 0 0]';
y_orig_w=[0 100 0]';
z_orig_w=[0 0 100]';
orig_w=[0 0 0 1]';
%Draw the vectors
 quiver3(orig_w(1),orig_w(2),orig_w(3),x_orig_w(1),x_orig_w(2),x_orig_w(3)....
 ,'Color','r','LineWidth',2);
 quiver3(orig_w(1),orig_w(2),orig_w(3),y_orig_w(1),y_orig_w(2),y_orig_w(3)....
 ,'Color','b','LineWidth',2);
 quiver3(orig_w(1),orig_w(2),orig_w(3),z_orig_w(1),z_orig_w(2),z_orig_w(3)....
 ,'Color','g','LineWidth',2);
%Label the vectors
 text(orig_w(1),orig_w(2),orig_w(3),'World Coordinate System');
text(x_orig_w(1),x_orig_w(2),x_orig_w(3),'x');
 text(y_orig_w(1),y_orig_w(2),y_orig_w(3),'y'); 
text(z_orig_w(1),z_orig_w(2),z_orig_w(3),'z');
 hold on;

%Camera Coordinate System
cKw = extrinsic_param_F; 
%Origin of camera referring to camera coordinate system
x_orig_c = [10 0 0 1]';
y_orig_c = [0 10 0 1]';
z_orig_c = [0 0 10 1]';
orig_c   = [0 0 0 1]';
% Change the reference point to World coordinate system
%Origin of camera referring to world coordinate system
c_to_w = getCameraPointInWorld( cKw, orig_c )
x_orig_c_to_w= getCameraPointInWorld( cKw, x_orig_c);
y_orig_c_to_w= getCameraPointInWorld( cKw, y_orig_c);
z_orig_c_to_w= getCameraPointInWorld( cKw, z_orig_c);
%%% Plot
hold on;
plot3([c_to_w(1) x_orig_c_to_w(1)],[c_to_w(2) x_orig_c_to_w(2)],[c_to_w(3) ....
    x_orig_c_to_w(3)],'r');
hold on;
plot3([c_to_w(1) y_orig_c_to_w(1)],[c_to_w(2) y_orig_c_to_w(2)],[c_to_w(3)....
    y_orig_c_to_w(3)],'g');
hold on;
plot3([c_to_w(1) z_orig_c_to_w(1)],[c_to_w(2) z_orig_c_to_w(2)],[c_to_w(3) ....
    z_orig_c_to_w(3)],'b');
hold on;
%Label
text(c_to_w(1),c_to_w(2),c_to_w(3),'Camera Coordinate System');
text(x_orig_c_to_w(1),x_orig_c_to_w(2),x_orig_c_to_w(3),'x');
text(y_orig_c_to_w(1),y_orig_c_to_w(2),y_orig_c_to_w(3),'y');
text(z_orig_c_to_w(1),z_orig_c_to_w(2),z_orig_c_to_w(3),'z');

%hold on;
%Image plane 
o_i=[0 0]';
a_i=[0 640]';
b_i=[ 480 640]';
c_i=[ 480 0]';
%Image plane to camera
o_i_to_c = getImagePlaneInCamera( o_i(1), o_i(2), f, intrinsic_param_F );
a_i_to_c = getImagePlaneInCamera( a_i(1), a_i(2), f, intrinsic_param_F );
b_i_to_c = getImagePlaneInCamera( b_i(1), b_i(2), f, intrinsic_param_F );
c_i_to_c = getImagePlaneInCamera( c_i(1), c_i(2), f, intrinsic_param_F );
%Camera to World
o_i_to_w= getCameraPointInWorld( cKw, o_i_to_c);
a_i_to_w= getCameraPointInWorld( cKw, a_i_to_c);
b_i_to_w= getCameraPointInWorld( cKw, b_i_to_c);
c_i_to_w= getCameraPointInWorld( cKw, c_i_to_c);

line([o_i_to_w(1) a_i_to_w(1)],[o_i_to_w(2) a_i_to_w(2)],[o_i_to_w(3) ....
    a_i_to_w(3)],'Color','bl','LineWidth',1);
line([o_i_to_w(1) c_i_to_w(1)],[o_i_to_w(2) c_i_to_w(2)],[o_i_to_w(3) ....
    c_i_to_w(3)],'Color','bl','LineWidth',1);
line([b_i_to_w(1) c_i_to_w(1)],[b_i_to_w(2) c_i_to_w(2)],[b_i_to_w(3) ....
    c_i_to_w(3)],'Color','bl','LineWidth',1);
line([b_i_to_w(1) a_i_to_w(1)],[b_i_to_w(2) a_i_to_w(2)],[b_i_to_w(3) ....
    a_i_to_w(3)],'Color','bl','LineWidth',1);
%Label
text(b_i_to_w(1),b_i_to_w(2),b_i_to_w(3),'640 x 480','Color','b');


u0 = intrinsic_param_F(1,3);
v0 = intrinsic_param_F(2,3);
o_ix=[0 0]';
a_ix=[0 2*v0]';
b_ix=[2*u0 0]';
c_ix=[2*u0 2*v0]';
%Image plane to camera
o_i_to_cx = getImagePlaneInCamera( o_ix(1), o_ix(2), f, intrinsic_param_F );
a_i_to_cx = getImagePlaneInCamera( a_ix(1), a_ix(2), f, intrinsic_param_F );
b_i_to_cx = getImagePlaneInCamera( b_ix(1), b_ix(2), f, intrinsic_param_F );
c_i_to_cx = getImagePlaneInCamera( c_ix(1), c_ix(2), f, intrinsic_param_F );
%Camera to World
o_i_to_wx= getCameraPointInWorld( cKw, o_i_to_cx);
a_i_to_wx= getCameraPointInWorld( cKw, a_i_to_cx);
b_i_to_wx= getCameraPointInWorld( cKw, b_i_to_cx);
c_i_to_wx= getCameraPointInWorld( cKw, c_i_to_cx);
line([o_i_to_wx(1) a_i_to_wx(1)],[o_i_to_wx(2) a_i_to_wx(2)],[o_i_to_wx(3)....
    a_i_to_wx(3)],'Color','g','LineWidth',1);
line([o_i_to_wx(1) c_i_to_wx(1)],[o_i_to_wx(2) c_i_to_wx(2)],[o_i_to_wx(3) ....
    c_i_to_wx(3)],'Color','g','LineWidth',1);
line([b_i_to_wx(1) c_i_to_wx(1)],[b_i_to_wx(2) c_i_to_wx(2)],[b_i_to_wx(3)....
    c_i_to_wx(3)],'Color','g','LineWidth',1);
line([b_i_to_wx(1) a_i_to_wx(1)],[b_i_to_wx(2) a_i_to_wx(2)],[b_i_to_wx(3)....
    a_i_to_wx(3)],'Color','g','LineWidth',1);
line([b_i_to_wx(1) o_i_to_wx(1)],[b_i_to_wx(2) o_i_to_wx(2)],[b_i_to_wx(3) ....
    o_i_to_wx(3)],'Color','g','LineWidth',1);
line([c_i_to_wx(1) a_i_to_wx(1)],[c_i_to_wx(2) a_i_to_wx(2)],[c_i_to_wx(3) ....
    a_i_to_wx(3)],'Color','g','LineWidth',1);
%Label
text(b_i_to_wx(1),b_i_to_wx(2),b_i_to_wx(3),'Image border limit','Color','g');

hold on;
%Label
text(o_i_to_w(1),o_i_to_w(2),o_i_to_w(3),'Image plane');

%Focal point- from camera to world
focal_point = [0 0 f 1]';
focal_point_c_to_w = getCameraPointInWorld( cKw, focal_point );
hold on;
plot3(focal_point_c_to_w(1),focal_point_c_to_w(2),focal_point_c_to_w(3),'y*');
hold on;
text(focal_point_c_to_w(1),focal_point_c_to_w(2),focal_point_c_to_w(3),'[u0, v0]');
hold on
line([c_to_w(1) focal_point_c_to_w(1)],[c_to_w(2) focal_point_c_to_w(2)],....
[c_to_w(3) focal_point_c_to_w(3)],'Color','y','LineWidth',1);
focal_point_l_text = [0 0 f/2 1 ]';
focal_point_l_text_w = getCameraPointInWorld( cKw, focal_point_l_text );
hold on;
text(focal_point_l_text_w(1),focal_point_l_text_w(2),focal_point_l_text_w(3),....
'Focal length','Color','y');

%2D Projection points
for i = 1: size(p2d,2)
    point_i_to_c = getImagePlaneInCamera( p2d(1,i), p2d(2,i), f, intrinsic_param_F );
    point_i_to_w = getCameraPointInWorld( cKw, point_i_to_c);
    hold on
    plot3(point_i_to_w(1),point_i_to_w(2),point_i_to_w(3),'xr');
    hold on
    plot3(xyz(1,i),xyz(2,i),xyz(3,i), '^g');
    hold on
    line([xyz(1,i) c_to_w(1)],[xyz(2,i) c_to_w(2)],[xyz(3,i) c_to_w(3)],....
    'Color','b','LineWidth',1);
end

end