function point = getImagePlaneInCamera( x, y, f, intrinsic_param )

u0 = intrinsic_param(1,3);
v0 = intrinsic_param(2,3);
au = intrinsic_param(1,1);
av = intrinsic_param(2,2);

ku = -au / f;
kv = -av / f;

xI = (-x+u0) / ku; 
yI = (-y+v0) / kv;
zI = f;

point = [  xI; yI; zI; 1 ];


end
