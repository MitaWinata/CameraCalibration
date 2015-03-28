function A = hall( p3d, p2d  )
%HALL Summary of this function goes here
%   Detailed explanation goes here
Q = []; 
B = [];

for row = 1:size(p2d,2)
   idx = 2*row-1;
    
   r1 = [ p3d(1,row) p3d(2,row) p3d(3,row) 1 0 0 0 0  ....
       -p2d(1,row)*p3d(1,row) -p2d(1,row)*p3d(2,row) ....
       -p2d(1,row)*p3d(3,row) ];
   r2 = [ 0 0 0 0 p3d(1,row) p3d(2,row) p3d(3,row) 1 ....
       -p2d(2,row)*p3d(1,row) -p2d(2,row)*p3d(2,row) ....
       -p2d(2,row)*p3d(3,row) ];
   Q(idx,: ) = r1;
   Q(idx+1, : ) = r2;
   
   B(idx,: ) = p2d(1,row);
   B(idx+1, : ) = p2d(2,row);
   
end
%Get the calibration matrix using Least Squares
A = pinv(Q)*B;
A = [A; 1];
A = reshape(A, 4, 3)';

end

