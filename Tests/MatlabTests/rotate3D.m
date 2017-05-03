function [ rotated_point ] = rotate3D( point, x, y, z )
%ROTATE3D Summary of this function goes here
%   Detailed explanation goes here
    rmat_x = [1 0      0;
              0 cos(x) -sin(x);
              0 sin(x) cos(x)];
          
    rmat_y = [cos(y)  0 sin(y);
              0       1 0;
              -sin(y) 0 cos(y)];
          
    rmat_z = [cos(z) -sin(z) 0;
              sin(z) cos(z)  0;
              0      0       1];
          
    rotated_point = rmat_x * rmat_y * rmat_z * point;

end

