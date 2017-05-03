function [ rmat ] = Project( p0, p1, p2 )

v1 = [1; 0; 0];
v2 = [0; 1; 0];
v3 = cross(v1, v2);

v = [v1 v2 v3]

p0
p1
p2

% Apply forward transformation
% After calibration (get points p0, p1, p2):
w1 = (p1 - p0) / norm(p1 - p0)
temp = (p2 - p0) / norm(p2 - p0)
w3 = cross(w1, temp)
w2 = cross(w3, w1)
% w1 = rotate3D(v1, rotation(1), rotation(2), rotation(3));
% w2 = rotate3D(v2, rotation(1), rotation(2), rotation(3));
% w3 = rotate3D(v3, rotation(1), rotation(2), rotation(3));

w = [w1 w2 w3]

B = v*w'

[U, S, V] = svd(B)

M = diag([1, 1, det(U)*det(V)])

R = U * M * V'

rmat = R;

end

