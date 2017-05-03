p0 = [0; 0; 0];
p1 = [10; 0; 0];
p2 = [0; 10; 0];

rotation = [pi/4, pi/4, pi/4];

p0_gls = rotate3D(p0, rotation);
p1_gls = rotate3D(p1, rotation);
p2_gls = rotate3D(p2, rotation);

rmat = Project(p0_gls, p1_gls, p2_gls);

ptest = [5; 5; 1];
ptest_gls = rotate3D(ptest, rotation);

ptest_projected = rmat * ptest_gls