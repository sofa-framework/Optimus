s=0.1;
r=0.02;
l0=0.0;
l1=0.3;
l2=0.1;

Point(1) = { 0, 0, l0, s};
Point(2) = { r, 0, l0, s};
Point(3) = { 0, r, l0, s};
Point(4) = {-r, 0, l0, s};
Point(5) = { 0,-r, l0, s};

Circle(1) = {2,1,3};
Circle(2) = {3,1,4};
Circle(3) = {4,1,5};
Circle(4) = {5,1,2};

Line Loop(5) = {1,2,3,4};
Plane Surface(6) = {5};

Extrude {0,0,l1} {
  Surface{6};
}
