s=0.01;
r=0.02;
l0=0.0;
l1=0.15;
l2=0.15;

Point(1) = { 0, 0, l0, s};
Point(2) = { r, 0, l0, s};
Point(3) = { 0, r, l0, s};
Point(4) = {-r, 0, l0, s};
Point(5) = { 0,-r, l0, s};

Point(6) = { 0, 0, l1, s};
Point(7) = { r, 0, l1, s};
Point(8) = { 0, r, l1, s};
Point(9) = {-r, 0, l1, s};
Point(10) = { 0,-r, l1, s};

Circle(1) = {2,1,3};
Circle(2) = {3,1,4};
Circle(3) = {4,1,5};
Circle(4) = {5,1,2};

Circle(5) = {7,6,8};
Circle(6) = {8,6,9};
Circle(7) = {9,6,10};
Circle(8) = {10,6,7};

Line Loop(9) = {1,2,3,4};
Line Loop(10) = {5,6,7,8};
Plane Surface(6) = {9};
Plane Surface(7) = {10};

Extrude {0,0,l1} {
  Surface{6};
}

Extrude {0,0,l2} {
  Surface{7};
}
