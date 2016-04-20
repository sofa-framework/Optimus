s=0.01;
r=0.02;
l0=0.0;
l1=0.08;
l2=0.16;

ext=0.08;

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

Point(11) = { 0, 0, l2, s};
Point(12) = { r, 0, l2, s};
Point(13) = { 0, r, l2, s};
Point(14) = {-r, 0, l2, s};
Point(15) = { 0,-r, l2, s};


Circle(1) = {2,1,3};
Circle(2) = {3,1,4};
Circle(3) = {4,1,5};
Circle(4) = {5,1,2};

Circle(5) = {7,6,8};
Circle(6) = {8,6,9};
Circle(7) = {9,6,10};
Circle(8) = {10,6,7};

Circle(9) = {12,11,13};
Circle(10) = {13,11,14};
Circle(11) = {14,11,15};
Circle(12) = {15,11,12};


Line Loop(13) = {1,2,3,4};
Line Loop(14) = {5,6,7,8};
Line Loop(15) = {9,10,11,12};

Plane Surface(6) = {13};
Plane Surface(7) = {14};
Plane Surface(8) = {15};

Extrude {0,0,ext} {
  Surface{6};
}

Extrude {0,0,ext} {
  Surface{7};
}

Extrude {0,0,ext} {
  Surface{8};
}
