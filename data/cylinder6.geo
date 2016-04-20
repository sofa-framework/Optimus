s=0.007;
r=0.02;
l0=0.0;
l1=0.04;
l2=0.08;
l3=0.12;
l4=0.16;
l5=0.20;

ext=0.04;

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

Point(16) = { 0, 0, l3, s};
Point(17) = { r, 0, l3, s};
Point(18) = { 0, r, l3, s};
Point(19) = {-r, 0, l3, s};
Point(20) = { 0,-r, l3, s};

Point(21) = { 0, 0, l4, s};
Point(22) = { r, 0, l4, s};
Point(23) = { 0, r, l4, s};
Point(24) = {-r, 0, l4, s};
Point(25) = { 0,-r, l4, s};

Point(26) = { 0, 0, l5, s};
Point(27) = { r, 0, l5, s};
Point(28) = { 0, r, l5, s};
Point(29) = {-r, 0, l5, s};
Point(30) = { 0,-r, l5, s};


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

Circle(13) = {17,16,18};
Circle(14) = {18,16,19};
Circle(15) = {19,16,20};
Circle(16) = {20,16,17};

Circle(17) = {22,21,23};
Circle(18) = {23,21,24};
Circle(19) = {24,21,25};
Circle(20) = {25,21,22};

Circle(21) = {27,26,28};
Circle(22) = {28,26,29};
Circle(23) = {29,26,30};
Circle(24) = {30,26,27};


Line Loop(13) = {1,2,3,4};
Line Loop(14) = {5,6,7,8};
Line Loop(15) = {9,10,11,12};
Line Loop(16) = {13,14,15,16};
Line Loop(17) = {17,18,19,20};
Line Loop(18) = {21,22,23,24};

Plane Surface(6) = {13};
Plane Surface(7) = {14};
Plane Surface(8) = {15};
Plane Surface(9) = {16};
Plane Surface(10) = {17};
Plane Surface(11) = {18};

Extrude {0,0,ext} {
  Surface{6};
}

Extrude {0,0,ext} {
  Surface{7};
}

Extrude {0,0,ext} {
  Surface{8};
}

Extrude {0,0,ext} {
  Surface{9};
}
Extrude {0,0,ext} {
  Surface{10};
}
Extrude {0,0,ext} {
  Surface{11};
}
