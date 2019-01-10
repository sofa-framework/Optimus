lc1 = 0.009;

// rectangle

pxl=0.0;
pxh=0.03;
pyl=0.0;
pyh=0.03;
pzl=0;
length=0.2;

Point(1) = {pxl,pyl,pzl,lc1};
Point(2) = {pxh,pyl,pzl,lc1};
Point(3) = {pxh,pyh,pzl,lc1};
Point(4) = {pxl,pyh,pzl,lc1};
Line(101) = {4,3};
Line(102) = {3,2};
Line(103) = {2,1};
Line(104) = {1,4};
Line Loop(105) = {102,103,104,101};

Plane Surface(106) = {105};

Extrude {0,0,length} {
	Surface{106};
}	

