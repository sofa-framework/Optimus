lc = 0.013;

//cube
x=0.1;
y=0.1;
z=0.1;

Point(1) = {x/2, -y/2, -z/2, lc};
Point(2) = {-x/2, -y/2, -z/2, lc};
Point(3) = {-x/2, y/2, -z/2, lc};
Point(4) = {x/2, y/2, -z/2, lc};
Point(5) = {x/2, y/2, z/2, lc};
Point(6) = {-x/2, -y/2, z/2, lc};
Point(7) = {x/2, -y/2, z/2, lc};
Point(8) = {-x/2, y/2, z/2, lc};

Line (1) = {4, 3};
Line (2) = {3, 2};
Line (3) = {2, 1};
Line (4) = {1, 4};
Line (5) = {4, 5};
Line (6) = {5, 8};
Line (7) = {8, 3};
Line (8) = {6, 8};
Line (9) = {5, 7};
Line (10) = {7, 6};
Line (11) = {6, 2};
Line (12) = {1, 7};

Line Loop (13) = {1,2,3,4};
Plane Surface(14) = {13};
Line Loop (15) = {6,-8,-10,-9};
Plane Surface(16) = {15};
Line Loop (17) = {3,11,10,12};
Plane Surface(18) = {17};
Line Loop (19) = {1,-7,-6,-5};
Plane Surface(20) = {19};
Line Loop (21) = {-2,-7,-8,11};
Plane Surface(22) = {21};
Line Loop (23) = {4,5,9,-12};
Plane Surface(24) = {23};

Surface Loop(30) = {14, 16, 18, 20, 22, 24};


/// sphere 1
lc1 = .01;

cx1=0.0;
cy1=0.0;
cz1=0.0;
r1=0.03;

Point(100) = {cx1,cy1,cz1, lc1};
Point(200) = {cx1+r1,cy1,cz1,lc1};
Point(300) = {cx1,cy1+r1,cz1,lc1};
Point(400) = {cx1-r1,cy1,cz1,lc1};
Point(500) = {cx1,cy1-r1,cz1,lc1};
Point(600) = {cx1,cy1,cz1-r1,lc1};
Point(700) = {cx1,cy1,cz1+r1,lc1};

Circle(100) = {200,100,300};
Circle(200) = {300,100,400};
Circle(300) = {400,100,500};
Circle(400) = {500,100,200};
Circle(500) = {300,100,600};
Circle(600) = {600,100,500};
Circle(700) = {500,100,700};
Circle(800) = {700,100,300};
Circle(900) = {200,100,700};
Circle(1000) = {700,100,400};
Circle(1100) = {400,100,600};
Circle(1200) = {600,100,200};
Line Loop(1300) = {200,800,-1000};
Ruled Surface(1400) = {1300};
Line Loop(1500) = {1000,300,700};
Ruled Surface(1600) = {1500};
Line Loop(1700) = {-800,-900,100};
Ruled Surface(1800) = {1700};
Line Loop(1900) = {-1100,-200,500};
Ruled Surface(2000) = {1900};
Line Loop(2100) = {-500,-1200,-100};
Ruled Surface(2200) = {2100};
Line Loop(2300) = {-300,1100,600};
Ruled Surface(2400) = {2300};
Line Loop(2500) = {-700,400,900};
Ruled Surface(2600) = {2500};
Line Loop(2700) = {-400,1200,-600};
Ruled Surface(2800) = {2700};
Surface Loop(2900) = {2800,2600,1600,1400,2000,2400,2200,1800};

Volume(1) = {30, 2900};
Volume(2) = {2900};

