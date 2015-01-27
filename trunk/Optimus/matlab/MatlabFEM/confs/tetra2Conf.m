gconst=1;

strcParams.bodyID='twoTetraSofa';
strcParams.problemType='forward';
strcParams.lagrangian='total';
strcParams.solutionMethod = 'full';
strcParams.experimentTag='_test';

%strcParams.bodyID='tetra2_15G';
%strcParams.problemType='inverse';
%strcParams.lagrangian='updated';
%strcParams.solutionMethod = 'quasi';
%strcParams.experimentTag='_inc1';

%how many steps in quasi static incrementing 
strcParams.numIncSteps=1;

%dirs
strcParams.rootDir=[ pwd '/..'];
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
strcParams.bcFixedFile='fixed1.bc';
strcParams.bcDisplacedFile='';%'displaced1.bc';

%if the coordinates of the mesh nodes are to be changed ([m] vs. [mm] etc.)
strcParams.convertCoeff = 1.0;

%material properties for StVenant-Kirchhoff
strcParams.typeMaterial='SV'; 
E=1e6;
nu=0.45;
lambda=(E*nu)/((1+nu)*(1-2*nu));
mu=E/(2+2*nu);
strcParams.SVelastCoeff=[lambda mu];

%material properties for Mooney-Rivlin
%strcParams.typeMaterial='MR'; 
%strcParams.MRelastCoeff=[0.0013011 0.0012766];    
%incompressibilty (necessary 1 for MR, optional [0/1] for SV)
strcParams.isIncompressible=0;   


strcParams.quadOrder=1;
%solver stopping criterium
strcParams.newtonStopCriterium=10e-10;
strcParams.newtonMaxIt=2;

%problem type  forward/inverse


%Lagrangian method  total/updated

strcParams.massLumping=0;
strcParams.totalMass=0.027;

%solution method quasi/full

strcParams.perturbation=0.000000001;

%saving data, 4 == save all
strcParams.saveData=3;  

%gravity:  G=m*g!!!
strcParams.density = 5000;   %[kg/m^3]
strcParams.gravity = gconst*[0 0 0];  %[m/s^2]
strcParams.volumeForces=[0 0 0];

%do and save profiling
strcParams.doProfile = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%visualization parameters:
%visParams.truthNodeFile='truthCylinder/sofaGravityNodes.mat';
%visParams.truthFaceFile='truthCylinder/sofaGravityFaces.mat';

%visParams.truthNodeFile='tetra2/nodes.mat';
%visParams.truthFaceFile='tetra2/faces.mat'; 

visParams.showDeformed = 1;

%visualize forces
visParams.showForcePlots=1;
visParams.showForceVector=1;
visParams.forceVectorColor='magenta';
visParams.forceVectorScale=1;

%visualize points 
visParams.showFixedPoints=1;
visParams.showDisplacedPoints=1;
visParams.pointSize=10;
visParams.fixedPointColor='yellow';
visParams.presPointRestColor='green';
visParams.presPointDispColor='red';

%organ visualization
visParams.faceColRest='green';
visParams.edgeColRest='green';
visParams.faceColDef='red';
visParams.edgeColDef='red';
visParams.faceAlfa=0.3;
visParams.frameWidthRel=0.25;

visParams.liverAxesLimits=[0 0 0 0 0 0];
visParams.exchangeYZ=1;
