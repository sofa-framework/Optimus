%root directory where the program is located directories
strcParams.rootDir=[ pwd '/..'];

%main identificator of the input data, determines which mesh will be used 
strcParams.bodyID='cylinder7054deformed1';

%main identificator of the output data (determines the name of output directory
strcParams.experimentTag='test_SVC_inverse';

%how many steps in quasi static incrementing 
strcParams.numLoadSteps=1;

%external tools 
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
%strcParams.bcFixedFile='fixed42.bc';
strcParams.bcFixedFile='X0_nodes.mat';
strcParams.bcDisplacedFile=''; %displaced1.bc';   

%if the coordinates of the mesh nodes are to be changed ([m] vs. [mm] etc.)
strcParams.convertCoeff = 1.0;

%material properties for StVenant-Kirchhoff
strcParams.typeMaterial='SV'; 
E=61000;
nu=0.49;
lambda=(E*nu)/((1+nu)*(1-2*nu));
mu=E/(2+2*nu);
strcParams.SVelastCoeff=[lambda mu];

%material properties for Mooney-Rivlin
%strcParams.typeMaterial='MR'; 
%strcParams.MRelastCoeff=[0.0013011 0.0012766];    

%incompressibilty (necessary 1 for MR, optional [0/1] for SV)
strcParams.isIncompressible=0;   

%solver stopping criterium
strcParams.newtonStopCriterium=10e-10;
strcParams.newtonMaxIt=25;

%solution method
strcParams.quasiNewton=1;
strcParams.perturbation=0.0000001;

%saving data, 4 == save all
strcParams.saveData=4;  

%gravity:  G=m*g!!!
strcParams.density = 1000;   %[kg/m^3]
strcParams.gravity = [0 -9.81 0];  %[m/s^2]
strcParams.volumeForces=[0 0 0];

%do and save profiling
strcParams.doProfile = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%visualization parameters:
%visParams.truthNodeFile='truthCylinder/sofaGravityNodes.mat';
%visParams.truthFaceFile='truthCylinder/sofaGravityFaces.mat';

visParams.truthNodeFile='cylinder7054/nodes.mat';
visParams.truthFaceFile='cylinder7054/faces.mat'; 

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
