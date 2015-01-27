%root directory where the program is located directories
strcParams.rootDir=[ pwd '/..'];

%main identificator of the input data, determines which mesh will be used 
strcParams.bodyID='cylinder7054';
strcParams.problemType='forward';
strcParams.lagrangian='total';
strcParams.solutionMethod='full';
strcParams.experimentTag='test2014';

strcParams.numIncSteps=5;

%dirs
strcParams.rootDir=[ pwd '/..'];
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
%strcParams.bcFixedFile='fixed42.bc';
strcParams.bcFixedFile='X0_nodes.mat';
strcParams.bcDisplacedFile=''; %displaced1.bc';   

%if the coordinates of the mesh nodes are to be changed ([m] vs. [mm] etc.)
strcParams.convertCoeff = 1.0;

%material properties for StVenant-Kirchhoff
strcParams.typeMaterial='SV'; 
E=1e4;
nu=0.45;
lambda=(E*nu)/((1+nu)*(1-2*nu));
mu=E/(2+2*nu);
strcParams.SVelastCoeff=[lambda mu];

%material properties for Mooney-Rivlin
%strcParams.typeMaterial='MR'; 
%strcParams.MRelastCoeff=[1301.1 1276.6];    

%incompressibilty (necessary 1 for MR, optional [0/1] for SV)
strcParams.isIncompressible=0;   

%solver stopping criterium
strcParams.newtonStopCriterium=10e-10;
strcParams.newtonMaxIt=100;


strcParams.massLumping=0;
strcParams.totalMass=0.216;
strcParams.perturbation=0.000000001;

%saving data, 4 == save all
strcParams.saveData=4;  

%gravity:  G=m*g!!!
strcParams.density = 1000;   %[kg/m^3]
%strcParams.gravity = gconst*[0 -9.81 0];  %[m/s^2]
gconst=1.0;
strcParams.gravity = gconst*[0 0 0];  %[m/s^2]

strcParams.volumeForces=[0 0 0];  %these forces are integrated over element volumes
  
strcParams.directLoads=[0 0.001 0]; %these are added directly to the right-hand side for each node

%do and save profiling
strcParams.doProfile = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%visualization parameters:
%visParams.truthNodeFile='truthCylinder/sofaGravityNodes.mat';
%visParams.truthFaceFile='truthCylinder/sofaGravityFaces.mat';


visParams.truthNodeFile='cylinder152/nodes.mat';
visParams.truthFaceFile='cylinder152/faces.mat'; 

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
visParams.faceColRest='blue';
visParams.edgeColRest='blue';
visParams.faceColDef='red';
visParams.edgeColDef='red';
visParams.faceAlfa=1;
visParams.frameWidthRel=0.25;

visParams.liverAxesLimits=[0 0 0 0 0 0];
visParams.exchangeYZ=1;