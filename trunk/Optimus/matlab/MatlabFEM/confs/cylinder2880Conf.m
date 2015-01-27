%root directory where the program is located directories
strcParams.rootDir=[ pwd '/..'];

%main identificator of the input data, determines which mesh will be used 
strcParams.bodyID='cylinder2880';

%main identificator of the output data (determines the name of output directory
strcParams.experimentTag='test_SVC';

%how many steps in quasi static incrementing 
strcParams.numLoadSteps=1;

%external tools 
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
%strcParams.bcFixedFile='fixed42.bc';
strcParams.bcFixedFile='X0_nodes.mat';
strcParams.bcDisplacedFile='displaced1.bc';   

%if the coordinates of the mesh nodes are to be changed ([m] vs. [mm] etc.)
strcParams.convertCoeff = 1.0;

%material properties for StVenant-Kirchhoff
strcParams.typeMaterial='SV'; 
E=61000;
nu=0.49;
lambda=(E*nu)/((1+nu)*(1-2*nu));
mu=E/(2+2*nu);
strcParams.SVelastCoeff=[lambda mu];
strcParams.isIncompressible=0;    

%material properties for Mooney-Rivlin
%strcParams.typeMaterial='MR'; 
%strcParams.MRelastCoeff=[0.0013011 0.0012766];    

%incompressibilty (necessary for MR, optional for SV)
strcParams.isIncompressible=0;    

%solver stopping criterium
strcParams.stopCriterium=10e-10;

%saving data, 4 == save all
strcParams.saveData=4;  

%gravity:  G=m*g!!!
strcParams.density = 1000;   %[kg/m^3]
strcParams.gravity = [0 -9.81 0];  %[m/s^2]
strcParams.volumeForces=[0 0 0];

%do and save profiling
strcParams.doProfile = 0;
