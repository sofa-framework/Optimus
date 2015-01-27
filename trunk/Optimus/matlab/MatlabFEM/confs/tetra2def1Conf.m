%root directory where the program is located directories
strcParams.rootDir=[ pwd '/..'];

%main identificator of the input data, determines which mesh will be used 
strcParams.bodyID='tetra2deformed1';

%main identificator of the output data (determines the name of output directory
strcParams.experimentTag='test_SVCinv';

%how many steps in quasi static incrementing 
strcParams.numLoadSteps=1;

%external tools 
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
strcParams.bcFixedFile='fixed3.bc';
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
strcParams.newtonMaxIt=100;

%solution method
strcParams.quasiNewton=1;
strcParams.perturbation=0.0001;

%saving data, 4 == save all
strcParams.saveData=4;

%gravity:  G=m*g!!!
strcParams.density = 5000;   %[kg/m^3]
strcParams.gravity = [0 -9.81 0];  %[m/s^2]
strcParams.volumeForces=[0 0 0];

%do and save profiling
strcParams.doProfile = 0;

visParams.truthOrgan = 'tetra2';
