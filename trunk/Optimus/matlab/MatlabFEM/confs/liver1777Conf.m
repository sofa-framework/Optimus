%root directory where the program is located directories
strcParams.rootDir=[ pwd '/..'];

%main identificator of the input data, determines which mesh will be used 
strcParams.bodyID='liver1777';

%main identificator of the output data (determines the name of output directory
strcParams.experimentTag='test_SVI';

%how many steps in quasi static incrementing 
strcParams.numLoadSteps=5;

%external tools 
strcParams.pathToTetraTBX=sprintf('%s/externs/Tetraquad', strcParams.rootDir);

%boundary conditions files
strcParams.bcFixedFile='fixed11.bc';
strcParams.bcDisplacedFile='displaced1_13.bc';   

%if the coordinates of the mesh nodes are to be changed ([m] vs. [mm] etc.)
strcParams.convertCoeff = 1.0;

%material properties
strcParams.typeMaterial='SV'; 
strcParams.SVelastCoeff=[0.25431 0.0051899];   %FOR StVenant
strcParams.MRelastCoeff=[0.0013011 0.0012766];    
strcParams.isIncompressible=1;    

%solver stopping criterium
strcParams.newtonStopCriterium=10e-10;
strcParams.newtonMaxIt=25;

%solution method
strcParams.quasiNewton=0;
strcParams.perturbation=0.0000001;

%saving data, 4 == save all
strcParams.saveData=4;  

%gravity?
strcParams.volumeForces=[0 0 0];
strcParams.gravity=[0 0 0];
strcParams.density=1000;

%do and save profiling
strcParams.doProfile = 0;
