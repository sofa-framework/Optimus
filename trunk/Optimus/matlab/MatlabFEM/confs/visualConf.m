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

visParams.truthOrgan='nodes_truth.mat'; %truthCylinder';

visParams.exchangeYZ=1;