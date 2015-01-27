function [strcParams, strcModel, strcFEM, strcIncompress, strcBVP, strcSolution] = setupProblem(experimentConf)
    %read the configuration file    
    addpath('../confs');
    eval(sprintf('%s', experimentConf));  

    strcParams = setupParameters(strcParams);                        
    strcModel = setupModel(strcParams);    
    strcFEM = setupFEM(strcModel);
    strcIncompress = setupIncompressibility(strcModel, strcParams.isIncompressible);    
    strcBVP = setupBVP(strcModel, strcFEM, strcIncompress);
    strcSolution = setupSolution(strcParams);
    
    %copy the configuration file to output directory so the configuration
    %used is available with the corresponding output data                                   
    eval(sprintf('!cp %s/confs/%s.m %s/expConf.m', strcParams.rootDir, experimentConf, strcParams.outDir));
    if (strcParams.saveData > 2)
        save(sprintf('%s/model_data.mat',strcParams.outDir), 'strcParams','strcModel');
    end
end

function strcParams = setupParameters(strcParams)
    strcParams.meshDir=sprintf('%s/input_mesh/%s', strcParams.rootDir, strcParams.bodyID);
    strcParams.srcDir=sprintf('%s/scripts', strcParams.rootDir);
    strcParams.workDir=sprintf('%s/workdir/%s', strcParams.rootDir, strcParams.bodyID);
    
    if (strcParams.isIncompressible == 0)
        strInc='C';
    else
        strInc='I'
    end        
    
    strcParams.experimentTag = [strcParams.typeMaterial strInc '_' strcParams.solutionMethod '_' strcParams.lagrangian '_' strcParams.problemType strcParams.experimentTag];
    strcParams.outDir=sprintf('%s/%s', strcParams.workDir,strcParams.experimentTag);
    %fprintf('Storing results to %s\n', strcParams.outDir);
    
    if (~isfield(strcParams, 'nodeFile'))
        strcParams.nodeFile = 'nodes.mat';
    end
                        
    strcParams.nodesFile=sprintf('%s/%s',strcParams.meshDir, strcParams.nodeFile);    
    strcParams.elementsFile=sprintf('%s/elems.mat',strcParams.meshDir);
    strcParams.facesFile=sprintf('%s/faces.mat',strcParams.meshDir);
    strcParams.fixedNodesFile=sprintf('%s/%s',strcParams.meshDir,strcParams.bcFixedFile);
    
    if (strcParams.bcDisplacedFile)
        strcParams.displacedNodesFile=sprintf('%s/%s',strcParams.meshDir,strcParams.bcDisplacedFile);    
    else
        strcParams.displacedNodesFile='';
    end            
    
    addpath(sprintf('%s', strcParams.pathToTetraTBX));
    addpath(sprintf('%s', strcParams.srcDir));
    
    if (exist(strcParams.workDir) ~= 7)
        eval(sprintf('!mkdir -p %s', strcParams.workDir));        
    end;
    cd(strcParams.workDir);    
    
    if (exist(strcParams.outDir) ~= 7)
        eval(sprintf('!mkdir -p %s', strcParams.outDir));        
    else
        eval(sprintf('!mkdir %s/arch_`date +%%y%%m%%d_%%k%%M`', strcParams.outDir))
        eval(sprintf('!mv %s/*.mat %s/arch_`date +%%y%%m%%d_%%k%%M`', strcParams.outDir,strcParams.outDir)); 
    end;    
    eval(sprintf('!cp %s %s', strcParams.fixedNodesFile, strcParams.outDir));    
    if (exist(strcParams.displacedNodesFile,'file'))
        eval(sprintf('!cp %s %s', strcParams.displacedNodesFile, strcParams.outDir));    
    end
end

function strcModel = setupModel(strcParams)
    %check the material type and coefficients
    strcModel.typeMaterial = strcParams.typeMaterial;
    if (strcModel.typeMaterial == 'SV')
        strcModel.elastCoeff=strcParams.SVelastCoeff*strcParams.convertCoeff^2;
        fprintf('StVenant-Kirchhoff material, lam=%f, mu=%f\n', strcModel.elastCoeff(1), strcModel.elastCoeff(2));
        %disp('Lambda,mu:');
        %disp(strcModel.elastCoeff);
    elseif (strcModel.typeMaterial == 'MR')
        strcModel.elastCoeff=strcParams.MRelastCoeff*strcParams.convertCoeff^2;
        disp('Mooney-Rivlin material');
        disp('C[01],C[10]:');
        disp(strcModel.elastCoeff);
    else
        disp('Unknown material');
        return;
    end    
    %read elements and geometry
    strcModel.matElems=load(strcParams.elementsFile,'-ASCII');
    strcModel.matCoors=load(strcParams.nodesFile,'-ASCII');
    strcModel.matFaces=load(strcParams.facesFile,'-ASCII');
    
    strcModel.matCoors = strcModel.matCoors./strcParams.convertCoeff;
    
    fixedBCs=load(strcParams.fixedNodesFile,'-ASCII');
    strcModel.vecFixedNodes=fixedBCs(:,1);
        
    if (exist(strcParams.displacedNodesFile))
        displBCs=load(strcParams.displacedNodesFile,'-ASCII');
        strcModel.givenPrescribed = 1;
        strcModel.vecDisplNodes=displBCs(:,1);
        strcModel.prescribedDispls=displBCs(:,2:size(displBCs,2))./strcParams.convertCoeff;
    else
        strcModel.givenPrescribed = 0;
    end

    %number of elems, nodes and DOFs and BCs
    strcModel.nElems=size(strcModel.matElems,1);
    strcModel.nNodes=size(strcModel.matCoors,1);   
    
    %store the undeformed coordinates
    strcModel.matCoorsRest=strcModel.matCoors;
    
    %number of degrees of freedom  (3D problem => 3*#nodes
    strcModel.nDOFs=strcModel.nNodes*3;
    
    %forces in vertices (surface tractions)   OBSOLETE
    strcModel.surfaceTractions=zeros(strcModel.nDOFs,1);
    %forces in elements (volume forces) 
  
    strcModel.volumeForces =  repmat(strcParams.volumeForces, strcModel.nElems,1);
    
    if (isfield(strcParams,'directLoads'))
        strcModel.directLoads = repmat(strcParams.directLoads', strcModel.nNodes, 1);
    else
        strcModel.directLoads = zeros(strcModel.nDOFs,1);
    end
    
    strcModel.initialVertDisp = zeros(strcModel.nDOFs, 1);
    
    strcModel.gravity=reshape(strcParams.gravity,3,1);
    strcModel.density=strcParams.density;
    
    if (isfield(strcParams,'massLumping'))
        strcModel.massLumping=strcParams.massLumping;
        
        if (strcModel.massLumping)
            if (isfield(strcParams,'totalMass'))
                strcModel.totalMass=strcParams.totalMass;
                strcModel.givenTotalMass=1;
            else
                strcModel.givenTotalMass=0;
            end
        end
    else
        strcModel.massLumping=0;
    end
    
    if (isfield(strcParams,'problemType'))        
        if (strcmp(strcParams.problemType, 'forward'))
            strcModel.inverseProblem=0;
        elseif (strcmp(strcParams.problemType,'inverse'))
            strcModel.inverseProblem=1;
        else            
            fprintf('ERROR: problem type %s not known...\n', strcModel.problemType);
        end
    else
        strcModel.inverseProblem=0;
    end
    
end

function strcSolution = setupSolution(strcParams)
    strcSolution.newtonStopCriterium = strcParams.newtonStopCriterium;
    strcSolution.newtonMaxIt = strcParams.newtonMaxIt;

    if (isfield(strcParams,'lagrangian'))
        if (strcmp(strcParams.lagrangian,'total') || strcmp(strcParams.lagrangian,'updated') )
            strcSolution.lagrangian = strcParams.lagrangian;
        else
            fprintf('Unknown lagrangian method %s, should be updated or total\n', strcParams.lagrangian);
        end
    else
        fprintf('No lagrangian specified, using total\n'); 
        strcSolution.lagrangian = 'total'
        
    end
    
    if (isfield(strcParams,'solutionMethod'))
        strcSolution.solutionMethod = strcParams.solutionMethod;
    else        
        fprintf('No solution method specified, using full\n'); 
        strcSolution.solutionMethod = 'full';
    end

    if (strcmp(strcSolution.solutionMethod,'quasi'))
        if (isfield(strcParams,'perturbation'))
            strcSolution.perturbation = strcParams.perturbation;
        else
            fprintf('Perturbation for quasi-Newton not set! Set to default value of 0.001.\n');
            strcSolution.quasiNewton = 0.001;            
        end        
    end
    
    strcSolution.numIncSteps = strcParams.numIncSteps;
end

function strcBVP = setupBVP(strcModel, strcFEM, strcIncompress)
    %fixed nodes and DOFs      
    strcBVP.nFixedNodes=size(strcModel.vecFixedNodes,1);
    strcBVP.nFixedDOFs=3*strcBVP.nFixedNodes;
    strcBVP.vecFixedstrcModel.matCoors=zeros(1,strcBVP.nFixedDOFs);
    strcBVP.vecFixedLMCoors=zeros(1,strcBVP.nFixedDOFs);
    for node=1:strcBVP.nFixedNodes
        strcBVP.vecFixedstrcModel.matCoors(3*(node-1)+1:3*node) = 3*(strcModel.vecFixedNodes(node)-1)+1:3*strcModel.vecFixedNodes(node);
        strcBVP.vecFixedLMCoors(3*(node-1)+1:3*node) = strcModel.nDOFs+strcIncompress.size+(3*(node-1)+1:3*node);
    end
    strcBVP.valsFixedDOFs=zeros(size(strcBVP.vecFixedstrcModel.matCoors));
    %disp('Fixed DOFs:');
    %disp(strcBVP.vecFixedstrcModel.matCoors);
    
    %displaced nodes    
    if (strcModel.givenPrescribed)
        strcBVP.nDisplNodes=size(strcModel.vecDisplNodes,1);
        strcBVP.nDisplDOFs=3*strcBVP.nDisplNodes;
        strcBVP.vecDisplstrcModel.matCoors=zeros(1,strcBVP.nDisplDOFs);
        strcBVP.vecDisplLMCoors=zeros(1,strcBVP.nDisplDOFs);
        for node=1:strcBVP.nDisplNodes
            strcBVP.vecDisplstrcModel.matCoors(3*(node-1)+1:3*node) = 3*(strcModel.vecDisplNodes(node)-1)+1:3*strcModel.vecDisplNodes(node);
            strcBVP.vecDisplLMCoors(3*(node-1)+1:3*node) = strcModel.nDOFs+strcIncompress.size+strcBVP.nFixedDOFs+(3*(node-1)+1:3*node);
        end     
        strcBVP.valsDisplacedDOFs=zeros(size(strcBVP.vecDisplstrcModel.matCoors));     
 
        %disp('Displaced DOFs:');
        %disp(strcBVP.vecDisplstrcModel.matCoors);
    else
        strcBVP.nDisplNodes=0;
        strcBVP.nDisplDOFs=0;
    end
    
    strcBVP.size = strcBVP.nFixedDOFs + strcBVP.nDisplDOFs;   
    
    strcBVP.nSystemSize = strcFEM.size + strcIncompress.size + strcBVP.size;
        
    strcBVP.forcesInFixed = zeros(strcBVP.nFixedDOFs,1);
    strcBVP.forcesInDispl = zeros(strcBVP.nDisplDOFs,1);
        
    %structure for displacements in vertices
    strcBVP.vertDisp=zeros(strcFEM.size,1); 
end

function strcIncompress = setupIncompressibility(strcModel, isIncompressible)
    strcIncompress.isIncompress = isIncompressible;

    %pressure LM for the incompressibility
    if (strcIncompress.isIncompress)
        strcIncompress.size = strcModel.nNodes;
        strcIncompress.vertPressure = zeros(strcModel.nNodes,1);
        strcIncompress.vecPressureLMCoors = strcModel.nDOFs+1:strcModel.nDOFs+strcModel.nNodes;
    else
        strcIncompress.size = 0;
        strcIncompress.vertPressure = zeros(strcModel.nNodes,1);
        strcIncompress.vecPressureLMCoors = 0;
    end
end

function strcFEM = setupFEM(strcModel)
    strcFEM.size = strcModel.nDOFs;
    strcFEM.qSize = 64;
    strcFEM.vertMatrices=zeros(strcModel.nElems, 4, 4);
    strcFEM.invVertMatrices=zeros(strcModel.nElems, 4, 4);
    strcFEM.gaussQuad=zeros(strcModel.nElems, strcFEM.qSize, 5);
    strcFEM.shapeFunctions=zeros(strcModel.nElems, strcFEM.qSize, 4);
    strcFEM.intShapeFunctions=zeros(strcModel.nElems, 4);      %only for incompressibility term    

    %for each element, precompute:
    for el=1:strcModel.nElems
        %vertex matrices, volumes and inverted vertex matrices
        vert = [ strcModel.matCoorsRest(strcModel.matElems(el,1),:); strcModel.matCoorsRest(strcModel.matElems(el,2),:); strcModel.matCoorsRest(strcModel.matElems(el,3),:); strcModel.matCoorsRest(strcModel.matElems(el,4),:) ];
        vertMat = [ 1 strcModel.matCoorsRest(strcModel.matElems(el,1),:); 1 strcModel.matCoorsRest(strcModel.matElems(el,2),:); 1 strcModel.matCoorsRest(strcModel.matElems(el,3),:); 1 strcModel.matCoorsRest(strcModel.matElems(el,4),:) ];        
        invVertMat=inv(vertMat);
        
        strcFEM.elemVols(el)=1/6*abs(det(vertMat));
        strcFEM.vertMatrices(el,:,:)=vertMat(:,:);
        strcFEM.invVertMatrices(el, :, :)=invVertMat(:,:);
               
        %quadrature points
        [X(:,2),X(:,3),X(:,4),W]=tetraquad(4,vert);        
        strcFEM.qSize = size(X(:,1),1);
        X(:,1) = ones(strcFEM.qSize,1);
        
        %shape functions using the quadrature points
        strcFEM.shapeFunctions(el,:,:) = computeShapeFunctions(invVertMat, X);
        for sm=1:4   %only for incompressibility term    
            strcFEM.intShapeFunctions(el,sm)=dot(strcFEM.shapeFunctions(el,:,sm), W);
        end        
        strcFEM.gaussQuad(el,:,1:4)=X(:,1:4);
        strcFEM.gaussQuad(el,:,5)=W(:,1);       
    end
    %save('strcFEM.mat','strcFEM');
    strcFEM.totalVolume=sum(strcFEM.elemVols(:));
end

function [phi] = computeShapeFunctions(invVert, X)
    %size(X);
    for i = 1:4 
        phi(:,i) = invVert(1,i).*X(:,1) + invVert(2,i).*X(:,2) + invVert(3,i).*X(:,3) + invVert(4,i).*X(:,4);        
    end
    %size(phi) 
end