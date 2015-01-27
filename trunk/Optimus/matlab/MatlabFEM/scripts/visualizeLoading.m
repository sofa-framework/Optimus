%argument determines the path to the computed data in the $ROOT/workdir/
function [ret] = visualizeLoading(object, app, exp, loadSteps)

close all

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

%visParams.truthObject='~/Work/MUNI/Choral2/tetra2/input';
%visParams.truthObject='~/Work/Choral2/cylinder152_1G/input';
%visParams.truthObject='~/Work/MUNI/Choral2/cylinder22931_1G/input';
%visParams.truthObject ='~/Work/MUNI/Choral2/sofa_cylinder_deformed';

visParams.exchangeYZ=1;

addpath('../confs');

rootDir='/home/ip/Work/MatlabScripts/MatlabFEM';

a='-ASCII';
if (strcmp(app,'M'))
    inDir=sprintf('%s/input_mesh/%s/', rootDir, object);
    load(sprintf('%s/faces.mat',inDir),a);
    load(sprintf('%s/nodes.mat',inDir),a);
    disp(sprintf('%s/nodes.mat',inDir));
    load(sprintf('%s/X0_nodes.mat',inDir),a);
    nodes_fixed=X0_nodes;
    expDir=sprintf('%s/workdir/%s/%s', rootDir, object, exp);
    %expDir2=sprintf('/home/ip/Work/MUNI/MatlabElasticity/workdir/%s/%s', object, exp2);
elseif (strcmp(app,'CH'))
    inDir=sprintf('/home/ip/Work/Choral2/%s/', object);
    load(sprintf('%s/input/faces.mat',inDir),a);
    load(sprintf('%s/input/nodes.mat',inDir),a);
    load(sprintf('%s/input/nodes_fixed.mat',inDir),a);
    expDir=sprintf('%s/%s', inDir, exp);
    %expDir2=sprintf('%s/%s', inDir, exp2);
end

if (visParams.exchangeYZ)
    temp=nodes;
    nodes(:,2)=temp(:,3);
    nodes(:,3)=temp(:,2);
end

for i=1:3
    minima(i)=min(nodes(:,i));
    maxima(i)=max(nodes(:,i));
end

prescFile='';
if (exist(prescFile))
    load(prescFile,a);
    nds=nodes_presc;
    nodes_presc_ix=int16(nds(:,1));
    nodes_presc_val=nds(:,2:4);
else
    visParams.showDisplacedPoints=false;
end

for step=1:loadSteps
    pos=load(sprintf('%s/positions_inc%03d.mat', expDir,step), a);
    positions(step).X = pos;
    if (visParams.exchangeYZ)
        temp=positions(step).X;
        positions(step).X(:,2)=temp(:,3);
        positions(step).X(:,3)=temp(:,2);
        
        for i=1:3
            lminima(i)=min(positions(step).X(:,i));
            if (lminima(i) < minima(i))
                minima(i) = lminima(i);
            end
            lmaxima(i)=max(positions(step).X(:,i));
            if (lmaxima(i) > maxima(i))
                maxima(i) = lmaxima(i);
            end
        end
    end
end

%load truth object to compare
if (isfield(visParams,'truthObject'))
    truthNodeFile = sprintf('%s/nodes_truth.mat', visParams.truthObject);
    truthFaceFile = sprintf('%s/faces_truth.mat',visParams.truthObject);
    if (exist(truthNodeFile,'file') & exist(truthFaceFile,'file'))
        truthNodes = load(truthNodeFile,'-ASCII');
        truthFaces = load(truthFaceFile,'-ASCII');
        showTruth = 1;
        %truthFaces=load(sprintf('../input_mesh/%s/faces.mat',visParams.truthOrgan),'-ASCII');
        temp=truthNodes;
        truthNodes(:,2)=temp(:,3);
        truthNodes(:,3)=temp(:,2);
        
        for i=1:3
            lminima(i)=min(truthNodes(:,i));
            if (lminima(i) < minima(i))
                minima(i) = lminima(i);
            end
            lmaxima(i)=max(truthNodes(:,i));
            if (lmaxima(i) > maxima(i))
                maxima(i) = lmaxima(i);
            end
        end
        
    else
        showTruth = 0;
        truthNodes = [];
        fprintf('Truth files %s or %s does not exist, not comparing!\n', truthNodeFile, truthFaceFile);
    end
else
    showTruth = 0;
    truthNodes = [];
end

frameX=visParams.frameWidthRel*(maxima(1)-minima(1));
frameY=visParams.frameWidthRel*(maxima(2)-minima(2));
frameZ=visParams.frameWidthRel*(maxima(3)-minima(3));

axisExt = [minima(1) maxima(1) minima(2) maxima(2) minima(3) maxima(3)] + [-frameX frameX -frameY frameY -frameZ frameZ];

figure('Position', [10 10 1200 800],'DoubleBuffer','on',...
    'Renderer','OpenGL',...
    'Color',[1 1 1],...
    'InvertHardcopy','off',...
    'Name','Force-displacement Curves',...
    'XVisual','0x23 (TrueColor, depth 32, RGB mask 0xff0000 0xff00 0x00ff)');
% figLiver=figure(1);
% set(figLiver,'Renderer','OpenGL');
% set(figLiver,'Position', [0 0 850 850]);
% set(figLiver,'Color',[1 1 1]);
% set(figLiver,'InvertHardcopy','off');
%
% figPlot=figure(2);
% axes('FontSize',14,'FontWeight','Bold'); %,'Parent',figPlot);
% axis(plotAxesLimits)
% set(figPlot,'Position', [900 0 450 450]);
% set(figPlot,'Color',[1 1 1],'InvertHardcopy','off',...
%     'XVisual','0x23 (TrueColor, depth 32, RGB mask 0xff0000 0xff00 0x00ff)');
%axes('FontSize',14,'FontWeight','Bold','Parent',figPlot);

%az = 52;
%el = -20;
%view(29, 34);
grid on

if (visParams.showFixedPoints)
    fixedPoints=nodes(nodes_fixed,:);
end

if (visParams.showDisplacedPoints)
    presPointsRest=nodes(nodes_presc_ix,:);
    presPointsDisp=deformedCoors1(nodes_presc_ix,:);
end

for loadStep=1:loadSteps
    clf('reset');
    %subplot('Position',[0.05 0.05 0.60 1]); %('Position',[200 200 200 200]);
    set(gca,'DataAspectRatio',[1 1 1]) %, 'Position', [10 10 800 800])
    axis(axisExt);
    hold on
    patch('Vertices', nodes, 'Faces', faces, ...
        'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColRest, 'EdgeColor', visParams.edgeColRest,'FaceAlpha',0.1);
    
    patch('Vertices', positions(loadStep).X, 'Faces', faces, ...
        'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColDef, 'EdgeColor', visParams.edgeColDef,'FaceAlpha',0.1);
    
    
    if (showTruth)
        patch('Vertices', truthNodes, 'Faces', truthFaces, ...
                'FaceVertexCData',hsv(8),'FaceColor','b', 'EdgeColor', 'b','FaceAlpha',0.1);
    end
    
    if (visParams.showFixedPoints)
        stem3(fixedPoints(:,1),fixedPoints(:,2),fixedPoints(:,3), ...
                'LineStyle','none','MarkerSize',visParams.pointSize,'MarkerFaceColor',visParams.fixedPointColor);
    end
    if (visParams.showDisplacedPoints)
        stem3(presPointsRest(:,1),presPointsRest(:,2),presPointsRest(:,3), ...
                'LineStyle','none','MarkerSize',visParams.pointSize,'MarkerFaceColor',visParams.presPointRestColor);
        stem3(presPointsDisp(:,1),presPointsDisp(:,2),presPointsDisp(:,3), ...
                'LineStyle','none','MarkerSize',visParams.pointSize,'MarkerFaceColor',visParams.presPointDispColor);
    end

    grid on
    drawnow
    if (loadStep == 1)
        disp('press any key')
        pause
    else
        pause(0.2);
    end
    
end
end
