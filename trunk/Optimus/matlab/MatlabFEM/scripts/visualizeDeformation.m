%argument determines the path to the computed data in the $ROOT/workdir/
function visualizeDeformation(object, app, exp1, exp2)
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
    visParams.faceColDef1='red';
    visParams.edgeColDef1='red';    
    visParams.faceColDef2='cyan';
    visParams.edgeColDef2='cyan';
    visParams.faceAlfa=0.3;
    visParams.frameWidthRel=0.25;

    visParams.liverAxesLimits=[0 0 0 0 0 0];

    %visParams.truthObject='~/Work/MUNI/Choral2/tetra2/input';
    %visParams.truthObject ='~/Work/MUNI/Choral2/sofa_cylinder_deformed';

    visParams.exchangeYZ=1;


    %===========================================================

    addpath('../confs');
    
    a='-ASCII';
    if (strcmp(app,'M'))
        inDir=sprintf('../input_mesh/%s/', object);
        load(sprintf('%s/faces.mat',inDir),a);
        load(sprintf('%s/nodes.mat',inDir),a);
        load(sprintf('%s/X0_nodes.mat',inDir),a);
        nodes_fixed=X0_nodes;
        expDir1=sprintf('../workdir/%s/%s', object, exp1);
        expDir2=sprintf('../workdir/%s/%s', object, exp2);
    elseif (strcmp(app,'CH'))        
        inDir=sprintf('/home/ipeterlik/Work/MUNI/Choral2/%s/', object);
        load(sprintf('%s/input/faces.mat',inDir),a);
        load(sprintf('%s/input/nodes.mat',inDir),a);
        load(sprintf('%s/params/nodes_fixed.mat',inDir),a);
        expDir1=sprintf('%s/%s', inDir, exp1);
        expDir2=sprintf('%s/%s', inDir, exp2);        
    end
    
    prescFile = ' '; %sprintf('%s/params/nodes_presc.mat',rootDir);
    if (exist(prescFile))
        load(prescFile,a);
        nds=nodes_presc;
        nodes_presc_ix=int16(nds(:,1));
        nodes_presc_val=nds(:,2:4);      
    else
        visParams.showDisplacedPoints=false;            
    end
    
    load(sprintf('%s/positions_final.mat', expDir1), a);     
    positions1 = positions_final;
    
    
    load(sprintf('%s/positions_final.mat', expDir2), a);     
    positions2 = positions_final;            
    
    if (visParams.exchangeYZ)                                    
        temp=nodes;
        nodes(:,2)=temp(:,3);
        nodes(:,3)=temp(:,2);                                                
    end
    
    if (visParams.exchangeYZ)
        temp=positions1;
        positions1(:,2)=temp(:,3);
        positions1(:,3)=temp(:,2);
        temp=positions2;
        positions2(:,2)=temp(:,3);
        positions2(:,3)=temp(:,2);
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
        else
            showTruth = 0;
            truthNodes = [];
            fprintf('Truth files %s or %s does not exist, not comparing!\n', truthNodeFile, truthFaceFile);
        end
    else
        showTruth = 0;
        truthNodes = [];
    end        

    %check data find the bounding box
    if (sum(visParams.liverAxesLimits) == 0 && visParams.showDeformed)                        
        deformedCoors1=positions1;           
        deformedCoors2=positions2;           
        auxCoors=[nodes; deformedCoors1; deformedCoors2; truthNodes];

        frameX=visParams.frameWidthRel*(max(auxCoors(:,1))-min(auxCoors(:,1)));
        frameY=visParams.frameWidthRel*(max(auxCoors(:,2))-min(auxCoors(:,2)));
        frameZ=visParams.frameWidthRel*(max(auxCoors(:,3))-min(auxCoors(:,3)));
        liverAxesLimits =[-frameX frameX -frameY frameY -frameZ frameZ] + [min(auxCoors(:,1)) max(auxCoors(:,1)) min(auxCoors(:,2)) max(auxCoors(:,2)) min(auxCoors(:,3)) max(auxCoors(:,3))];           
    end
    
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
    
    if (visParams.showFixedPoints)
        fixedPoints=nodes(nodes_fixed,:);
    end
    if (visParams.showDisplacedPoints)            
        presPointsRest=nodes(nodes_presc_ix,:);
        presPointsDisp=deformedCoors1(nodes_presc_ix,:);            
    end

    %%%%%figure(1);
    clf('reset');
    subplot('Position',[0.05 0.05 0.60 1]); %('Position',[200 200 200 200]);
    set(gca,'DataAspectRatio',[1 1 1]) %, 'Position', [10 10 800 800])
    axis(liverAxesLimits);
    hold on
    patch('Vertices', nodes, 'Faces', faces, ...
                'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColRest, 'EdgeColor', visParams.edgeColRest,'FaceAlpha',0.1);

    patch('Vertices', deformedCoors1, 'Faces', faces, ...
                'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColDef1, 'EdgeColor', visParams.edgeColDef1,'FaceAlpha',0.1);                                       
            
    patch('Vertices', deformedCoors2, 'Faces', faces, ...
                'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColDef2, 'EdgeColor', visParams.edgeColDef2,'FaceAlpha',0.1);                                                   

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

    az = 52;
    el = -20;
    view(29, 34);
    grid on	   

end

    

