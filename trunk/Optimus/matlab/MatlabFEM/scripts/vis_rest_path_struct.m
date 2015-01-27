%argument determines the path to the computed data in the $ROOT/workdir/
function [ret] = vis_rest_path_struct(experimentPath)
    close all
    
    addpath('../confs');
    %eval('visualConf');
    
    ap=sprintf('../workdir/%s/expConf.m', experimentPath);
    if (~exist(ap, 'file'))
        disp(sprintf('Experiment configuration file %s does not exist, exiting...', ap));
        ret = -1;
        return;
    end
    
    disp(sprintf('Visualizing according to config file %s',ap));
    addpath(sprintf('../workdir/%s', experimentPath));    
    eval('expConf');    
        
    a='-ASCII';
    %dirPrefix=sprintf('%s/workdir/%s', strcParams.rootDir, strcParams.bodyID);
    workDir=sprintf('%s/workdir/%s', strcParams.rootDir, strcParams.bodyID);
    inDir=sprintf('%s/workdir/%s', strcParams.rootDir, experimentPath); % strcParams.experimentTag);
    if (isfield(strcParams, 'numIncSteps'))
        loadSteps=1:strcParams.numIncSteps;
    else
        loadSteps=1;
    end

    fileModelData=sprintf('%s/model_data', inDir);
    load(fileModelData);
    
    if (visParams.exchangeYZ)                                    
        temp=strcModel.matCoors;
        strcModel.matCoors(:,2)=temp(:,3);
        strcModel.matCoors(:,3)=temp(:,2);                                                
    end
    
    if (strcModel.givenPrescribed == 0)
        visParams.showForcePlots=0;
        visParams.showForceVector=0;
        visParams.showDisplacedPoints=0;
    end       

    pwd
    %load truth object to compare
    if (isfield(visParams,'truthNodeFile') & isfield(visParams,'truthFaceFile') )
        truthNodeFile = sprintf('../input_mesh/%s', visParams.truthNodeFile);        
        truthFaceFile = sprintf('../input_mesh/%s',visParams.truthFaceFile);          
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
        for loadStep=loadSteps          
            dataFile=sprintf('results_inc%03d.mat', loadStep);
            fileDeformedData=sprintf('%s/%s',inDir, dataFile);
            if (~exist(fileDeformedData))
                disp(sprintf('Data file %s does not exists! Exiting...', fileDeformedData));
                return
            else
                disp(sprintf('Reading from data file %s.', fileDeformedData));
            end

            load(fileDeformedData);
            
            if (visParams.exchangeYZ)                            
                temp=vertDisps;
                vertDisps(:,2)=temp(:,3);
                vertDisps(:,3)=temp(:,2);                                                                               
            end
            deformedCoors=strcModel.matCoors+vertDisps;   
            normForces(loadStep)=norm(forcesDispl);
            auxCoors=[strcModel.matCoors; deformedCoors; truthNodes];

            frameX=visParams.frameWidthRel*(max(auxCoors(:,1))-min(auxCoors(:,1)));
            frameY=visParams.frameWidthRel*(max(auxCoors(:,2))-min(auxCoors(:,2)));
            frameZ=visParams.frameWidthRel*(max(auxCoors(:,3))-min(auxCoors(:,3)));
            extremaTemp(loadStep,:)=[-frameX frameX -frameY frameY -frameZ frameZ] + [min(auxCoors(:,1)) max(auxCoors(:,1)) min(auxCoors(:,2)) max(auxCoors(:,2)) min(auxCoors(:,3)) max(auxCoors(:,3))];    
        end        
        visParams.liverAxesLimits = [min(extremaTemp(:,1)) max(extremaTemp(:,2)) min(extremaTemp(:,3)) max(extremaTemp(:,4)) min(extremaTemp(:,5)) max(extremaTemp(:,6))];
        
        if (max(normForces > 0))
            plotAxesLimits = [0 length(loadSteps) 0 max(normForces)];
        else
            plotAxesLimits = [0 1 0 1];
        end
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
 
    %az = 52;
    %el = -20;
    %view(29, 34);
    grid on
 
    for loadStep=loadSteps
        if (visParams.showDeformed)    
            dataFile=sprintf('results_inc%03d.mat', loadStep);
            fileDeformedData=sprintf('%s/%s',inDir, dataFile);        
                disp(sprintf('Reading from data file %s.', fileDeformedData));        

            load(fileDeformedData);

            if (visParams.exchangeYZ)                            
                temp=vertDisps;
                vertDisps(:,2)=temp(:,3);
                vertDisps(:,3)=temp(:,2);                                                
            end            
            
            deformedCoors=strcModel.matCoors+vertDisps;               
        else
            auxCoors=strcModel.matCoors;
        end               

        if (visParams.showForcePlots | visParams.showForceVector)
            fixedForcesStart=deformedCoors(strcModel.vecFixedNodes,:);
            fixedForcesArr=forcesFixed;
            displForcesStart=deformedCoors(strcModel.vecDisplNodes,:);

            startForces=[deformedCoors(strcModel.vecDisplNodes,:)];
            %startForces=[deformedCoors(strcModel.vecFixedNodes,:); deformedCoors(strcModel.vecDisplNodes,:)];
            %arrForces=[forcesFixed; forcesDispl];
            normForce(loadStep) = norm(forcesDispl);            
            for i=1:3
                forceComponents(loadStep,i) = sum(abs(forcesDispl(:,i)));
            end
            arrForces=forcesDispl;
        end
        
        if (visParams.showFixedPoints)
            fixedPoints=strcModel.matCoors(strcModel.vecFixedNodes,:);
        end
        if (visParams.showDisplacedPoints)            
            presPointsRest=strcModel.matCoors(strcModel.vecDisplNodes,:);
            presPointsDisp=deformedCoors(strcModel.vecDisplNodes,:);            
        end

        %%%%%figure(1);
        clf('reset');
        subplot('Position',[0.05 0.05 0.60 1]); %('Position',[200 200 200 200]);
        set(gca,'DataAspectRatio',[1 1 1]) %, 'Position', [10 10 800 800])
        axis(visParams.liverAxesLimits);
        hold on
        patch('Vertices', strcModel.matCoors, 'Faces', strcModel.matFaces, ...
                    'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColRest, 'EdgeColor', visParams.edgeColRest,'FaceAlpha',0.1);

        patch('Vertices', deformedCoors, 'Faces', strcModel.matFaces, ...
                    'FaceVertexCData',hsv(8),'FaceColor',visParams.faceColDef, 'EdgeColor', visParams.edgeColDef,'FaceAlpha',0.1);       
                                
            
        if (showTruth)
            patch('Vertices', truthNodes, 'Faces', truthFaces, ...
                    'FaceVertexCData',hsv(8),'FaceColor','b', 'EdgeColor', 'b','FaceAlpha',0.1);
        end

        if (visParams.showForceVector)   
                quiver3(startForces(:,1),startForces(:,2), startForces(:,3), ...
                     arrForces(:,1), arrForces(:,2), arrForces(:,3), visParams.forceVectorScale,'LineWidth',5,'MarkerSize',5,'Color',visParams.forceVectorColor);
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
        
  
        if (visParams.showForcePlots)
            %%%%%figure(2);
            subplot('Position', [0.70 0.05 0.29 0.20]); %  (1,2,2)    
            %axes('FontSize',14,'FontWeight','Bold'); %,'Parent',figPlot);
            %xlabel('Step');
            %ylabel('Force [N]');    
        
            plot(0:loadStep,[0; forceComponents(:,3)],'LineWidth',2,'Color','magenta');
            %legend('Force Z', 'Location', 'NW');
            axis(plotAxesLimits);
            %set(gca,'DataAspectRatio',[1 1 1])
            subplot('Position', [0.70 0.30 0.29 0.20]);
            plot(0:loadStep,[0; forceComponents(:,2)],'LineWidth',2,'Color','yellow');
            %legend('Force Y', 'Location', 'NW');
            axis(plotAxesLimits);   

            subplot('Position', [0.70 0.55 0.29 0.20]);
            plot(0:loadStep,[0; forceComponents(:,1)],'LineWidth',2','Color','cyan');
            %legend('Force X', 'Location', 'NW');
            axis(plotAxesLimits);

            subplot('Position', [0.70 0.80 0.29 0.20]);
            plot(0:loadStep,[0; normForce'],'LineWidth',2);
            %legend('|Force|', 'Location', 'NW');
            axis(plotAxesLimits);
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
