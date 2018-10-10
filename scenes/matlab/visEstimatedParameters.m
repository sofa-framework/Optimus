addpath '~/AncillaIP/Matlab';
%groundTruth=[1500 6000 2000];   %P1
%groundTruth=[3500 4000 1000 6000 2000 7000 2500 8000 3000 1500];
%groundTruth=[1000 4000 2000];   %P1
groundTruth=[3000, 7000];
%$groundTruth=zeros(1,10);   %P2
%groundTruth = zeros(1,16);
showStdev = 1;
nsteps=500;

object='cylinder2';
numEl='138';  %2264 128
numElSda='138';
excit='press';   % force, displ
obsID = 'mid';
fem='StVenant';
integ='Newton3';
suffix='test_0.499nu';
filterType='UKFSimCorr';  % "ROUKF", "UKFSimCorr", and "UKFClassic"
transform = 'project';
sdaParams='45_45_200_ns1-5';
saveImage=0;

mainDir = [ '../assimStiffness/' object '_' numEl  '_' excit '_' obsID '_' fem '_' integ '_' suffix '/' ];

inputDir = [ mainDir filterType '_' numElSda  '_' transform '_' sdaParams ];
disp(inputDir)
%old naming convention:
%inputDir = [ mainDir filterType '_' transform '_' sdaParams ]


%inputDir='../assimStiffness/cyl3gravity_Euler1/UKFSimCorr_obs33_proj0'

estStateFile=[inputDir '/state.txt'];
estVarFile=[inputDir '/variance.txt'];
estCovarFile=[inputDir '/covariance.txt'];

%===================================================================
nparams=size(groundTruth,2);
estState=load(estStateFile);
estVar=load(estVarFile);
estCovar = load(estCovarFile);

if nsteps < 0
    nsteps=size(estState,1);
end

nstate=nparams;

ncovar=nparams*(nparams-1)/2;

if strcmp(transform,'abs')
    estState=abs(estState(1:nsteps,nstate-nparams+1:nstate));
    estVar=abs(estVar(1:nsteps,nstate-nparams+1:nstate));
    estStd=sqrt(estVar);
end


if strcmp(transform,'exp')
    estState=exp(estState(1:nsteps,nstate-nparams+1:nstate));
    estVar=estVar(1:nsteps,nstate-nparams+1:nstate);
    estStd=exp(sqrt(estVar));
end

if strcmp(transform,'project')
    estState=estState(1:nsteps,nstate-nparams+1:nstate);
    estVar=estVar(1:nsteps,nstate-nparams+1:nstate);
    estStd=sqrt(estVar);
end

correl = zeros(size(estCovar));

for ns = 1:nsteps
    gli = 0;
    for i=1:nparams
        for j = i+1:nparams
            gli = gli+1;
            correl(ns,gli) = estCovar(ns,gli)/(estStd(ns,i)* estStd(ns,j));
        end
    end
end

format short g
estimParams=estState(nsteps,:);
disp(groundTruth);
disp(estimParams);
disp(estStd(nsteps,:));
disp(correl(nsteps,:));

if length(estimParams) == 2
    k1=estimParams(1);
    k2=estimParams(2);
    keff = (k1 * k2)/(k1 + k2);
    keff2 = k2 / (k1+k2);
    k1=groundTruth(1);
    k2=groundTruth(2);
    keff_gt=(k1 * k2) / (k1 + k2);
    keff2_gt = k2 / (k1+k2);
    fprintf('Effective: GT: %f  estimated: %f\n', keff_gt, keff);
    fprintf('Effective2: GT: %f  estimated: %f\n', keff2_gt, keff2);
end




minval=0;
maxval=max(max(estState)) + max(max(estStd));

figure('InvertHardcopy','off','Color',[1 1 1],'Position', [0 0 1000 800]);
%axes('XLim', [1,nsteps], 'YLim', [0, 1.2*maxval]);
hold on
gtState=ones(size(estState));
cls=distinguishable_colors(nparams, 'w');

for i=1:nparams
    plot(1:nsteps, groundTruth(i)*ones(1,nsteps), 'Color', cls(i,:),'LineStyle', ':', 'LineWidth', 3);    
    if showStdev
        %errorbar(estState(:,i), estStd(:,i), 'Color', cls(i,:), 'MarkerSize', 2);
        plot(1:nsteps, estState(:,i)+estStd(:,i), 'Color', cls(i,:), 'LineStyle','--');
        plot(1:nsteps, estState(:,i)-estStd(:,i), 'Color', cls(i,:), 'LineStyle','--');
        plot(1:nsteps, estState(:,i), 'Color', cls(i,:), 'LineWidth', 2);
    else
        plot(1:nsteps, estState(:,i), 'Color', cls(i,:));
    end
    
end

mytitle=sprintf('%s', [filterType ' ' numEl ' ' numElSda ' ' integ ' ' obsID ' ' transform ' ' strrep(suffix, '_', ' '), ' ' strrep(sdaParams,'_',' ')]);
title(mytitle);
disp(mytitle);
box on
grid on

if saveImage
    fileName = strrep(mytitle,' ','_');
    saveas(gcf, [fileName '.png'], 'png')
end


% figure; 
% axes('XLim', [1,nsteps], 'YLim', [0, 1.2*maxval]);
% hold on
% plot(1:nsteps, correl(1:nsteps,:));
% title(sprintf('Correlation %s', [filterType ' ' integ ' ' obsID ' ' transform]));


return

figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estStd)), 2*max(max(estStd))]);
hold on

for i=1:nstate
    plot(1:nsteps, estStd(:,i), 'Color', cls(i,:));
end
title(sprintf('Variance %s', filterType));









