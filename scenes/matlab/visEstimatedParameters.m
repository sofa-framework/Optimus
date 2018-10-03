addpath '~/AncillaIP/Matlab';
%groundTruth=[1500 6000 2000];   %P1
%groundTruth=[3500 4000 1000 6000 2000 7000 2500 8000 3000 1500];
%groundTruth=[1000 4000 2000];   %P1
groundTruth=[3000, 7000]
%$groundTruth=zeros(1,10);   %P2
%groundTruth = zeros(1,16);
showStdev = 1;
nsteps=100;

object='cylinder2';
numEl='2264';
excit='displ';   % force, displ
obsID = 'mid-end';
fem='StVenant';
integ='Newton3';
suffix='test1_0.0nu';
filterType='ROUKF';  % "ROUKF", "UKFSimCorr", and "UKFClassic"
transform = 'project';
sdaParams='45_45_ns-5';

mainDir = [ '../assimStiffness/' object '_' numEl  '_' excit '_' obsID '_' fem '_' integ '_' suffix '/' ]
inputDir = [ mainDir filterType '_' transform '_' sdaParams ]


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
disp(estState(nsteps,:))
disp(estStd(nsteps,:))
disp(correl(nsteps,:))


minval=0;
maxval=max(max(estState)) + max(max(estStd));

figure; 
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
title(sprintf('State  %s', [filterType ' ' integ ' ' obsID ' ' transform]));

figure; 
%axes('XLim', [1,nsteps], 'YLim', [0, 1.2*maxval]);
hold on
plot(1:nsteps, correl(1:nsteps,:));
title(sprintf('Correlation %s', [filterType ' ' integ ' ' obsID ' ' transform]));


return

figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estStd)), 2*max(max(estStd))]);
hold on

for i=1:nstate
    plot(1:nsteps, estStd(:,i), 'Color', cls(i,:));
end
title(sprintf('Variance %s', filterType));









