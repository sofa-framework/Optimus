addpath '~/AncillaIP/Matlab';
%groundTruth=[1500 6000 2000];   %P1
groundTruth=zeros(1,16);   %P2
showStdev = 1;
nsteps=-1;

%prefix = '../assimStiffness/outCyl3_770_P1_pull_';
%prefix = '../assimStiffness/outCyl3_770_P2';
prefix='../assimBC/outSynth1Euler_';

%filterType='UKFSimCorr';
%filterType='UKFClassic';
filterType='ROUKF';

suffix= '_failed';

estStateFile=[prefix filterType '/state.txt'];
estVarFile=[prefix filterType '/variance.txt'];

%===================================================================
nparams=size(groundTruth,2);
estState=abs(load(estStateFile));
estVar=load(estVarFile);

if nsteps < 0
    nsteps=size(estState,1);
end
    
estState=estState(1:nsteps,nstate-nparams+1:nstate);
estVar=estVar(1:nsteps,nstate-nparams+1:nstate);
estStd=sqrt(estVar);

minval=0;
maxval=max(max(estState)) + max(max(estStd));

figure; 
axes('XLim', [1,nsteps], 'YLim', [0, maxval]);
hold on
gtState=ones(size(estState));
cls=distinguishable_colors(nparams, 'w');

for i=1:nparams
    plot(1:nsteps, groundTruth(i)*ones(1,nsteps), 'Color', cls(i,:),'LineStyle', ':');    
    if showStdev
        %errorbar(estState(:,i), estStd(:,i), 'Color', cls(i,:), 'MarkerSize', 2);
        plot(1:nsteps, estState(:,i)+estStd(:,i), 'Color', cls(i,:), 'LineStyle','--');
        plot(1:nsteps, estState(:,i)-estStd(:,i), 'Color', cls(i,:), 'LineStyle','--');
        plot(1:nsteps, estState(:,i), 'Color', cls(i,:), 'LineWidth', 2);
    else
        plot(1:nsteps, estState(:,i), 'Color', cls(i,:));
    end
    
end
title(sprintf('State  %s', filterType));

return

figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estStd)), max(max(estStd))]);
hold on

for i=1:nstate
    plot(1:nsteps, estStd(:,i), 'Color', cls(i,:));
end
title(sprintf('Variance %s', filterType));









