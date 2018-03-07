addpath '~/AncillaIP/Matlab';
%groundTruth=[1500 6000 2000];   %P1
groundTruth=zeros(1,16);   %P2
showStdev = 1;
nsteps=-1;

%integ='Euler1'
%integ='Newton3'
integ='VarSym3'

%filterType='UKFSimCorr';
%filterType='UKFClassic';
filterType='ROUKF';

inputDir=['../assimBC/brickD_FP1_OPogrid4_INT' integ '_TR1/DA_' filterType];


estStateFile=[inputDir '/state.txt'];
estVarFile=[inputDir '/variance.txt'];

%===================================================================
nparams=size(groundTruth,2);
estState=abs(load(estStateFile));
estVar=load(estVarFile);

if nsteps < 0
    nsteps=size(estState,1);
end

nstate=nparams
    
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
title(sprintf('State  %s', [filterType ' ' integ]));

return

figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estStd)), max(max(estStd))]);
hold on

for i=1:nstate
    plot(1:nsteps, estStd(:,i), 'Color', cls(i,:));
end
title(sprintf('Variance %s', filterType));









