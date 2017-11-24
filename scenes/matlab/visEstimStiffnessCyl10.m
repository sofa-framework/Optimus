addpath '~/AncillaIP/Matlab';
groundTruth=[3500 4000]  % 1000 6000 2000 7000 2500 8000 3000 1500];
numParams=size(groundTruth,2);

estStateFile='../assimStiffness/outCyl10_2Par/state_test.txt';
estVarFile='../assimStiffness/outCyl10_2Par/variance_test.txt';
filterType='UKF';

%estStateFile='../assimStiffness/outCyl10/state_test.txt';
%estVarFile='../assimStiffness/outCyl10/variance_test.txt';
%filterType='ROUKF';

estState=abs(load(estStateFile));
estVar=load(estVarFile);
estStd=sqrt(estVar);


nsteps=size(estState,1);
nstate=size(estState,2);

figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estState)), max(max(estState))]);
hold on
gtState=ones(size(estState));
cls=distinguishable_colors(numParams, 'w');

for i=1:nstate
    plot(1:nsteps, groundTruth(i)*ones(1,nsteps), 'Color', cls(i,:),'LineStyle', '--');
    plot(1:nsteps, estState(:,i), 'Color', cls(i,:));
end
title(sprintf('State  %s', filterType));


figure; 
axes('XLim', [1,nsteps], 'YLim', [min(min(estStd)), max(max(estStd))]);
hold on

for i=1:nstate
    plot(1:nsteps, estStd(:,i), 'Color', cls(i,:));
end
title(sprintf('Variance %s', filterType));









