simDir='../estimateSurfaceForces/brickD_fix-L1_obs-PLB_int-Newton3_TR1';
da = 'ROUKF_Ep0.0_SDp10000_SDon1e-05_SpringsN4';
daDir=[simDir '/' da ];

maxTime = 500;
t=1:maxTime;

simForceFile = [simDir '/observations/toolForce.txt'];
daForceFile = [daDir '/toolForce.txt'];

simForce=load(simForceFile, '-ASCII');
daForce=load(daForceFile, '-ASCII');

size(simForce);
size(daForce);

sf=simForce(1:maxTime,:);
df=daForce(1:maxTime,:);

figure;
plot(t, sf(:,1), 'r-', t, sf(:,2), 'r:', t, sf(:,3), 'r--', t, df(:,1), 'b-', t, df(:,2), 'b:', t, df(:,3), 'b--');
title(da);

disp(da);
disp(simForce(maxTime,:));
disp(daForce(maxTime,:));

pct=norm(daForce(maxTime,:)-simForce(maxTime,:));

disp(100*pct/norm(simForce(maxTime,:)));