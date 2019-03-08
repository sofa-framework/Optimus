mainDir = '../testPrecond/cyl10PF_Newton1_per200_ampMY002';
nsteps=400;
showFig=1;

sdaPrefix='ROUKF_star';
sdaDir1 = [sdaPrefix '_sd100_pcg0_OMP1_NOGUI'];
sdaDir2 = [sdaPrefix '_sd100_pcg1_OMP1_NOGUI'];
sdaDir3 = [sdaPrefix '_sd100_pcg2_OMP1_NOGUI'];
sdaDir4 = [sdaPrefix '_sd100_pcgAdapt5_OMP1_NOGUI'];

inputDir1 = [mainDir '/' sdaDir1];
inputDir2 = [mainDir '/' sdaDir2];
inputDir3 = [mainDir '/' sdaDir3];
inputDir4 = [mainDir '/' sdaDir4];
%disp([inputDir1 'vs' inputDir2])

t1=load([inputDir1 '/pred_sim']);
t2=load([inputDir2 '/pred_sim']);
t3=load([inputDir3 '/pred_sim']);
t4=load([inputDir4 '/pred_sim']);

tot(1) = sum(t1(1:nsteps));
tot(2) = sum(t2(1:nsteps));
tot(3) = sum(t3(1:nsteps));
tot(4) = sum(t4(1:nsteps));

if showFig
    figure; 
    plot(1:nsteps, t1(1:nsteps), 'r-', 1:nsteps, t2(1:nsteps), 'b-',  1:nsteps, t3(1:nsteps), 'g-', 1:nsteps, t4(1:nsteps), 'm-', 'LineWidth', 1);
    ylim([0e5,7e5]);
    title('Amp2 ROUKF Star OMP1 NoGUI');
    %legend({'OMP1 GUI', 'OMP2 GUI', 'OMP1 NOGUI', 'OMP2 NOGUI'});
    legend({'No precond', 'Precond Update1','Precond Update2','Precond Update5'});
    grid on
    xlabel('Time step');
    ylabel('Time [ms]')
end
