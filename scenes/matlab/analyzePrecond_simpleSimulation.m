pth='../testPrecond/out_testMesh72k/';

filePrefix='out_OMP4_Homog_';
fileSuffix='ParAdapt25A';

time_assemb=load([pth filePrefix fileSuffix '_assemb']);
time_solve=load([pth filePrefix fileSuffix '_solve']);
ncgit=load([pth filePrefix fileSuffix '_cgit']);

cost = time_assemb + time_solve;

figure; yyaxis left; plot(cost); yyaxis right; plot(ncgit); grid on; title([strrep(filePrefix, '_', ' ') strrep(fileSuffix, '_', ' ')]);

fprintf('\nShowing: %s %s %s\n', pth, filePrefix, fileSuffix);
fprintf('Time [ms] total: %f  mean: %f std: %f  max: %f\n', sum(cost(2:end))/1000, mean(cost(2:end))/1000, std(cost(2:end))/1000, max(cost(2:end))/1000);

