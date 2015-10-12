
val=load('evals/valComplexDyn_22NOISE_ABER3'); I=1e2; E=1e2; stop=1000;
figure; 
plot(val(1:stop, :),'LineWidth',2); 
title(sprintf('\nSimple initVal=%g initE=%g\n final: kF_1 =  %g  kF_2 = %g kF_3 =  %g\n kL_1 = %g kL_2 = %g\n kA_1 = %g kA_2 = %g\n', I, E, val(stop,1), val(stop,2), val(stop,3), val(stop,4), val(stop,5), val(stop, 6), val(stop, 7)), 'FontWeight','bold', 'FontSize', 12); 
legend('kF_1', 'kF_2', 'kF_3', 'kL_1', 'kL_2', 'kA_1', 'kA_2');

%grep Actual complexDyn_22_NOISE_ABER2  | awk '{print $4" "$5" "$6" "$7" "$8" "$9" "$10}'  > valComplexDyn_22NOISE_ABER2
