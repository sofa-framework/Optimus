clear 
close all
clc
% 
% load('plots/g.mat','g');
% g1= g(:,2:4);
% g2= g(:,9:11);
% g3= g(:,16:18);
% G=cat(2,g1,g2,g3);
% 
% 



s_0 = dlmread('print_state_0');
inn_0 = dlmread('print_inn_0');
% f_0 = dlmread('print_filter_0');
cov_0 = dlmread('print_cov_0');
%
s_1 = dlmread('print_state_1');
inn_1 = dlmread('print_inn_1');
% f_1 = dlmread('print_filter_1');
cov_1 = dlmread('print_cov_1');
%
s_2 = dlmread('print_state_2');
inn_2 = dlmread('print_inn_2');
% f_2 = dlmread('print_filter_2');
cov_2 = dlmread('print_cov_2');
%
s_3 = dlmread('print_state_3');
inn_3 = dlmread('print_inn_3');
% f_3 = dlmread('print_filter_3');
cov_3 = dlmread('print_cov_3');
%
s_4 = dlmread('print_state_4');
inn_4 = dlmread('print_inn_4');
% f_4 = dlmread('print_filter_4');
cov_4 = dlmread('print_cov_4');
%
s_5 = dlmread('print_state_5');
inn_5 = dlmread('print_inn_5');
% f_5 = dlmread('print_filter_5');
cov_5 = dlmread('print_cov_5');
%
s_6 = dlmread('print_state_6');
inn_6 = dlmread('print_inn_6');
% f_6 = dlmread('print_filter_6');
cov_6 = dlmread('print_cov_6');
%
s_7 = dlmread('print_state_7');
inn_7 = dlmread('print_inn_7');
% f_7 = dlmread('print_filter_7');
cov_7 = dlmread('print_cov_7');
%
s_8 = dlmread('print_state_8');
inn_8 = dlmread('print_inn_8');
% f_8 = dlmread('print_filter_8');
cov_8 = dlmread('print_cov_8');
%
s_9 = dlmread('print_state_9');
inn_9 = dlmread('print_inn_9');
% f_9 = dlmread('print_filter_9');
cov_9 = dlmread('print_cov_9');
%
s_10 = dlmread('print_state_10');
inn_10 = dlmread('print_inn_10');
% f_10 = dlmread('print_filter_10');
cov_10 = dlmread('print_cov_10');
%
s_11 = dlmread('print_state_11');
inn_11 = dlmread('print_inn_11');
% f_11 = dlmread('print_filter_11');
cov_11 = dlmread('print_cov_11');
%
% 
% f1_0= f_0(:,2:4);
% f2_0= f_0(:,9:11);
% f3_0= f_0(:,16:18);
% F_0=cat(2,f1_0,f2_0,f3_0);
% 
% H_0=zeros(size(F_0,1),1);
% for i = 1:(size(G,1))
%    H_0(i) = HausdorffDist(F_0(i,:),G(i,:));
% end



figure;
plot(cov_0(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance NO SPEED Correction' ) ;
saveas(gcf, 'PosCov_0', 'pdf')

figure;
plot(inn_0(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_0', 'pdf')


figure;
plot(cov_1(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance NO SPEED Correction' ) ;
saveas(gcf, 'PosCov_1', 'pdf')

figure;
plot(inn_1(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation ') ;
saveas(gcf, 'PosInn_1', 'pdf')


figure;
plot(cov_2(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_2', 'pdf')

figure;
plot(inn_2(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_2', 'pdf')

figure;
plot(cov_3(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_3', 'pdf')

figure;
plot(inn_3(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_3', 'pdf')

figure;
plot(cov_4(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_4', 'pdf')

figure;
plot(inn_4(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_4', 'pdf')

figure;
plot(cov_5(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_5', 'pdf')

figure;
plot(inn_5(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_5', 'pdf')

figure;
plot(cov_6(1:600,1:15))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_6', 'pdf')

figure;
plot(inn_6(1:600,1:15)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_6', 'pdf')



figure;
plot(cov_7(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_7', 'pdf')

figure;
plot(inn_7(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_7', 'pdf')

figure;
plot(cov_8(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_8', 'pdf')

figure;
plot(inn_8(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_8', 'pdf')

figure;
plot(cov_9(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_9', 'pdf')

figure;
plot(inn_9(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_9', 'pdf')

figure;
plot(cov_11(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_11', 'pdf')

figure;
plot(inn_11(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_11', 'pdf')

figure;
plot(cov_10(1:600,:))
legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z', 'fourthNode x','fourthNode y','fourthNode z','fifthNode x','fifthNode y','fifthNode z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
title( 'Position Covariance' ) ;
saveas(gcf, 'PosCov_10', 'pdf')

figure;
plot(inn_10(1:600,:)); 
xlabel('t - [simulation step]') % x-axis label
ylabel('Innovation - [mm]') % y-axis label
 title( 'Innovation') ;
saveas(gcf, 'PosInn_10', 'pdf')

 
% figure;
% plot(s_0(1:600,:)); 
% legend('firstNode x','firstNode y','firstNode z','secondNode x','secondNode y','secondNode z','thirdNode x','thirdNode y','thirdNode z','Location','NorthEastOutside');
% xlabel('t - [simulation step]') % x-axis label
% ylabel('state - [m/s]') % y-axis label
% title( 'Position Final State') ;
% saveas(gcf, 'plots/PosState_0', 'pdf')
% 
% E_0=find(H_0<0.001);
% figure;
% plot(H_0(1:size(G,1)))
% hold on
% plot(E_0(1),H_0(E_0(1)),'r*')
% title('Hausdorff Distance between Fiter Beam and Ground Truth Beam ')
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Hausdorff Distance - [mm]') % y-axis label
% saveas(gcf, 'plots/HausdorffPos_0', 'pdf')
% hold off

%


