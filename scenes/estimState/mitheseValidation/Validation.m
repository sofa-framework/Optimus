clear 
close all
clc


state_mass5 = dlmread('print_state_mass5');
inn_mass5 = dlmread('print_inn_mass5');
cov_mass5 = dlmread('print_cov_mass5');
g = dlmread('gnoplan');
G=g(:,2:16);

figure(1);
subplot(4,1,1);
plot(cov_mass5(1:0.1:6000,1:3))
hold on
% plot(cov_mass5(1,1),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
% %axis([-20 6000 0.0000003 0.00000055])
title({' {\itPos&Vel estimation and Mass assimilation}',' ','Covariance of 2nd Node Pos&Velition'}) ;
subplot(4,1,2);
plot(cov_mass5(1:0.1:6000,4:6))
hold on
% plot(cov_mass5(1,4),'o')
hold off
grid on
ylabel('Covariance') % y-axis label
legend('X','Y','Z', 'Location','NorthEastOutside');
%axis([-20 6000 0.0000003 0.00000055])
title('Covariance of 3rd Node Pos&Velition') ;
subplot(4,1,3);
plot(cov_mass5(1:0.1:6000,7:9))
hold on
% plot(cov_mass5(1,7),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
%axis([-20 6000 0.0000003 0.00000055])
title('Covariance of 4th Node Pos&Velition') ;
subplot(4,1,4);
plot(cov_mass5(1:0.1:6000,10:12))
hold on
% plot(cov_mass5(1,10),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
%axis([-20 6000 0.0000003 0.00000055])
title('Covariance of 5th Node Pos&Velition') ;
% subplot(5,1,5);
% plot(cov_mass5(1:0.1:6000,13:15))
% hold on
% plot(cov_mass5(1,13),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
% ylabel('Covariance') % y-axis label
% %axis([-20 6000 0.0000003 0.00000055])
% title('Covariance of Fifth Node Pos&Velition') ;
saveas(gcf, 'Pos&VelCov_mass5', 'pdf')
grid on

figure(2);
plot(cov_mass5(1:6000,25))
hold on
% plot(cov_mass5(1,13),'o')
hold off
grid on
% leged('Mass Value','Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
xlabel('t - [simulation step]') % x-axis label
title({' {\itPos&Vel estimation and Mass assimilation}',' ','Covariance of Mass '}) ;
% 
% 
% 
% figure(3);
% subplot(5,3,1);
% plot(inn_mass5(1:6000,1)); grid on
% ylabel('Innovation - [mm]') % y-axis label
% title({' {\itPos&Vel estimation and Mass assimilation}',' ','First Observation Innovation'}) ;
% % %axis([-20 6000 -0.005 0.005])
% subplot(5,3,2);
% plot(inn_mass5(1:6000,2));grid on
% % %axis([-20 6000 -0.005 0.005])
% ylabel('Innovation - [mm]') % y-axis label
% title('Second Observation Innovation') ;
% subplot(5,3,3);
% plot(inn_mass5(1:6000,3));grid on
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005])
% title('Third Observation Innovation') ;
% subplot(5,3,4);
% plot(inn_mass5(1:6000,4));grid on
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005])
% title('Fourth Observation Innovation') ;
% subplot(5,3,5);
% plot(inn_mass5(1:6000,5));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('Fifth Observation Innovation') ;
% subplot(5,3,6);
% plot(inn_mass5(1:6000,6));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('Sixth Observation Innovation') ;
% subplot(5,3,7);
% plot(inn_mass5(1:6000,7));grid on
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005])
% title('7th Observation Innovation') ;
% subplot(5,3,8);
% plot(inn_mass5(1:6000,8));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('8th Observation Innovation') ;
% subplot(5,3,9);
% plot(inn_mass5(1:6000,9));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('9th Observation Innovation') ;
% subplot(5,3,10);
% plot(inn_mass5(1:6000,10));grid on
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005])
% title('10th Observation Innovation') ;
% subplot(5,3,11);
% plot(inn_mass5(1:6000,11));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('11th Observation Innovation') ;
% subplot(5,3,12);
% plot(inn_mass5(1:6000,12));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('12th Observation Innovation') ;
% subplot(5,3,13);
% plot(inn_mass5(1:6000,13));grid on
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005])
% title('13th Observation Innovation') ;
% subplot(5,3,14);
% plot(inn_mass5(1:6000,14));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('14th Observation Innovation') ;
% subplot(5,3,15);
% plot(inn_mass5(1:6000,15));
% ylabel('Innovation - [mm]') % y-axis label
% % %axis([-20 6000 -0.005 0.005]);
% title('15th Observation Innovation') ;
% saveas(gcf, 'Pos&VelInn_mass5', 'pdf')
% grid on
% hold off

H_mass5=zeros(6000,1);
for i = 1:6000
   H_mass5(i) = HausdorffDist(state_mass5(i,1:12),G(i,4:15));
end

figure(4);
pp5 = splinefit(1:6000,H_mass5,8,1);  
xx = linspace(1,6000);
y5 = ppval(pp5,xx);
ast=ones(size(H_mass5))*y5(end);
plot(H_mass5(1:6000),'g')
hold on
plot(ast,'r'), 
legend('Hausdorff Distance',['Min Dist =' num2str(ast(1))]);
grid on
title({' {\itPos&Vel estimation and Mass assimilation}',' ','Hausdorff Distance between Fiter Beam and Ground Truth Beam '})
xlabel('t - [simulation step]') % x-axis label
ylabel('Hausdorff Distance - [mm]') % y-axis label
saveas(gcf, 'HausdorffPos&Vel_mass5', 'pdf')
hold off

figure(5)
plot(state_mass5(:,25)), grid on;
hold on
plot(ones(size(state_mass5)),'r')
plot(ones(size(state_mass5))*state_mass5(end,end),'g')
title({'Mass Estimation '})
xlabel('t - [simulation step]') % x-axis label
ylabel('mass - [Kg]') % y-axis label


