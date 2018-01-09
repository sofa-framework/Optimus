clear 
close all
clc


state_mass = dlmread('print_state_mass');
inn_mass = dlmread('print_inn_mass');
cov_mass = dlmread('print_cov_mass');
g = dlmread('g');
G=g(:,2:16);

figure(1);
subplot(5,1,1);
plot(cov_mass(1:0.1:6000,1:3))
hold on
plot(cov_mass(1,1),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
axis([-20 6000 0.0000003 0.00000055])
title({' {\itPOSITION ESTIMATION w Speed Correction Q=R}',' ','Covariance of First Node Position'}) ;
subplot(5,1,2);
plot(cov_mass(1:0.1:6000,4:6))
hold on
plot(cov_mass(1,4),'o')
hold off
grid on
ylabel('Covariance') % y-axis label
legend('X','Y','Z', 'Location','NorthEastOutside');
axis([-20 6000 0.0000003 0.00000055])
title('Covariance of Second Node Position') ;
subplot(5,1,3);
plot(cov_mass(1:0.1:6000,7:9))
hold on
plot(cov_mass(1,7),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
axis([-20 6000 0.0000003 0.00000055])
title('Covariance of Third Node Position') ;
subplot(5,1,4);
plot(cov_mass(1:0.1:6000,10:12))
hold on
plot(cov_mass(1,10),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
axis([-20 6000 0.0000003 0.00000055])
title('Covariance of Fourth Node Position') ;
subplot(5,1,5);
plot(cov_mass(1:0.1:6000,13:15))
hold on
plot(cov_mass(1,13),'o')
hold off
grid on
legend('X','Y','Z', 'Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('Covariance') % y-axis label
axis([-20 6000 0.0000003 0.00000055])
title('Covariance of Fifth Node Position') ;
saveas(gcf, 'PosCov_mass', 'pdf')
grid on

figure(2);
plot(cov_mass(1:0.1:6000,16))
hold on
plot(cov_mass(1,16),'o')
hold off
grid on
legend('Mass Value','Location','NorthEastOutside');
ylabel('Covariance') % y-axis label
xlabel('t - [simulation step]') % x-axis label
title({' {\itPOSITION ESTIMATION w Speed Correction Q=R}',' ','Covariance of Mass Parameter'}) ;



figure(3);
subplot(5,3,1);
plot(inn_mass(1:6000,1)); grid on
ylabel('Innovation - [mm]') % y-axis label
title({' {\itPOSITION ESTIMATION w Speed Correction Q=R}',' ','First Observation Innovation'}) ;
% axis([-20 6000 -0.005 0.005])
subplot(5,3,2);
plot(inn_mass(1:6000,2));grid on
% axis([-20 6000 -0.005 0.005])
ylabel('Innovation - [mm]') % y-axis label
title('Second Observation Innovation') ;
subplot(5,3,3);
plot(inn_mass(1:6000,3));grid on
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005])
title('Third Observation Innovation') ;
subplot(5,3,4);
plot(inn_mass(1:6000,4));grid on
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005])
title('Fourth Observation Innovation') ;
subplot(5,3,5);
plot(inn_mass(1:6000,5));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('Fifth Observation Innovation') ;
subplot(5,3,6);
plot(inn_mass(1:6000,6));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('Sixth Observation Innovation') ;
subplot(5,3,7);
plot(inn_mass(1:6000,7));grid on
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005])
title('7th Observation Innovation') ;
subplot(5,3,8);
plot(inn_mass(1:6000,8));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('8th Observation Innovation') ;
subplot(5,3,9);
plot(inn_mass(1:6000,9));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('9th Observation Innovation') ;
subplot(5,3,10);
plot(inn_mass(1:6000,10));grid on
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005])
title('10th Observation Innovation') ;
subplot(5,3,11);
plot(inn_mass(1:6000,11));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('11th Observation Innovation') ;
subplot(5,3,12);
plot(inn_mass(1:6000,12));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('12th Observation Innovation') ;
subplot(5,3,13);
plot(inn_mass(1:6000,13));grid on
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005])
title('13th Observation Innovation') ;
subplot(5,3,14);
plot(inn_mass(1:6000,14));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('14th Observation Innovation') ;
subplot(5,3,15);
plot(inn_mass(1:6000,15));
ylabel('Innovation - [mm]') % y-axis label
% axis([-20 6000 -0.005 0.005]);
title('15th Observation Innovation') ;
saveas(gcf, 'PosInn_mass', 'pdf')
grid on
hold off

H_mass=zeros(6000,1);
for i = 1:6000
   H_mass(i) = HausdorffDist(state_mass(i,1:15),G(i,:));
end

figure(4);
pp5 = splinefit(1:6000,H_mass,8,1);  
xx = linspace(1,6000);
y5 = ppval(pp5,xx);
ast=ones(size(H_mass))*y5(end);
plot(H_mass(1:6000),'g')
hold on
plot(ast,'r'), 
legend('Hausdorff Distance',['Min Dist =' num2str(ast(1))]);
grid on
title({' {\itPOSITION ESTIMATION w Speed Correction Q=R}',' ','Hausdorff Distance between Fiter Beam and Ground Truth Beam '})
xlabel('t - [simulation step]') % x-axis label
ylabel('Hausdorff Distance - [mm]') % y-axis label
saveas(gcf, 'HausdorffPos_mass', 'pdf')
hold off

figure(5)
plot(state_mass(:,16)), grid on;
