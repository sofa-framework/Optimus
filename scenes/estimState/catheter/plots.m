% clear 
% close all
% clc

g= dlmread('g');
n = dlmread('n');

A=[0.0225 0.0225 0.0225 0.0225 0.0225 0.0225];

cov_00 = dlmread('print/print_cov_0');
cov_0=cat(1,A,cov_00);
inn_0 = dlmread('print/print_inn_0');
f_0= dlmread('print/FilteredBeam_0');
s_0= dlmread('print/print_state_0');

cov_10 = dlmread('print/print_cov_1');
cov_1=cat(1,A,cov_10);
inn_1 = dlmread('print/print_inn_1');
f_1= dlmread('print/FilteredBeam_1');
s_1= dlmread('print/print_state_1');


% cov_20 = dlmread('print/print_cov_2');
% cov_2=cat(1,A,cov_20);
% inn_2 = dlmread('print/print_inn_2');
% f_2= dlmread('print/FilteredBeam_2');
% s_2= dlmread('print/print_state_2');

%% PRINTING COVARIANCES 

figure('NumberTitle', 'off', 'Name', 'Covariance Test_0')
subplot(2,1,1)
plot(cov_0(1:18000,1:3))
hold on
plot(0,cov_0(1,1),cov_0(1,2),cov_0(1,3),'r*')
grid on
legend('pos x','pos y','pos z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
xlim([-1000 18000]);
title( 'Position Covariance' ) ;
subplot(2,1,2)
plot(cov_0(1:18000,4:6))
hold on
plot(0,cov_0(1,4),cov_0(1,5),cov_0(1,6),'r*')
xlim([-1000 18000]);
% ylim([0 0.1]);
grid on
legend('vel x','vel y','vel z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
title( 'Velocity Covariance' ) ;
suptitle('Pos, Vel Q = R')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/cov_0.bmp')



figure('NumberTitle', 'off', 'Name', 'Covariance Test_1')
subplot(2,1,1)
plot(cov_1(1:18000,1:3))
hold on
plot(0,cov_1(1,1),cov_1(1,2),cov_1(1,3),'r*')
grid on
legend('pos x','pos y','pos z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
xlim([-1000 18000]);
title( 'Position Covariance' ) ;
subplot(2,1,2)
plot(cov_1(1:18000,4:6))
hold on
plot(0,cov_1(1,4),cov_1(1,5),cov_1(1,6),'r*')
xlim([-1000 18000]);
% ylim([0 0.1]);
grid on
legend('vel x','vel y','vel z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
title( 'Velocity Covariance' ) ;
suptitle('Asynchronous Observations Pos, Vel R = Q')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/cov_1.bmp')


% figure('NumberTitle', 'off', 'Name', 'Covariance Test_2')
% subplot(2,1,1)
% plot(cov_2(1:18000,1:3))
% hold on
% plot(0,cov_2(1,1),cov_2(1,2),cov_2(1,3),'r*')
% grid on
% legend('pos x','pos y','pos z','Location','NorthEastOutside');
% xlabel('t - [simulation step]') % x-axis label
% ylabel('covariance') % y-axis label
% xlim([-1000 18000]);
% title( 'Position Covariance' ) ;
% subplot(2,1,2)
% plot(cov_2(1:18000,4:6))
% hold on
% plot(0,cov_2(1,4),cov_2(1,5),cov_2(1,6),'r*')
% xlim([-1000 18000]);
% ylim([0 0.1]);
% grid on
% legend('vel x','vel y','vel z','Location','NorthEastOutside');
% xlabel('t - [simulation step]') % x-axis label
% ylabel('covariance') % y-axis label
% title( 'Velocity Covariance' ) ;
% suptitle('Pos, Vel Q < R')
% set(gcf, 'PaperUnits', 'centimeters');
% set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/cov_2.bmp')

%% PRINTING INNOVATION 


figure('NumberTitle', 'off', 'Name', 'Innovation Test 0')
plot(inn_0(1:18000,:))
grid on
legend('Innovation x','Innovation y','Innovation z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('innovation - [m]') % y-axis label
title({'Pos, Vel Q = R','Innovation - Difference between predicted position and observed position'} ) ;
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/inn_0.bmp')

figure('NumberTitle', 'off', 'Name', 'Innovation Test 1')
plot(inn_1(1:18000,:))
grid on
legend('Innovation x','Innovation y','Innovation z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('innovation - [m]') % y-axis label
title({'Asynchronous Observations Pos, Vel R = Q','Innovation - Difference between predicted position and observed position'} ) ;
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/inn_1.bmp')

% figure('NumberTitle', 'off', 'Name', 'Innovation Test 2')
% plot(inn_2(1:18000,:))
% grid on
% legend('Innovation x','Innovation y','Innovation z','Location','NorthEastOutside');
% xlabel('t - [simulation step]') % x-axis label
% ylabel('innovation - [m]') % y-axis label
% title({'Pos, Vel Q < R','Innovation - Difference between predicted position and observed position'} ) ;
% set(gcf, 'PaperUnits', 'centimeters');
% set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/inn_2.bmp')

%% PRINTING DISTANCE FILTER GROUND TRUTH 

F_0=s_0(1:18000,1:3);
F_1=s_1(1:18000,1:3);
% F_2=f_2(1:18000,2:4);
G=g(1:18000,2:4);

d_0=sqrt(sum(((F_0-G).^2)'));
d_1=sqrt(sum(((F_1-G).^2)'));
% d_2=sqrt(sum(((F_2-G).^2)'));

figure('NumberTitle', 'off', 'Name', 'Distance Test 0 ')
plot(d_0)
grid on
title({'Pos, Vel Q = R','Distance between Fiter Result and Ground Truth '})
xlabel('t - [simulation step]') % x-axis label
ylabel('Distance - [m]') % y-axis label
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/haus_0.bmp')

figure('NumberTitle', 'off', 'Name', 'Distance Test 1 ')
plot(d_1)
grid on
title({'Asynchronous Observations Pos, Vel R = Q','Distance between Fiter Result and Ground Truth '})
xlabel('t - [simulation step]') % x-axis label
ylabel('Distance - [m]') % y-axis label
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
saveas(gcf,'print/haus_1.bmp')

% figure('NumberTitle', 'off', 'Name', 'Distance Test 2 ')
% plot(d_2)
% grid on
% title({'Pos, Vel Q < R','Distance between Fiter Result and Ground Truth '})
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Distance - [m]') % y-axis label
% set(gcf, 'PaperUnits', 'centimeters');
% set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/haus_2.bmp')
