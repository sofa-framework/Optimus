clear 
close all
clc

g= dlmread('g');
n = dlmread('n');
cov_2 = dlmread('print/print_cov_1');
inn_2 = dlmread('print/print_inn_1');
f= dlmread('print/FilteredBeam_1');
s_2= dlmread('print/print_state_2');

% figure('NumberTitle', 'off', 'Name', 'GroundTruth and Noisy Trajectory')
% ax(1)=subplot(3,1,1);
% plot(g(1:10:500,2),'-')
% grid on
% hold on
% plot(n(1:10:500,2), '-x')
% xlabel('t - [simulation step]') % x-axis label
% ylabel('X - [m]') % y-axis label
% % xlim([g(1,1) g(end,1)]);
% % ylim([-0.5 1.5]);
% title( 'GroundTruth and Noisy Trajectory - X ' ) ;
% x=abs(mean(g(1:100:end,2)-n(1:100:end,2)))/0.01;
% text(g(500,1),g(500,2),['Mean Error =', num2str(x)],'VerticalAlignment','top',...
%     'HorizontalAlignment','right',...
%     'FontSize',8);
% ax(2)=subplot(3,1,2);
% plot(g(1:10:500,3),'-')
% grid on
% hold on
% plot(n(1:10:500,3), '-x')
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Y - [m]') % y-axis label
% title( 'GroundTruth and Noisy Trajectory - Y ' ) ;
% % xlim([g(1,1) g(end,1)]);
% % ylim([-0.6 0]);
% x=abs(mean(g(1:100:end,3)-n(1:100:end,3)))/0.01;
% text(g(500,1),g(500,3),['Mean Error =', num2str(x)],'VerticalAlignment','bottom',...
%     'HorizontalAlignment','right',...
%     'FontSize',8);
% ax(3)=subplot(3,1,3);
% plot(g(1:10:500,4),'-')
% grid on
% hold on
% plot(n(1:10:500,4), '-x')
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Z - [m]') % y-axis label
% title( 'GroundTruth and Noisy Trajectory - Z ' ) ;
% % xlim([g(1,1) g(end,1)]);
% % ylim([-0.05 0.3]);
% x=abs(mean(g(1:100:end,4)-n(1:100:end,4)))/0.01;
% text(g(500,1),g(500,4),['Mean Error =', num2str(x)],'VerticalAlignment','top',...
%     'HorizontalAlignment','right',...
%     'FontSize',8);

%%
figure('NumberTitle', 'off', 'Name', 'Covariance')
subplot(3,1,1)
plot(cov_2(1:18000,1:3))
hold on
plot(0,cov_2(1,1),cov_2(1,2),cov_2(1,3),'r*')
grid on
legend('pos x','pos y','pos z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
xlim([-1000 18000]);
title( 'Position Covariance' ) ;
subplot(3,1,2)
plot(cov_2(1:18000,4:6))
hold on
plot(0,cov_2(1,4),cov_2(1,5),cov_2(1,6),'r*')
xlim([-1000 18000]);
ylim([0 0.1]);
grid on
legend('vel x','vel y','vel z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('covariance') % y-axis label
title( 'Velocity Covariance' ) ;
% subplot(3,1,3)
% plot(cov_2(1:18000,7:9))
% xlim([-1000 18000]);
% hold on
% plot(0,cov_2(1,7),cov_2(1,8),cov_2(1,9),'r*')
% grid on
% legend('force x','force y','force z','Location','NorthEastOutside');
% xlabel('t - [simulation step]') % x-axis label
% ylabel('covariance') % y-axis label
% title( 'Correction Forces  Covariance' ) ;
suptitle('Pos, Vel Q < R')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/cov_2.bmp')

figure('NumberTitle', 'off', 'Name', 'Innovation')
plot(inn_2(1:18000,:))
grid on
legend('Innovation x','Innovation y','Innovation z','Location','NorthEastOutside');
xlabel('t - [simulation step]') % x-axis label
ylabel('innovation - [m]') % y-axis label
title({'Pos, Vel Q < R','Innovation - Difference between predicted position and observed position'} ) ;
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/inn_2.bmp')

 
size_F = size(f);
size_G = size(g);

F=f(1:18000,2:4);
G=g(1:18000,2:4);

H=zeros(size(F,1),1);
for i = 1:(size(F,1))
   H(i) = HausdorffDist(F(i,:),G(i,:));
end
E=find(H<0.001);

figure('NumberTitle', 'off', 'Name', 'Hausdorff Distance')
plot(H(1:100:size(F,1)))
grid on
title({'Pos, Vel Q < R','Hausdorff Distance between Fiter Beam and Ground Truth Beam '})
xlabel('t - [simulation step]') % x-axis label
ylabel('Hausdorff Distance - [m]') % y-axis label
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 60 30]); %x_width=10cm y_width=15cm
% saveas(gcf,'print/haus_2.bmp')

d=zeros(18000,2);
d(:,1)=s_2(1:18000,1);
for i=1:size(d,1)
    d(i,2)=-0;
end
% figure('NumberTitle', 'off', 'Name', 'parameter')
% plot(s_2(1:18000,7:9))
% grid on
% hold on
% plot(d(1:18000,2))
% legend('force x','force y','force z','Expected Force Null','Location','NorthEastOutside');
% title({'Pos, Vel Q < R','Correction Forces Magnitude'})
% xlabel('t - [simulation step]') % x-axis label
% ylabel('magnitude - [m]') % y-axis label