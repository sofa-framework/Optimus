clear 
% close all
clc


state_mass0 = dlmread('print_state_mass');
inn_mass0 = dlmread('print_inn_mass');
cov_mass0 = dlmread('print_cov_mass');
g = dlmread('g');
G=g(:,2:end);

% figure;
% subplot(5,1,1);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,1:3))
% hold on
% plot(cov_mass0(1,1),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title({'{\itPosition & Velocity Estimation Qp=0 Qv<R}',' ',' ','Covariance of 1st Node  '}) ;
% subplot(5,1,2);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,4:6))
% hold on
% plot(cov_mass0(1,4),'o')
% hold off
% grid on
% ylabel('Covariance') % y-axis label
% legend('X','Y','Z', 'Location','NorthEastOutside');
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 2nd Node ') ;
% subplot(5,1,3);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,7:9))
% hold on
% plot(cov_mass0(1,7),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 3rd Node') ;
% subplot(5,1,4);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,10:12))
% hold on
% plot(cov_mass0(1,10),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 4th Node ') ;
% subplot(5,1,5);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,13:15))
% hold on
% plot(cov_mass0(1,13),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% title('Covariance of 5th Node ') ;
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Covariance') % y-axis label
% % axis([0 size(cov_mass0,1) 0 0.012]) 
% saveas(gcf, 'pos0_cov', 'bmp')
% grid on

% figure;
% subplot(5,1,1);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,16:18))
% hold on
% plot(cov_mass0(1,16),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title({'{\itPosition & Velocity Estimation Qp=0 Qv<R}',' ',' ','Covariance of 1st Node Velocity '}) ;
% subplot(5,1,2);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,19:21))
% hold on
% plot(cov_mass0(1,19),'o')
% hold off
% grid on
% ylabel('Covariance') % y-axis label
% legend('X','Y','Z', 'Location','NorthEastOutside');
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 2nd Node Velocity') ;
% subplot(5,1,3);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,22:24))
% hold on
% plot(cov_mass0(1,22),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 3rd Node Velocity') ;
% subplot(5,1,4);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,25:27))
% hold on
% plot(cov_mass0(1,25),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% title('Covariance of 4th Node Velocity') ;
% subplot(5,1,5);
% plot(cov_mass0(1:1:size(state_mass0,1)-1,28:30));
% hold on
% plot(cov_mass0(1,28),'o')
% hold off
% grid on
% legend('X','Y','Z', 'Location','NorthEastOutside');
% title('Covariance of 5th Node Velocity ') ;
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Covariance') % y-axis label
% % axis([-20 size(cov_mass0,1) 0.0000003 0.00000055]) 
% grid on
% 



% 
% H_mass0=zeros(size(state_mass0,1)-1,1);
% for i = 1:(size(state_mass0,1)-1)
%    H_mass0(i) = HausdorffDist(state_mass0(i,1:15),G(i,:));
% end
% %%
% figure;
% pp5 = splinefit(1:1:size(state_mass0,1)-1,H_mass0,8,1);  
% xx = linspace(1,size(state_mass0,1)-1);
% y5 = ppval(pp5,xx);
% ast=ones(size(H_mass0))*y5(end);
% plot(H_mass0(1:1:size(state_mass0,1)-1),'g')
% hold on
% plot(ast,'r'), 
% legend('Hausdorff Distance',['Average Dist =' num2str(ast(1))]);
% % axis([0 sze(cov_mass0,1) 0 0.012]) 
% grid on
% title({' {\itPosition & Velocity Estimation Qp=0 Qv<R}',' ',' ','Hausdorff Distance between Fiter Beam and Ground Truth Beam '})
% xlabel('t - [simulation step]') % x-axis label
% ylabel('Hausdorff Distance - [mm]') % y-axis label
% hold off

figure
plot(state_mass0(1:0.1:6000,26:29)), grid on;
hold on
plot(ones(60000,1)*0.2,'r--','Linewidth',3)
% plot(13500,-0.5,'bo')
legend(['Mass 2nd Node =' num2str(state_mass0(end,26))],...
    ['Mass 3rd Node =' num2str(state_mass0(end,27))],...
    ['Mass 4th Node =' num2str(state_mass0(end,28))],...
    ['Mass 5th Node=' num2str(state_mass0(end,29))],...
    ['Real Mass =' num2str(0.2)]);
axis([-20 6000 -0.05 1]) 
title({'Mass Estimation - Incorrect Model'})
xlabel({'t - [simulation step]'}) % x-axis label
ylabel('mass - [Kg]') % y-axis label

% figure
% plot(state_mass0(1:50:2000,1:3),'o--','Linewidth',1);
% hold on
% plot(zeros(size(state_mass0(1:50:2000,:),1)),'k','Linewidth',3);
% hold off
% title('Enforcment of fixed consraint at the base');
% legend('X','Y','Z','Fixed point X=0,Y=0,Z=0');
% 
% xlabel('t - [simulation step]') % x-axis label
% ylabel('displacement around P=(0,0,0)')
