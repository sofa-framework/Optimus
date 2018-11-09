clear 
close all

load('h_mu_20.mat','h_mu_20');
load('h_Wmu_20.mat','h_Wmu_20');
load('RMSE_mu_20.mat','RMSE_mu_20');
load('RMSE_Wmu_20.mat','RMSE_Wmu_20');
load('d_mu_20.mat','d_mu_20');
load('d_Wmu_20.mat','d_Wmu_20');

t=0:1999;


%% PLOT HAUSDORFF DISTANCE 

h_mean_ORI_Q1=mean(cat(2,h_mu_20(1:2000,1)*1000,h_mu_20(1:2000,4)*1000,h_mu_20(1:2000,7)*1000),2);
h_mean_ORI_Q2=mean(cat(2,h_mu_20(1:2000,2)*1000,h_mu_20(1:2000,5)*1000,h_mu_20(1:2000,8)*1000),2);
h_mean_ORI_Q3=mean(cat(2,h_mu_20(1:2000,3)*1000,h_mu_20(1:2000,6)*1000,h_mu_20(1:2000,9)*1000),2);
h_mean_TOP_Q1=mean(cat(2,h_mu_20(1:2000,10)*1000,h_mu_20(1:2000,13)*1000,h_mu_20(1:2000,16)*1000),2);
h_mean_TOP_Q2=mean(cat(2,h_mu_20(1:2000,11)*1000,h_mu_20(1:2000,14)*1000,h_mu_20(1:2000,17)*1000),2);
h_mean_TOP_Q3=mean(cat(2,h_mu_20(1:2000,12)*1000,h_mu_20(1:2000,15)*1000,h_mu_20(1:2000,18)*1000),2);

%%% ORI
figure(1)
plot(t,h_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,h_mu_20(1:2000,1),'r',t,h_mu_20(1:2000,4),'r',t,h_mu_20(1:2000,7),'r')
plot(t,h_mean_ORI_Q1,'r')
hold on
% plot(t,h_mu_20(1:2000,2),'b',t,h_mu_20(1:2000,5),'b',t,h_mu_20(1:2000,8),'b')
plot(t,h_mean_ORI_Q2,'b')
hold on
% plot(t,h_mu_20(1:2000,3),'g',t,h_mu_20(1:2000,6),'g',t,h_mu_20(1:2000,9),'g')
plot(t,h_mean_ORI_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
hold off
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel('Hausdorf Distance [mm]')
title({'Hausdorff Distance','Observations from ORIZONTAL VIEW','Friction Coefficient μ=0.044'});


%%%TOP --> small uncertainty provides better results because from TOP view there's more information
figure(2)
plot(t,h_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,h_mu_20(1:2000,10),'r',t,h_mu_20(1:2000,13),'r',t,h_mu_20(1:2000,16),'r')
plot(t,h_mean_TOP_Q1,'r')
hold on
% plot(t,h_mu_20(1:2000,11),'b',t,h_mu_20(1:2000,14),'b',t,h_mu_20(1:2000,17),'b') 
plot(t,h_mean_TOP_Q2,'b')
hold on
plot(t,h_mean_TOP_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel('Hausdorf Distance [mm]')
% plot(t,h_mu_20(1:2000,12),'g',t,h_mu_20(1:2000,15),'g',t,h_mu_20(1:2000,18),'g')
title({'Hausdorff Distance','Observations from TOP VIEW','Friction Coefficient μ=0.044'});



%% PLOT RMSE TIP

RMSE_mean_ORI_Q1=mean(cat(2,RMSE_mu_20(1:2000,1)*1000,RMSE_mu_20(1:2000,4)*1000,RMSE_mu_20(1:2000,7)*1000),2);
RMSE_mean_ORI_Q2=mean(cat(2,RMSE_mu_20(1:2000,2)*1000,RMSE_mu_20(1:2000,5)*1000,RMSE_mu_20(1:2000,8)*1000),2);
RMSE_mean_ORI_Q3=mean(cat(2,RMSE_mu_20(1:2000,3)*1000,RMSE_mu_20(1:2000,6)*1000,RMSE_mu_20(1:2000,9)*1000),2);
RMSE_mean_TOP_Q1=mean(cat(2,RMSE_mu_20(1:2000,10)*1000,RMSE_mu_20(1:2000,13)*1000,RMSE_mu_20(1:2000,16)*1000),2);
RMSE_mean_TOP_Q2=mean(cat(2,RMSE_mu_20(1:2000,11)*1000,RMSE_mu_20(1:2000,14)*1000,RMSE_mu_20(1:2000,17)*1000),2);
RMSE_mean_TOP_Q3=mean(cat(2,RMSE_mu_20(1:2000,12)*1000,RMSE_mu_20(1:2000,15)*1000,RMSE_mu_20(1:2000,18)*1000),2);

%%% ORI
figure(3)
plot(t,RMSE_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,RMSE_mu_20(1:2000,1),'r',t,RMSE_mu_20(1:2000,4),'r',t,RMSE_mu_20(1:2000,7),'r')
plot(t,RMSE_mean_ORI_Q1,'r')
hold on
% plot(t,RMSE_mu_20(1:2000,2),'b',t,RMSE_mu_20(1:2000,5),'b',t,RMSE_mu_20(1:2000,8),'b')
plot(t,RMSE_mean_ORI_Q2,'b')
hold on
% plot(t,RMSE_mu_20(1:2000,3),'g',t,RMSE_mu_20(1:2000,6),'g',t,RMSE_mu_20(1:2000,9),'g')
plot(t,RMSE_mean_ORI_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
hold off
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel('RMSE [mm]')
title({'RMSE at the TIP','Observations from ORIZONTAL VIEW','Friction Coefficient μ=0.044'});


%%%TOP --> small uncertainty provides better results because from TOP view tRMSEere's more information
figure(4)
plot(t,RMSE_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,RMSE_mu_20(1:2000,10),'r',t,RMSE_mu_20(1:2000,13),'r',t,RMSE_mu_20(1:2000,16),'r')
plot(t,RMSE_mean_TOP_Q1,'r')
hold on
% plot(t,RMSE_mu_20(1:2000,11),'b',t,RMSE_mu_20(1:2000,14),'b',t,RMSE_mu_20(1:2000,17),'b') 
plot(t,RMSE_mean_TOP_Q2,'b')
hold on
plot(t,RMSE_mean_TOP_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel(' RMSE[mm]')
% plot(t,h_mu_20(1:2000,12),'g',t,h_mu_20(1:2000,15),'g',t,h_mu_20(1:2000,18),'g')
title({'RMSE at the TIP','Observations from TOP VIEW','Friction Coefficient μ=0.044'});

%% PLOT TIP Mean Distance

d_mean_ORI_Q1=mean(cat(2,d_mu_20(1:2000,1)*1000,d_mu_20(1:2000,4)*1000,d_mu_20(1:2000,7)*1000),2);
d_mean_ORI_Q2=mean(cat(2,d_mu_20(1:2000,2)*1000,d_mu_20(1:2000,5)*1000,d_mu_20(1:2000,8)*1000),2);
d_mean_ORI_Q3=mean(cat(2,d_mu_20(1:2000,3)*1000,d_mu_20(1:2000,6)*1000,d_mu_20(1:2000,9)*1000),2);
d_mean_TOP_Q1=mean(cat(2,d_mu_20(1:2000,10)*1000,d_mu_20(1:2000,13)*1000,d_mu_20(1:2000,16)*1000),2);
d_mean_TOP_Q2=mean(cat(2,d_mu_20(1:2000,11)*1000,d_mu_20(1:2000,14)*1000,d_mu_20(1:2000,17)*1000),2);
d_mean_TOP_Q3=mean(cat(2,d_mu_20(1:2000,12)*1000,d_mu_20(1:2000,15)*1000,d_mu_20(1:2000,18)*1000),2);

%%% ORI
figure(5)
plot(t,d_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,d_mu_20(1:2000,1),'r',t,d_mu_20(1:2000,4),'r',t,d_mu_20(1:2000,7),'r')
plot(t,d_mean_ORI_Q1,'r')
hold on
% plot(t,d_mu_20(1:2000,2),'b',t,d_mu_20(1:2000,5),'b',t,d_mu_20(1:2000,8),'b')
plot(t,d_mean_ORI_Q2,'b')
hold on
% plot(t,d_mu_20(1:2000,3),'g',t,d_mu_20(1:2000,6),'g',t,d_mu_20(1:2000,9),'g')
plot(t,d_mean_ORI_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
hold off
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel('Mean Distance  [mm]')
title({'Tip Mean Distance ','Observations from ORIZONTAL VIEW','Friction Coefficient μ=0.044'});


%%%TOP --> small uncertainty provides better results because from TOP view tdere's more information
figure(6)
plot(t,d_Wmu_20(1:2000)*1000,'k--');
hold on
% plot(t,d_mu_20(1:2000,10),'r',t,d_mu_20(1:2000,13),'r',t,d_mu_20(1:2000,16),'r')
plot(t,d_mean_TOP_Q1,'r')
hold on
% plot(t,d_mu_20(1:2000,11),'b',t,d_mu_20(1:2000,14),'b',t,d_mu_20(1:2000,17),'b') 
plot(t,d_mean_TOP_Q2,'b')
hold on
plot(t,d_mean_TOP_Q3,'g')
legend('Wrong model ','Filter Correction Q_0','Filter Correction Q_{<<}','Filter Correction Q_{>>}')
grid(gca,'minor')
grid on
hold off
set(gca,'XTickLabel',[])
xlabel('T')
ylabel(' Mean Distance [mm]')
% plot(t,h_mu_20(1:2000,12),'g',t,h_mu_20(1:2000,15),'g',t,h_mu_20(1:2000,18),'g')
title({'Tip Mean Distance ','Observations from TOP VIEW','Friction Coefficient μ=0.044'});
