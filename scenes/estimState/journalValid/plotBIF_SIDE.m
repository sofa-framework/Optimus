%%PLOT TIP RMSE 
% load('RMSE2d_0.mat','RMSE2d_0')
% clear all
% close all 
% clc

load RMSE2d_0.mat
load RMSE2d_1.mat   
load RMSE2d_2.mat
load RMSE2d_3.mat

load RMSE_0.mat
load RMSE_1.mat
load RMSE_2.mat
load RMSE_3.mat

load hd_0.mat
load hd_1.mat
load hd_2.mat
load hd_3.mat

load hd2d_0.mat 
load hd2d_1.mat 
load hd2d_2.mat 
load hd2d_3.mat 

figure(1)
plot(RMSE_0*1000)
hold on
plot(RMSE_1*1000, 'Color',[1 0 0])
hold on
plot(RMSE_2*1000, 'Color',[0 0.8 0])
hold on
plot(RMSE_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'Root Mean Square Error'; ' 3D Ground Truth Tip - 3D  Filter Tip' }) ;

figure(2)
plot(RMSE2d_0, 'Color',[1 0 0])
hold on
plot(RMSE2d_1, 'Color',[1 0 0]) 
hold on
plot(RMSE2d_2, 'Color',[0 0.8 0])
hold on
plot(RMSE2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'2D Root Mean Square Error'; ' 2D Ground Truth Tip - 2D  Filter Tip' }) ;


figure(3)
plot(hd_0*1000)
hold on
plot(hd_1*1000, 'Color',[1 0 0])
hold on
plot(hd_2*1000, 'Color',[0 0.8 0])
hold on
plot(hd_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'Hausdorff Distance'; 'Ground Truth Catheter- Filter Catheter' }) ;

figure(4)
plot(hd2d_0)
hold on
plot(hd2d_1, 'Color',[1 0 0])
hold on
plot(hd2d_2, 'Color',[0 0.8 0])
hold on
plot(hd2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'2D Hausdorff Distance'; '2D  Ground Truth Catheter-  2D Filter Catheter' }) ;
