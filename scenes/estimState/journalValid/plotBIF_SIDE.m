%%PLOT TIP RMSE 
% load('RMSE2d_0.mat','RMSE2d_0')
clear all
% close all 
clc

load SIDE_RMSE2d_0.mat
load SIDE_RMSE2d_1.mat   
load SIDE_RMSE2d_2.mat
load SIDE_RMSE2d_3.mat

load SIDE_RMSE_0.mat
load SIDE_RMSE_1.mat
load SIDE_RMSE_2.mat
load SIDE_RMSE_3.mat

load SIDE_hd_0.mat
load SIDE_hd_1.mat
load SIDE_hd_2.mat
load SIDE_hd_3.mat

load SIDE_hd2d_0.mat 
load SIDE_hd2d_1.mat 
load SIDE_hd2d_2.mat 
load SIDE_hd2d_3.mat 

figure
plot(SIDE_RMSE_0*1000)
hold on
plot(SIDE_RMSE_1*1000, 'Color',[1 0 0])
hold on
plot(SIDE_RMSE_2*1000, 'Color',[0 0.8 0])
hold on
plot(SIDE_RMSE_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {' 3D Ground Truth Tip - 3D  Filter Tip';'Root Mean Square Error'  }) ;


figure
plot(SIDE_RMSE2d_0, 'Color',[1 0 0])
hold on
plot(SIDE_RMSE2d_1, 'Color',[1 0 0]) 
hold on
plot(SIDE_RMSE2d_2, 'Color',[0 0.8 0])
hold on
plot(SIDE_RMSE2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'2D Ground Truth Tip - 2D  Filter Tip'; ' Root Mean Square Error' }) ;


figure
plot(SIDE_hd_0*1000)
hold on
plot(SIDE_hd_1*1000, 'Color',[1 0 0])
hold on
plot(SIDE_hd_2*1000, 'Color',[0 0.8 0])
hold on
plot(SIDE_hd_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'Ground Truth Catheter - Filter Catheter';'Hausdorff Distance'  }) ;

figure
plot(SIDE_hd2d_0)
hold on
plot(SIDE_hd2d_1, 'Color',[1 0 0])
hold on
plot(SIDE_hd2d_2, 'Color',[0 0.8 0])
hold on
plot(SIDE_hd2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'2D  Ground Truth Catheter-  2D Filter Catheter';'Hausdorff Distance' }) ;
