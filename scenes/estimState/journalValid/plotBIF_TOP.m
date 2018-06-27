%%PLOT TIP TOP_RMSE 
% load('TOP_RMSE2d_0.mat','TOP_RMSE2d_0')
% clear all
% close all 
% clc

load TOP_RMSE2d_0.mat
load TOP_RMSE2d_1.mat   
load TOP_RMSE2d_2.mat
load TOP_RMSE2d_3.mat

load TOP_RMSE_0.mat
load TOP_RMSE_1.mat
load TOP_RMSE_2.mat
load TOP_RMSE_3.mat

load TOP_hd_0.mat
load TOP_hd_1.mat
load TOP_hd_2.mat
load TOP_hd_3.mat

load TOP_hd2d_0.mat 
load TOP_hd2d_1.mat 
load TOP_hd2d_2.mat 
load TOP_hd2d_3.mat 

figure
plot(TOP_RMSE_0*1000)
hold on
plot(TOP_RMSE_1*1000, 'Color',[1 0 0])
hold on
plot(TOP_RMSE_2*1000, 'Color',[0 0.8 0])
hold on
plot(TOP_RMSE_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('TOP_RMSE [mm] ') % y-axis label
title( {'Root Mean Square Error'; ' 3D Ground Truth Tip - 3D  Filter Tip' }) ;

figure
plot(TOP_RMSE2d_0)
hold on
plot(TOP_RMSE2d_1, 'Color',[1 0 0]) 
hold on
plot(TOP_RMSE2d_2, 'Color',[0 0.8 0])
hold on
plot(TOP_RMSE2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('TOP_RMSE [pixel] ') % y-axis label
title( {'2D Root Mean Square Error'; ' 2D Ground Truth Tip - 2D  Filter Tip' }) ;


figure
plot(TOP_hd_0*1000)
hold on
plot(TOP_hd_1*1000, 'Color',[1 0 0])
hold on
plot(TOP_hd_2*1000, 'Color',[0 0.8 0])
hold on
plot(TOP_hd_3*1000, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'Hausdorff Distance'; 'Ground Truth Catheter- Filter Catheter' }) ;

figure
plot(TOP_hd2d_0)
hold on
plot(TOP_hd2d_1, 'Color',[1 0 0])
hold on
plot(TOP_hd2d_2, 'Color',[0 0.8 0])
hold on
plot(TOP_hd2d_3, 'Color',[1 0 1])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'2D Hausdorff Distance'; '2D  Ground Truth Catheter-  2D Filter Catheter' }) ;
