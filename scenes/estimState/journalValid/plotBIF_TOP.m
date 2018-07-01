%%PLOT TIP TOP_RMSE 
% load('TOP_RMSE2d_0.mat','TOP_RMSE2d_0')
clear all
close all 
clc

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

figure(5)
plot(TOP_RMSE_0*1000,'LineWidth',1.5) %blu
hold on
plot(TOP_RMSE_1*1000,'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso
hold on
plot(TOP_RMSE_2*1000,'LineWidth',1.5, 'Color',[0 0.600000023841858 0]) %verde
hold on
plot(TOP_RMSE_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {' 3D Ground Truth Tip - 3D  Filter Tip';'Root Mean Square Error'  }) ;

figure(6)
plot(TOP_RMSE2d_0,'LineWidth',1.5)
hold on
plot(TOP_RMSE2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso 
hold on
plot(TOP_RMSE2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
%verde 
hold on
plot(TOP_RMSE2d_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'2D Ground Truth Tip - 2D  Filter Tip'; ' Root Mean Square Error' }) ;


figure(7)
plot(TOP_hd_0*1000,'LineWidth',1.5)
hold on
plot(TOP_hd_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(TOP_hd_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TOP_hd_3*1000, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'Ground Truth Catheter - Filter Catheter';'Hausdorff Distance'  }) ;

figure(8)
plot(TOP_hd2d_0,'LineWidth',1.5)
hold on
plot(TOP_hd2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(TOP_hd2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TOP_hd2d_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'2D  Ground Truth Catheter-  2D Filter Catheter';'Hausdorff Distance' }) ;
