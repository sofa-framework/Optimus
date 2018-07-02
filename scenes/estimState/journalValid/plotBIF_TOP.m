%%PLOT TIP TOP_RMSE 
% load('TOP_RMSE2d_0.mat','TOP_RMSE2d_0')
clear all
% close all 
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
load TEST_TOP_n.mat 
P=[775.552 -206.837 200.739 202.551;43.6739 289.403 727.253 109.588;0.148221 -0.711592 0.68678 0.348575]; %BIFTOP


figure
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
title( {'{\it Bifurcation Top View}';'Root Mean Square Error 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DRMSE_TOP.fig')

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
title( {'{\it Bifurcation Top View}'; ' Root Mean Square Error  2D Ground Truth Tip - 2D  Filter Tip' }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DRMSE_TOP.fig')

figure
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
title( {'{\it Bifurcation Top View} ';'Hausdorff Distance Ground Truth Catheter - Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DHD_TOP.fig')


figure
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
title( {'{\it Bifurcation Top View}';'Hausdorff Distance 2D Ground Truth Catheter- 2D Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DHD_TOP.fig')
