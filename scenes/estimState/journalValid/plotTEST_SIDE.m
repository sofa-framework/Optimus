clear 
close all 
clc

load TEST_SIDE_RMSE2d_0.mat
load TEST_SIDE_RMSE2d_1.mat   
load TEST_SIDE_RMSE2d_2.mat
load TEST_SIDE_RMSE2d_3.mat

load TEST_SIDE_RMSE_0.mat
load TEST_SIDE_RMSE_1.mat
load TEST_SIDE_RMSE_2.mat
load TEST_SIDE_RMSE_3.mat

load TEST_SIDE_hd_0.mat
load TEST_SIDE_hd_1.mat
load TEST_SIDE_hd_2.mat
load TEST_SIDE_hd_3.mat

load TEST_SIDE_hd2d_0.mat 
load TEST_SIDE_hd2d_1.mat 
load TEST_SIDE_hd2d_2.mat 
load TEST_SIDE_hd2d_3.mat 

figure
plot(TEST_SIDE_RMSE_0*1000,'LineWidth',1.5) 
hold on
plot(TEST_SIDE_RMSE_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(TEST_SIDE_RMSE_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TEST_SIDE_RMSE_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'{\it Simple Vessel Side View}';'Root Mean Square Error 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('simple3DRMSE_SIDE.fig')


figure
plot(TEST_SIDE_RMSE2d_0,'LineWidth',1.5) 
hold on
plot(TEST_SIDE_RMSE2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) 
hold on
plot(TEST_SIDE_RMSE2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TEST_SIDE_RMSE2d_3,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'{\it Simple Vessel Side View}';'Root Mean Square Error 2D Ground Truth Tip - 2D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('simple2DRMSE_SIDE.fig')


figure
plot(TEST_SIDE_hd_0*1000,'LineWidth',1.5) 
hold on
plot(TEST_SIDE_hd_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(TEST_SIDE_hd_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TEST_SIDE_hd_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'{\it Simple Vessel Side View}';'Hausdorff Distance Ground Truth Catheter - Filter Catheter'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('simple3DHD_SIDE.fig')



figure
plot(TEST_SIDE_hd2d_0,'LineWidth',1.5) 
hold on
plot(TEST_SIDE_hd2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(TEST_SIDE_hd2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(TEST_SIDE_hd2d_3, 'LineWidth',1.5 , 'Color',[0.87058824300766 0.490196079015732 0])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'{\it Simple Vessel Side View}';'Hausdorff Distance 2D  Ground Truth Catheter -  2D Filter Catheter'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('simple2DHD_SIDE.fig')

