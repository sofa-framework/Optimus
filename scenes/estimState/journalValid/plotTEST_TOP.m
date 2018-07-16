clear 
% close all 
clc

load d2d_TEST_TOP_0.mat
load d2d_TEST_TOP_1.mat   
load d2d_TEST_TOP_2.mat
load d2d_TEST_TOP_3.mat

load d_TEST_TOP_0.mat
load d_TEST_TOP_1.mat
load d_TEST_TOP_2.mat
load d_TEST_TOP_3.mat

load hd_TEST_TOP_0.mat
load hd_TEST_TOP_1.mat
load hd_TEST_TOP_2.mat
load hd_TEST_TOP_3.mat

load hd2d_TEST_TOP_0.mat 
load hd2d_TEST_TOP_1.mat 
load hd2d_TEST_TOP_2.mat 
load hd2d_TEST_TOP_3.mat 

load RMSE2d_TEST_TOP_0.mat 
load RMSE2d_TEST_TOP_1.mat 
load RMSE2d_TEST_TOP_2.mat 
load RMSE2d_TEST_TOP_3.mat 


load RMSE_TEST_TOP_0.mat 
load RMSE_TEST_TOP_1.mat 
load RMSE_TEST_TOP_2.mat 
load RMSE_TEST_TOP_3.mat 


figure
plot(d_TEST_TOP_0*1000,'LineWidth',1.5) 
hold on
plot(d_TEST_TOP_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(d_TEST_TOP_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(d_TEST_TOP_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('dist [mm] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'Distance 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('d_TEST_TOP.fig')





figure
plot(d2d_TEST_TOP_0,'LineWidth',1.5) 
hold on
plot(d2d_TEST_TOP_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) 
hold on
plot(d2d_TEST_TOP_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(d2d_TEST_TOP_3,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('dist [pixel] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'Distance 2D Ground Truth Tip - 2D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('d2d_TEST_TOP.fig')



figure
plot(hd_TEST_TOP_0*1000,'LineWidth',1.5) 
hold on
plot(hd_TEST_TOP_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(hd_TEST_TOP_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd_TEST_TOP_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Haussdorf [mm] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'Haussdorf Distance Ground Truth Catheter - Filter Catheter'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('hd_TEST_TOP.fig')

figure
plot(hd2d_TEST_TOP_0,'LineWidth',1.5) 
hold on
plot(hd2d_TEST_TOP_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(hd2d_TEST_TOP_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd2d_TEST_TOP_3, 'LineWidth',1.5 , 'Color',[0.87058824300766 0.490196079015732 0])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('Haussdorf [pixel] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'Haussdorf Distance 2D  Ground Truth Catheter -  2D Filter Catheter'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('hd2d_TEST_TOP.fig')



figure
plot(RMSE_TEST_TOP_0*1000,'LineWidth',1.5) 
hold on
plot(RMSE_TEST_TOP_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(RMSE_TEST_TOP_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(RMSE_TEST_TOP_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'RMSE Ground Truth Tip - Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('RMSE_TEST_TOP.fig')

figure
plot(RMSE2d_TEST_TOP_0,'LineWidth',1.5) 
hold on
plot(RMSE2d_TEST_TOP_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(RMSE2d_TEST_TOP_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(RMSE2d_TEST_TOP_3, 'LineWidth',1.5 , 'Color',[0.87058824300766 0.490196079015732 0])
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'{\it Simple Vessel Top View}';'RMSE 2D  Ground Truth Tip -  2D Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('RMSE2d_TEST_TOP.fig')
