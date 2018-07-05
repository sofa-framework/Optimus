clear
close all 
clc

load d2d_BIF_SIDE_0.mat
load d2d_BIF_SIDE_1.mat   
load d2d_BIF_SIDE_2.mat
load d2d_BIF_SIDE_3.mat

load d_BIF_SIDE_0.mat
load d_BIF_SIDE_1.mat
load d_BIF_SIDE_2.mat
load d_BIF_SIDE_3.mat

load hd_BIF_SIDE_0.mat
load hd_BIF_SIDE_1.mat
load hd_BIF_SIDE_2.mat
load hd_BIF_SIDE_3.mat

load hd2d_BIF_SIDE_0.mat 
load hd2d_BIF_SIDE_1.mat 
load hd2d_BIF_SIDE_2.mat 
load hd2d_BIF_SIDE_3.mat 

figure
plot(d_BIF_SIDE_0*1000 ,'LineWidth',1.5)
hold on
plot(d_BIF_SIDE_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(d_BIF_SIDE_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(d_BIF_SIDE_3*1000, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'{\it Bifurcation Side View}';'Distance 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DRMSE_SIDE.fig')



figure
plot(d2d_BIF_SIDE_0, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(d2d_BIF_SIDE_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) 
hold on
plot(d2d_BIF_SIDE_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(d2d_BIF_SIDE_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor

xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'{\it Bifurcation Side View}'; ' Distance  2D Ground Truth Tip - 2D  Filter Tip' }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DRMSE_SIDE.fig')


figure
plot(hd_BIF_SIDE_0*1000 ,'LineWidth',1.5)
hold on
plot(hd_BIF_SIDE_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(hd_BIF_SIDE_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd_BIF_SIDE_3*1000, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor

xlabel('h - [simulation step]') % x-axis label
ylabel('Haussdorf [mm] ') % y-axis label
title( {'{\it Bifurcation Side View} ';'Haussdorf Distance Ground Truth Catheter - Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DHD_SIDE.fig')


figure
plot(hd2d_BIF_SIDE_0,'LineWidth',1.5)
hold on
plot(hd2d_BIF_SIDE_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0])
hold on
plot(hd2d_BIF_SIDE_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd2d_BIF_SIDE_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor

xlabel('h - [simulation step]') % x-axis label
ylabel('Haussdorf [pixel] ') % y-axis label
title( {'{\it Bifurcation Side View}';'Haussdorf Distance 2D Ground Truth Catheter- 2D Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DHD_SIDE.fig')
