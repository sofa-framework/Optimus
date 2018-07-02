clear
close all 
clc

load OB_RMSE2d_0.mat; load OB_RMSE2d_1.mat ; load OB_RMSE2d_2.mat; load OB_RMSE2d_3.mat
load OB_RMSE_0.mat; load OB_RMSE_1.mat; load OB_RMSE_2.mat; load OB_RMSE_3.mat
load OB_hd_0.mat; load OB_hd_1.mat; load OB_hd_2.mat; load OB_hd_3.mat;
load OB_hd2d_0.mat; load OB_hd2d_1.mat; load OB_hd2d_2.mat; load OB_hd2d_3.mat ;

figure
plot(OB_RMSE_0*1000,'LineWidth',1.5) %blu
hold on
plot(OB_RMSE_1*1000,'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso
hold on
plot(OB_RMSE_2*1000,'LineWidth',1.5, 'Color',[0 0.600000023841858 0]) %verde
hold on
plot(OB_RMSE_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'{\it Bifurcation Oblique View}';'Root Mean Square Error 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DRMSE_OB.fig')


figure(6)
plot(OB_RMSE2d_0,'LineWidth',1.5)
hold on
plot(OB_RMSE2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso 
hold on
plot(OB_RMSE2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
%verde 
hold on
plot(OB_RMSE2d_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'{\it Bifurcation Oblique View}'; ' Root Mean Square Error  2D Ground Truth Tip - 2D  Filter Tip' }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DRMSE_OB.fig')


figure
plot(OB_hd_0*1000,'LineWidth',1.5)
hold on
plot(OB_hd_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(OB_hd_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(OB_hd_3*1000, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [mm] ') % y-axis label
title( {'{\it Bifurcation Oblique View} ';'Hausdorff Distance Ground Truth Catheter - Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3DHD_OB.fig')


figure
plot(OB_hd2d_0,'LineWidth',1.5)
hold on
plot(OB_hd2d_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(OB_hd2d_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(OB_hd2d_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'{\it Bifurcation Oblique View}';'Hausdorff Distance 2D Ground Truth Catheter- 2D Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DHD_OB.fig')


