clear
close all 
clc

load d2d_BIF_OB_0.mat; load d2d_BIF_OB_1.mat ; load d2d_BIF_OB_2.mat; load d2d_BIF_OB_3.mat
load d_BIF_OB_0.mat; load d_BIF_OB_1.mat; load d_BIF_OB_2.mat; load d_BIF_OB_3.mat
load hd_BIF_OB_0.mat; load hd_BIF_OB_1.mat; load hd_BIF_OB_2.mat; load hd_BIF_OB_3.mat;
load hd2d_BIF_OB_0.mat; load hd2d_BIF_OB_1.mat; load hd2d_BIF_OB_2.mat; load hd2d_BIF_OB_3.mat ;

figure
plot(d_BIF_OB_0*1000,'LineWidth',1.5) %blu
hold on
plot(d_BIF_OB_1*1000,'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso
hold on
plot(d_BIF_OB_2*1000,'LineWidth',1.5, 'Color',[0 0.600000023841858 0]) %verde
hold on
plot(d_BIF_OB_3*1000,'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('dist [mm] ') % y-axis label
title( {'{\it Bifurcation Oblique View}' ;'Distance 3D Ground Truth Tip - 3D  Filter Tip'  }) 
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif3Ddist_OB.fig')


figure(6)
plot(d2d_BIF_OB_0,'LineWidth',1.5)
hold on
plot(d2d_BIF_OB_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso 
hold on
plot(d2d_BIF_OB_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
%verde 
hold on
plot(d2d_BIF_OB_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0]) %giallo
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('dist [pixel] ') % y-axis label
title( {'{\it Bifurcation Oblique View}'; ' Distance  2D Ground Truth Tip - 2D  Filter Tip' }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2Ddist_OB.fig')


figure
plot(hd_BIF_OB_0*1000,'LineWidth',1.5)
hold on
plot(hd_BIF_OB_1*1000, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(hd_BIF_OB_2*1000, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd_BIF_OB_3*1000, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
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
plot(hd2d_BIF_OB_0,'LineWidth',1.5)
hold on
plot(hd2d_BIF_OB_1, 'LineWidth',1.5, 'Color',[0.600000023841858 0 0]) %rosso)
hold on
plot(hd2d_BIF_OB_2, 'LineWidth',1.5, 'Color',[0 0.600000023841858 0])
hold on
plot(hd2d_BIF_OB_3, 'LineWidth',1.5, 'Color',[0.87058824300766 0.490196079015732 0])
grid on
grid minor
xlabel('h - [simulation step]') % x-axis label
ylabel('Hausdorff [pixel] ') % y-axis label
title( {'{\it Bifurcation Oblique View}';'Hausdorff Distance 2D Ground Truth Catheter- 2D Filter Catheter'  }) ;
ax = gca;
ax.TitleFontSizeMultiplier = 1;
legend('Parameters Set 1','Parameters Set 2','Parameters Set 3','Parameters Set 4' );
savefig('bif2DHD_OB.fig')

