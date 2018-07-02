clear 
close all
clc



load TOP_hd_1.mat; load TOP_hd_3.mat;
load SIDE_hd_0.mat; load SIDE_hd_3.mat
load OB_hd_2.mat; load OB_hd_3.mat;
% 
% imgbif_ob = imread('bif_ob.png');
% imgbif_side = imread('bif_side.png');
% imgbif_top = imread('bif_top.png');
% 
% imgtest_top = imread('test_top.png');
% imgtest_ob = imread('bif_ob.png');
% imgtest_side = imread('test_side.png');
% 


TOP_A=cat(2,TOP_hd_1,TOP_hd_3);
TOP_M=mean(TOP_A,2);
TOP_Mst=ones(size(TOP_A,1),1);

for i=1:size(TOP_A,1);
    TOP_Mst(i)=std(TOP_A(i,:));
end

SIDE_A=cat(2,SIDE_hd_0,SIDE_hd_3);
SIDE_M=mean(SIDE_A,2);
SIDE_Mst=ones(size(SIDE_A,1),1);

for i=1:size(SIDE_A,1);
    SIDE_Mst(i)=std(SIDE_A(i,:));
end

OB_A=cat(2,OB_hd_2,OB_hd_3);
OB_M=mean(OB_A,2);
OB_Mst=ones(size(OB_A,1),1);

for i=1:size(OB_A,1);
    OB_Mst(i)=std(OB_A(i,:));
end

h=figure
errorbar(TOP_M(1:50:end),TOP_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[1 0 0])
grid on
grid minor
hold on
errorbar(SIDE_M(1:50:end),SIDE_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[0.0784313753247261 0.168627455830574 0.549019634723663])
hold on
errorbar(OB_M(1:50:end),OB_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[0.466666668653488 0.674509823322296 0.18823529779911])
legend('Top View ','Side View','Oblique View','Location','northeast')
set(gca,'XTickLabel',[0 250 500 750 1000 1250 1500])
xlabel('h - [simulation step]') % x-axis label
ylabel('hd  [m] ') % y-axis label
title( {'{\it Bifurcation} - Mean Haussdorf Distance for different views'; '3D Ground Truth Catheter - 3D  Filter Catheter' }) ;
set(h,'Position',[50 50 1200 800]);
set(h,'PaperOrientation','landscape');
pos = get(h,'Position');
print(h,'bif3Dhdmean','-dpdf','-r0')

%%% TEST SCENE %%




load TEST_TOP_hd_0.mat; load TEST_TOP_hd_1.mat; load TEST_TOP_hd_2.mat; load TEST_TOP_hd_3.mat;
load TEST_SIDE_hd_0.mat;load TEST_SIDE_hd_1.mat; load TEST_SIDE_hd_2.mat; load TEST_SIDE_hd_3.mat
load TEST_OB_hd_0.mat; load TEST_OB_hd_1.mat; load TEST_OB_hd_2.mat; load TEST_OB_hd_3.mat;

TOP_A=cat(2,TEST_TOP_hd_0,TEST_TOP_hd_1,TEST_TOP_hd_2,TEST_TOP_hd_3);
TOP_M=mean(TOP_A,2);
TOP_Mst=ones(size(TOP_A,1),1);

for i=1:size(TOP_A,1);
    TOP_Mst(i)=std(TOP_A(i,:));
end

SIDE_A=cat(2,TEST_SIDE_hd_0,TEST_SIDE_hd_1,TEST_SIDE_hd_2,TEST_SIDE_hd_3);
SIDE_M=mean(SIDE_A,2);
SIDE_Mst=ones(size(SIDE_A,1),1);

for i=1:size(SIDE_A,1);
    SIDE_Mst(i)=std(SIDE_A(i,:));
end

OB_A=cat(2,TEST_OB_hd_0,TEST_OB_hd_1,TEST_OB_hd_2,TEST_OB_hd_3);
OB_M=mean(OB_A,2);
OB_Mst=ones(size(OB_A,1),1);

for i=1:size(OB_A,1);
    OB_Mst(i)=std(OB_A(i,:));
end

h2=figure
errorbar(TOP_M(1:50:end),TOP_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[1 0 0])
grid on
grid minor
hold on
errorbar(SIDE_M(1:50:end),SIDE_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[0.0784313753247261 0.168627455830574 0.549019634723663])
hold on
errorbar(OB_M(1:50:end),OB_Mst(1:50:end),'MarkerFaceColor',[0 0 0],...
    'Marker','diamond','LineWidth',1.5,'Color',[0.466666668653488 0.674509823322296 0.18823529779911])
legend('Top View ','Side View','Oblique View','Location','northeast')
set(gca,'XTickLabel',[0 500 1000 1500 2000 2500 3000])
xlabel('h - [simulation step]') % x-axis label
ylabel('Haussdorff Distance [m] ') % y-axis label
title( {'{\it Simple Vessel} - Mean Haussdorf Distance for different views'; '3D Ground Truth Catheter - 3D  Filter Catheter' }) ;
set(h2,'Position',[50 50 1200 800]);
set(h2,'PaperOrientation','landscape');
pos = get(h2,'Position');
print(h2,'simple3DHDmean','-dpdf','-r0')

