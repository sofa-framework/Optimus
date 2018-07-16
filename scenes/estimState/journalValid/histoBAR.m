clear 
close all
clc



load RMSE_BIF_TOP_1.mat; load RMSE_BIF_TOP_3.mat;
load RMSE_BIF_SIDE_1.mat; load RMSE_BIF_SIDE_3.mat
load RMSE_BIF_OB_0.mat; load RMSE_BIF_OB_2.mat;

% load RMSE_TEST_TOP_1.mat; load RMSE_TEST_TOP_3.mat;
% load RMSE_TEST_SIDE_0.mat; load RMSE_TEST_SIDE_3.mat
% load RMSE_TEST_OB_1.mat; load RMSE_TEST_OB_3.mat;

TOP_A=cat(2,RMSE_BIF_TOP_1,RMSE_BIF_TOP_3);
TOP_M=mean(TOP_A,2);
TOP_Mst=ones(size(TOP_A,1),1);

for i=1:size(TOP_A,1);
    TOP_Mst(i)=std(TOP_A(i,:));
end

SIDE_A=cat(2,RMSE_BIF_SIDE_0,RMSE_BIF_SIDE_3);
SIDE_M=mean(SIDE_A,2);
SIDE_Mst=ones(size(SIDE_A,1),1);

for i=1:size(SIDE_A,1);
    SIDE_Mst(i)=std(SIDE_A(i,:));
end

OB_A=cat(2,RMSE_BIF_OB_3,RMSE_BIF_OB_1);
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
% Create line
% annotation(h,'line',[0.202156334231807 0.202156334231806],...
%     [0.926072927072927 0.115261472785486],...
%     'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],...
%     'LineWidth',2,...
%     'LineStyle','--');

annotation(h,'line',[0.296495956873317 0.296495956873316],...
    [0.922903354965162 0.112091900677721],...
    'Color',[0.850980401039124 0.325490206480026 0.0980392172932625],...
    'LineWidth',2,...
    'LineStyle','--');
legend('Top View ','Side View','Oblique View','Location','northeast')
set(gca,'XTickLabel',[0 250 500 750 1000 1250 1500])
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE  [m] ') % y-axis label
title( {'{\it Bifurcation} -Mean RMSE for different views'; '3D Ground Truth Tip - 3D  Filter Tip' }) ;
set(h,'Position',[50 50 1200 800]);
set(h,'PaperOrientation','landscape');
pos = get(h,'Position');
print(h,'bif3DRMSE_MEAN','-dpdf','-r0')
savefig('bif3DRMSE_MEAN.fig')


