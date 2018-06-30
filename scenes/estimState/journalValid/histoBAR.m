clear 
close all
clc


load TOP_RMSE_0.mat
load TOP_RMSE_1.mat   
load TOP_RMSE_2.mat
load TOP_RMSE_3.mat


load SIDE_RMSE_0.mat
load SIDE_RMSE_1.mat   
load SIDE_RMSE_2.mat
load SIDE_RMSE_3.mat

load OB_RMSE_0.mat
load OB_RMSE_1.mat   
load OB_RMSE_2.mat
load OB_RMSE_3.mat


TOP_A=cat(2,TOP_RMSE_0,TOP_RMSE_1,TOP_RMSE_2,TOP_RMSE_3);
TOP_M=mean(TOP_A,2);
TOP_Mst=ones(size(TOP_A,1),1);

for i=1:size(TOP_A,1);
    TOP_Mst(i)=std(TOP_A(i,:));
end


SIDE_A=cat(2,SIDE_RMSE_0,SIDE_RMSE_1,SIDE_RMSE_2,SIDE_RMSE_3);
SIDE_M=mean(SIDE_A,2);
SIDE_Mst=ones(size(SIDE_A,1),1);

for i=1:size(SIDE_A,1);
    SIDE_Mst(i)=std(SIDE_A(i,:));
end


OB_A=cat(2,OB_RMSE_0,OB_RMSE_1,OB_RMSE_2,OB_RMSE_3);
OB_M=mean(OB_A,2);
OB_Mst=ones(size(OB_A,1),1);

for i=1:size(OB_A,1);
    OB_Mst(i)=std(OB_A(i,:));
end

figure
errorbar(TOP_M(1:50:end),TOP_Mst(1:50:end),'*-')
grid on
hold on
errorbar(SIDE_M(1:50:end),SIDE_Mst(1:50:end),'*-')
hold on
errorbar(OB_M(1:50:end),OB_Mst(1:50:end),'*-')
legend('Top View ','Side View','Oblique View','Location','northeast')
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [pixel] ') % y-axis label
title( {'Mean value of RMSE for different views'; 'RMSE 3D Ground Truth Tip - 3D  Filter Tip' }) ;

