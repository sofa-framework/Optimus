clear 
close all
clc



load RMSE_BIF_TOP_1.mat; load RMSE_BIF_TOP_3.mat;
load RMSE_BIF_SIDE_0.mat; load RMSE_BIF_SIDE_3.mat
load RMSE_BIF_OB_1.mat; load RMSE_BIF_OB_3.mat;

% load RMSE_TEST_TOP_1.mat; load RMSE_TEST_TOP_3.mat;
% load RMSE_TEST_SIDE_1.mat; load RMSE_TEST_SIDE_3.mat
% load RMSE_TEST_OB_0.mat; load RMSE_TEST_OB_2.mat;

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

OB_A=cat(2,RMSE_BIF_OB_1,RMSE_BIF_OB_3);
OB_M=mean(OB_A,2);
OB_Mst=ones(size(OB_A,1),1);

for i=1:size(OB_A,1);
    OB_Mst(i)=std(OB_A(i,:));
end


TEST_TOP_RMSE=mean(TOP_M);
TEST_TOP_RMSEstd=mean(TOP_Mst);
TEST_OB_RMSE=mean(OB_M);
TEST_OB_RMSEstd=mean(OB_Mst);
TEST_SIDE_RMSE=mean(SIDE_M);
TEST_SIDE_RMSEstd=mean(SIDE_Mst);