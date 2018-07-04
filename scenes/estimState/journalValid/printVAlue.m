clear 
close all
clc

%load Ground Truth%
gtREAD_BIF_TOP= dlmread('groundTruth_BIF_TOP');
gtREAD_BIF_SIDE= dlmread('groundTruth_BIF_SIDE');
gtREAD_BIF_OB= dlmread('groundTruth_BIF_OB');
gtREAD_TEST_TOP= dlmread('groundTruth_TEST_TOP');
gtREAD_TEST_SIDE= dlmread('groundTruth_TEST_SIDE');
gtREAD_TEST_OB= dlmread('groundTruth_TEST_OB');

gt_BIF_TOP=gtREAD_BIF_TOP(:,2:end);
gt_BIF_OB=gtREAD_BIF_OB(:,2:end);
gt_BIF_SIDE=gtREAD_BIF_SIDE(:,2:end);
gt_TEST_TOP=gtREAD_TEST_TOP(:,2:end);
gt_TEST_OB=gtREAD_TEST_OB(:,2:end);
gt_TEST_SIDE=gtREAD_TEST_SIDE(:,2:end);

%load Projection matrix%
P_BIF_TOP=[775.552 -206.837 200.739 202.551;43.6739 289.403 727.253 109.588;0.148221 -0.711592 0.68678 0.348575]; %BIFSIDE
P_BIF_SIDE=[770.277 -32.7486 -300.267 225.406;217.307 -715.345 235.834 98.5108;0.194673 -0.612749 -0.765925 0.372637]; %BIFSIDE
P_BIF_OB=[513.41 148.368 631.63 215.592;-193.229 -585.209 484.511 25.2104;0.760325 -0.628768 0.16296 0.445612]; %BIFOB
P_TEST_TOP=[19.8421 -404.504 721.763 143.041;-708.31 -335.931 -2.3288 68.0737;0.0501423 -0.998723 -0.00620961 0.362366]; %TESTSIDE
P_TEST_OB=[560.532 -169.605 584.463 149.275;-87.0633 -776.497 63.4843 14.5066;0.827775 -0.499977 -0.254581 0.286321]; %TESTOB

P_TEST_OB_0=[560.946 -169.854 584.336 149.418;-87.0629 -776.497 63.4844 14.5067;0.827776 -0.499976 -0.25458 0.286322] %TESTOB_0
P_TEST_SIDE=[2281.11 -2.34947e-13 -400 273.957;-3.07529e-13 -2281.11 -300 -6.35711;-9.39116e-17 1.11022e-16 -1 0.317505]; %TESTSIDE



%load State%
%bifurcationscene
stateREAD_BIF_TOP_0= dlmread('matlab/print_state_BIF_TOP_0');
stateREAD_BIF_TOP_1= dlmread('matlab/print_state_BIF_TOP_1');
stateREAD_BIF_TOP_2= dlmread('matlab/print_state_BIF_TOP_2');
stateREAD_BIF_TOP_3= dlmread('matlab/print_state_BIF_TOP_3');

stateREAD_BIF_SIDE_0= dlmread('matlab/print_state_BIF_SIDE_0');
stateREAD_BIF_SIDE_1= dlmread('matlab/print_state_BIF_SIDE_1');
stateREAD_BIF_SIDE_2= dlmread('matlab/print_state_BIF_SIDE_2');
stateREAD_BIF_SIDE_3= dlmread('matlab/print_state_BIF_SIDE_3');

stateREAD_BIF_OB_0= dlmread('matlab/print_state_BIF_OB_0');
stateREAD_BIF_OB_1= dlmread('matlab/print_state_BIF_OB_1');
stateREAD_BIF_OB_2= dlmread('matlab/print_state_BIF_OB_2');
stateREAD_BIF_OB_3= dlmread('matlab/print_state_BIF_OB_3');

%%simple scene
stateREAD_TEST_TOP_0= dlmread('matlab/print_state_TEST_TOP_0');
stateREAD_TEST_TOP_1= dlmread('matlab/print_state_TEST_TOP_1');
stateREAD_TEST_TOP_2= dlmread('matlab/print_state_TEST_TOP_2');
stateREAD_TEST_TOP_3= dlmread('matlab/print_state_TEST_TOP_3');

stateREAD_TEST_SIDE_0= dlmread('matlab/print_state_TEST_SIDE_0');
stateREAD_TEST_SIDE_1= dlmread('matlab/print_state_TEST_SIDE_1');
stateREAD_TEST_SIDE_2= dlmread('matlab/print_state_TEST_SIDE_2');
stateREAD_TEST_SIDE_3= dlmread('matlab/print_state_TEST_SIDE_3');

stateREAD_TEST_OB_0= dlmread('matlab/print_state_TEST_OB_0');
stateREAD_TEST_OB_1= dlmread('matlab/print_state_TEST_OB_1');
stateREAD_TEST_OB_2= dlmread('matlab/print_state_TEST_OB_2');
stateREAD_TEST_OB_3= dlmread('matlab/print_state_TEST_OB_3');

%%retrieve position from filter state%%
pos_BIF_TOP_0=...
    cat(2,stateREAD_BIF_TOP_0(:,1:3),stateREAD_BIF_TOP_0(:,7:9),stateREAD_BIF_TOP_0(:,13:15),stateREAD_BIF_TOP_0(:,19:21),stateREAD_BIF_TOP_0(:,25:27),stateREAD_BIF_TOP_0(:,31:33),...
    stateREAD_BIF_TOP_0(:,37:39),stateREAD_BIF_TOP_0(:,43:45),stateREAD_BIF_TOP_0(:,49:51),stateREAD_BIF_TOP_0(:,55:57));
pos_BIF_TOP_1=...
    cat(2,stateREAD_BIF_TOP_1(:,1:3),stateREAD_BIF_TOP_1(:,7:9),stateREAD_BIF_TOP_1(:,13:15),stateREAD_BIF_TOP_1(:,19:21),stateREAD_BIF_TOP_1(:,25:27),stateREAD_BIF_TOP_1(:,31:33),...
    stateREAD_BIF_TOP_1(:,37:39),stateREAD_BIF_TOP_1(:,43:45),stateREAD_BIF_TOP_1(:,49:51),stateREAD_BIF_TOP_1(:,55:57));
pos_BIF_TOP_2=...
    cat(2,stateREAD_BIF_TOP_2(:,1:3),stateREAD_BIF_TOP_2(:,7:9),stateREAD_BIF_TOP_2(:,13:15),stateREAD_BIF_TOP_2(:,19:21),stateREAD_BIF_TOP_2(:,25:27),stateREAD_BIF_TOP_2(:,31:33),...
    stateREAD_BIF_TOP_2(:,37:39),stateREAD_BIF_TOP_2(:,43:45),stateREAD_BIF_TOP_2(:,49:51),stateREAD_BIF_TOP_2(:,55:57));
pos_BIF_TOP_3=...
    cat(2,stateREAD_BIF_TOP_3(:,1:3),stateREAD_BIF_TOP_3(:,7:9),stateREAD_BIF_TOP_3(:,13:15),stateREAD_BIF_TOP_3(:,19:21),stateREAD_BIF_TOP_3(:,25:27),stateREAD_BIF_TOP_3(:,31:33),...
    stateREAD_BIF_TOP_3(:,37:39),stateREAD_BIF_TOP_3(:,43:45),stateREAD_BIF_TOP_3(:,49:51),stateREAD_BIF_TOP_3(:,55:57));

pos_BIF_OB_0=...
    cat(2,stateREAD_BIF_OB_0(:,1:3),stateREAD_BIF_OB_0(:,7:9),stateREAD_BIF_OB_0(:,13:15),stateREAD_BIF_OB_0(:,19:21),stateREAD_BIF_OB_0(:,25:27),stateREAD_BIF_OB_0(:,31:33),...
    stateREAD_BIF_OB_0(:,37:39),stateREAD_BIF_OB_0(:,43:45),stateREAD_BIF_OB_0(:,49:51),stateREAD_BIF_OB_0(:,55:57));
pos_BIF_OB_1=...
    cat(2,stateREAD_BIF_OB_1(:,1:3),stateREAD_BIF_OB_1(:,7:9),stateREAD_BIF_OB_1(:,13:15),stateREAD_BIF_OB_1(:,19:21),stateREAD_BIF_OB_1(:,25:27),stateREAD_BIF_OB_1(:,31:33),...
    stateREAD_BIF_OB_1(:,37:39),stateREAD_BIF_OB_1(:,43:45),stateREAD_BIF_OB_1(:,49:51),stateREAD_BIF_OB_1(:,55:57));
pos_BIF_OB_2=...
    cat(2,stateREAD_BIF_OB_2(:,1:3),stateREAD_BIF_OB_2(:,7:9),stateREAD_BIF_OB_2(:,13:15),stateREAD_BIF_OB_2(:,19:21),stateREAD_BIF_OB_2(:,25:27),stateREAD_BIF_OB_2(:,31:33),...
    stateREAD_BIF_OB_2(:,37:39),stateREAD_BIF_OB_2(:,43:45),stateREAD_BIF_OB_2(:,49:51),stateREAD_BIF_OB_2(:,55:57));
pos_BIF_OB_3=...
    cat(2,stateREAD_BIF_OB_3(:,1:3),stateREAD_BIF_OB_3(:,7:9),stateREAD_BIF_OB_3(:,13:15),stateREAD_BIF_OB_3(:,19:21),stateREAD_BIF_OB_3(:,25:27),stateREAD_BIF_OB_3(:,31:33),...
    stateREAD_BIF_OB_3(:,37:39),stateREAD_BIF_OB_3(:,43:45),stateREAD_BIF_OB_3(:,49:51),stateREAD_BIF_OB_3(:,55:57));

pos_BIF_SIDE_0=...
    cat(2,stateREAD_BIF_SIDE_0(:,1:3),stateREAD_BIF_SIDE_0(:,7:9),stateREAD_BIF_SIDE_0(:,13:15),stateREAD_BIF_SIDE_0(:,19:21),stateREAD_BIF_SIDE_0(:,25:27),stateREAD_BIF_SIDE_0(:,31:33),...
    stateREAD_BIF_SIDE_0(:,37:39),stateREAD_BIF_SIDE_0(:,43:45),stateREAD_BIF_SIDE_0(:,49:51),stateREAD_BIF_SIDE_0(:,55:57));
pos_BIF_SIDE_1=...
    cat(2,stateREAD_BIF_SIDE_1(:,1:3),stateREAD_BIF_SIDE_1(:,7:9),stateREAD_BIF_SIDE_1(:,13:15),stateREAD_BIF_SIDE_1(:,19:21),stateREAD_BIF_SIDE_1(:,25:27),stateREAD_BIF_SIDE_1(:,31:33),...
    stateREAD_BIF_SIDE_1(:,37:39),stateREAD_BIF_SIDE_1(:,43:45),stateREAD_BIF_SIDE_1(:,49:51),stateREAD_BIF_SIDE_1(:,55:57));
pos_BIF_SIDE_2=...
    cat(2,stateREAD_BIF_SIDE_2(:,1:3),stateREAD_BIF_SIDE_2(:,7:9),stateREAD_BIF_SIDE_2(:,13:15),stateREAD_BIF_SIDE_2(:,19:21),stateREAD_BIF_SIDE_2(:,25:27),stateREAD_BIF_SIDE_2(:,31:33),...
    stateREAD_BIF_SIDE_2(:,37:39),stateREAD_BIF_SIDE_2(:,43:45),stateREAD_BIF_SIDE_2(:,49:51),stateREAD_BIF_SIDE_2(:,55:57));
pos_BIF_SIDE_3=...
    cat(2,stateREAD_BIF_SIDE_3(:,1:3),stateREAD_BIF_SIDE_3(:,7:9),stateREAD_BIF_SIDE_3(:,13:15),stateREAD_BIF_SIDE_3(:,19:21),stateREAD_BIF_SIDE_3(:,25:27),stateREAD_BIF_SIDE_3(:,31:33),...
    stateREAD_BIF_SIDE_3(:,37:39),stateREAD_BIF_SIDE_3(:,43:45),stateREAD_BIF_SIDE_3(:,49:51),stateREAD_BIF_SIDE_3(:,55:57));
%%

pos_TEST_TOP_0=...
    cat(2,stateREAD_TEST_TOP_0(:,1:3),stateREAD_TEST_TOP_0(:,7:9),stateREAD_TEST_TOP_0(:,13:15),stateREAD_TEST_TOP_0(:,19:21),stateREAD_TEST_TOP_0(:,25:27),stateREAD_TEST_TOP_0(:,31:33),...
    stateREAD_TEST_TOP_0(:,37:39),stateREAD_TEST_TOP_0(:,43:45),stateREAD_TEST_TOP_0(:,49:51),stateREAD_TEST_TOP_0(:,55:57));
pos_TEST_TOP_1=...
    cat(2,stateREAD_TEST_TOP_1(:,1:3),stateREAD_TEST_TOP_1(:,7:9),stateREAD_TEST_TOP_1(:,13:15),stateREAD_TEST_TOP_1(:,19:21),stateREAD_TEST_TOP_1(:,25:27),stateREAD_TEST_TOP_1(:,31:33),...
    stateREAD_TEST_TOP_1(:,37:39),stateREAD_TEST_TOP_1(:,43:45),stateREAD_TEST_TOP_1(:,49:51),stateREAD_TEST_TOP_1(:,55:57));
pos_TEST_TOP_2=...
    cat(2,stateREAD_TEST_TOP_2(:,1:3),stateREAD_TEST_TOP_2(:,7:9),stateREAD_TEST_TOP_2(:,13:15),stateREAD_TEST_TOP_2(:,19:21),stateREAD_TEST_TOP_2(:,25:27),stateREAD_TEST_TOP_2(:,31:33),...
    stateREAD_TEST_TOP_2(:,37:39),stateREAD_TEST_TOP_2(:,43:45),stateREAD_TEST_TOP_2(:,49:51),stateREAD_TEST_TOP_2(:,55:57));
pos_TEST_TOP_3=...
    cat(2,stateREAD_TEST_TOP_3(:,1:3),stateREAD_TEST_TOP_3(:,7:9),stateREAD_TEST_TOP_3(:,13:15),stateREAD_TEST_TOP_3(:,19:21),stateREAD_TEST_TOP_3(:,25:27),stateREAD_TEST_TOP_3(:,31:33),...
    stateREAD_TEST_TOP_3(:,37:39),stateREAD_TEST_TOP_3(:,43:45),stateREAD_TEST_TOP_3(:,49:51),stateREAD_TEST_TOP_3(:,55:57));

pos_TEST_OB_0=...
    cat(2,stateREAD_TEST_OB_0(:,1:3),stateREAD_TEST_OB_0(:,7:9),stateREAD_TEST_OB_0(:,13:15),stateREAD_TEST_OB_0(:,19:21),stateREAD_TEST_OB_0(:,25:27),stateREAD_TEST_OB_0(:,31:33),...
    stateREAD_TEST_OB_0(:,37:39),stateREAD_TEST_OB_0(:,43:45),stateREAD_TEST_OB_0(:,49:51),stateREAD_TEST_OB_0(:,55:57));
pos_TEST_OB_1=...
    cat(2,stateREAD_TEST_OB_1(:,1:3),stateREAD_TEST_OB_1(:,7:9),stateREAD_TEST_OB_1(:,13:15),stateREAD_TEST_OB_1(:,19:21),stateREAD_TEST_OB_1(:,25:27),stateREAD_TEST_OB_1(:,31:33),...
    stateREAD_TEST_OB_1(:,37:39),stateREAD_TEST_OB_1(:,43:45),stateREAD_TEST_OB_1(:,49:51),stateREAD_TEST_OB_1(:,55:57));
pos_TEST_OB_2=...
    cat(2,stateREAD_TEST_OB_2(:,1:3),stateREAD_TEST_OB_2(:,7:9),stateREAD_TEST_OB_2(:,13:15),stateREAD_TEST_OB_2(:,19:21),stateREAD_TEST_OB_2(:,25:27),stateREAD_TEST_OB_2(:,31:33),...
    stateREAD_TEST_OB_2(:,37:39),stateREAD_TEST_OB_2(:,43:45),stateREAD_TEST_OB_2(:,49:51),stateREAD_TEST_OB_2(:,55:57));
pos_TEST_OB_3=...
    cat(2,stateREAD_TEST_OB_3(:,1:3),stateREAD_TEST_OB_3(:,7:9),stateREAD_TEST_OB_3(:,13:15),stateREAD_TEST_OB_3(:,19:21),stateREAD_TEST_OB_3(:,25:27),stateREAD_TEST_OB_3(:,31:33),...
    stateREAD_TEST_OB_3(:,37:39),stateREAD_TEST_OB_3(:,43:45),stateREAD_TEST_OB_3(:,49:51),stateREAD_TEST_OB_3(:,55:57));

pos_TEST_SIDE_0=...
    cat(2,stateREAD_TEST_SIDE_0(:,1:3),stateREAD_TEST_SIDE_0(:,7:9),stateREAD_TEST_SIDE_0(:,13:15),stateREAD_TEST_SIDE_0(:,19:21),stateREAD_TEST_SIDE_0(:,25:27),stateREAD_TEST_SIDE_0(:,31:33),...
    stateREAD_TEST_SIDE_0(:,37:39),stateREAD_TEST_SIDE_0(:,43:45),stateREAD_TEST_SIDE_0(:,49:51),stateREAD_TEST_SIDE_0(:,55:57));
pos_TEST_SIDE_1=...
    cat(2,stateREAD_TEST_SIDE_1(:,1:3),stateREAD_TEST_SIDE_1(:,7:9),stateREAD_TEST_SIDE_1(:,13:15),stateREAD_TEST_SIDE_1(:,19:21),stateREAD_TEST_SIDE_1(:,25:27),stateREAD_TEST_SIDE_1(:,31:33),...
    stateREAD_TEST_SIDE_1(:,37:39),stateREAD_TEST_SIDE_1(:,43:45),stateREAD_TEST_SIDE_1(:,49:51),stateREAD_TEST_SIDE_1(:,55:57));
pos_TEST_SIDE_2=...
    cat(2,stateREAD_TEST_SIDE_2(:,1:3),stateREAD_TEST_SIDE_2(:,7:9),stateREAD_TEST_SIDE_2(:,13:15),stateREAD_TEST_SIDE_2(:,19:21),stateREAD_TEST_SIDE_2(:,25:27),stateREAD_TEST_SIDE_2(:,31:33),...
    stateREAD_TEST_SIDE_2(:,37:39),stateREAD_TEST_SIDE_2(:,43:45),stateREAD_TEST_SIDE_2(:,49:51),stateREAD_TEST_SIDE_2(:,55:57));
pos_TEST_SIDE_3=...
    cat(2,stateREAD_TEST_SIDE_3(:,1:3),stateREAD_TEST_SIDE_3(:,7:9),stateREAD_TEST_SIDE_3(:,13:15),stateREAD_TEST_SIDE_3(:,19:21),stateREAD_TEST_SIDE_3(:,25:27),stateREAD_TEST_SIDE_3(:,31:33),...
    stateREAD_TEST_SIDE_3(:,37:39),stateREAD_TEST_SIDE_3(:,43:45),stateREAD_TEST_SIDE_3(:,49:51),stateREAD_TEST_SIDE_3(:,55:57));

%%
msBIF=1230;
msTEST=2900;

%% Project Bifurcation Scene
gt2d_BIF_TOP=proj(gt_BIF_TOP,P_BIF_TOP,msBIF);
pos2d_BIF_TOP_0=proj(pos_BIF_TOP_0, P_BIF_TOP, msBIF);
pos2d_BIF_TOP_1=proj(pos_BIF_TOP_1, P_BIF_TOP, msBIF);
pos2d_BIF_TOP_2=proj(pos_BIF_TOP_2, P_BIF_TOP, msBIF);
pos2d_BIF_TOP_3=proj(pos_BIF_TOP_3, P_BIF_TOP, msBIF);

gt2d_BIF_OB=proj(gt_BIF_OB,P_BIF_OB,msBIF);
pos2d_BIF_OB_0=proj(pos_BIF_OB_0, P_BIF_OB, msBIF);
pos2d_BIF_OB_1=proj(pos_BIF_OB_1, P_BIF_OB, msBIF);
pos2d_BIF_OB_2=proj(pos_BIF_OB_2, P_BIF_OB, msBIF);
pos2d_BIF_OB_3=proj(pos_BIF_OB_3, P_BIF_OB, msBIF);

gt2d_BIF_SIDE=proj(gt_BIF_SIDE,P_BIF_SIDE,msBIF);
pos2d_BIF_SIDE_0=proj(pos_BIF_SIDE_0, P_BIF_SIDE, msBIF);
pos2d_BIF_SIDE_1=proj(pos_BIF_SIDE_1, P_BIF_SIDE, msBIF);
pos2d_BIF_SIDE_2=proj(pos_BIF_SIDE_2, P_BIF_SIDE, msBIF);
pos2d_BIF_SIDE_3=proj(pos_BIF_SIDE_3, P_BIF_SIDE, msBIF);

%% Project Simple Scene
gt2d_TEST_TOP=proj(gt_TEST_TOP,P_TEST_TOP,msTEST);
pos2d_TEST_TOP_0=proj(pos_TEST_TOP_0, P_TEST_TOP, msTEST);
pos2d_TEST_TOP_1=proj(pos_TEST_TOP_1, P_TEST_TOP, msTEST);
pos2d_TEST_TOP_2=proj(pos_TEST_TOP_2, P_TEST_TOP, msTEST);
pos2d_TEST_TOP_3=proj(pos_TEST_TOP_3, P_TEST_TOP, msTEST);

gt2d_TEST_OB=proj(gt_TEST_OB,P_TEST_OB,msTEST);
pos2d_TEST_OB_0=proj(pos_TEST_OB_0, P_TEST_OB_0, msTEST);
pos2d_TEST_OB_1=proj(pos_TEST_OB_1, P_TEST_OB, msTEST);
pos2d_TEST_OB_2=proj(pos_TEST_OB_2, P_TEST_OB, msTEST);
pos2d_TEST_OB_3=proj(pos_TEST_OB_3, P_TEST_OB, msTEST);

gt2d_TEST_SIDE=proj(gt_TEST_SIDE,P_TEST_SIDE,msTEST);
pos2d_TEST_SIDE_0=proj(pos_TEST_SIDE_0, P_TEST_SIDE, msTEST);
pos2d_TEST_SIDE_1=proj(pos_TEST_SIDE_1, P_TEST_SIDE, msTEST);
pos2d_TEST_SIDE_2=proj(pos_TEST_SIDE_2, P_TEST_SIDE, msTEST);
pos2d_TEST_SIDE_3=proj(pos_TEST_SIDE_3, P_TEST_SIDE, msTEST);


% ALL3D=cat(3,pos_BIF_TOP_0,pos_BIF_TOP_1,pos_BIF_TOP_2,pos_BIF_TOP_3,...
%             pos_BIF_SIDE_0,pos_BIF_SIDE_1,pos_BIF_SIDE_2,pos_BIF_SIDE_3,...
%             pos_BIF_OB_0,pos_BIF_OB_1,pos_BIF_OB_2,pos_BIF_SIDE_3,...
%             pos_TEST_TOP_0,pos_TEST_TOP_1,pos_TEST_TOP_2,pos_TEST_TOP_3,...
%             pos_TEST_SIDE_0,pos_TEST_SIDE_1,pos_TEST_SIDE_2,pos_TEST_SIDE_3,...
%             pos_TEST_OB_0,pos_TEST_OB_1,pos_TEST_OB_2,pos_TEST_SIDE_3);
%         
% ALL2D=cat(3,pos2d_BIF_TOP_0,pos2d_BIF_TOP_1,pos2d_BIF_TOP_2,pos2d_BIF_TOP_3,...
%             pos2d_BIF_SIDE_0,pos2d_BIF_SIDE_1,pos2d_BIF_SIDE_2,pos2d_BIF_SIDE_3,...
%             pos2d_BIF_OB_0,pos2d_BIF_OB_1,pos2d_BIF_OB_2,pos2d_BIF_SIDE_3,...
%             pos2d_TEST_TOP_0,pos2d_TEST_TOP_1,pos2d_TEST_TOP_2,pos2d_TEST_TOP_3,...
%             pos2d_TEST_SIDE_0,pos2d_TEST_SIDE_1,pos2d_TEST_SIDE_2,pos2d_TEST_SIDE_3,...
%             pos2d_TEST_OB_0,pos2d_TEST_OB_1,pos2d_TEST_OB_2,pos2d_TEST_SIDE_3);
%         
% gtALL3D=cat(3,gt_BIF_TOP,gt_BIF_SIDE,gt_BIF_OB, gt_TEST_TOP, gt_TEST_SIDE, gt_TEST_OB);
% gtALL2D=cat(3,gt2d_BIF_TOP,gt2d_BIF_SIDE,gt2d_BIF_OB, gt2d_TEST_TOP, gt2d_TEST_SIDE, gt2d_TEST_OB);

%% MIN DIST TIP 3D %%

d_TEST_TOP_0=ones(msTEST,1); d_TEST_TOP_1=ones(msTEST,1); d_TEST_TOP_2=ones(msTEST,1); d_TEST_TOP_3=ones(msTEST,1);
d_TEST_SIDE_0=ones(msTEST,1); d_TEST_SIDE_1=ones(msTEST,1); d_TEST_SIDE_2=ones(msTEST,1); d_TEST_SIDE_3=ones(msTEST,1);
d_TEST_OB_0=ones(msTEST,1); d_TEST_OB_1=ones(msTEST,1); d_TEST_OB_2=ones(msTEST,1); d_TEST_OB_3=ones(msTEST,1);

d_BIF_TOP_0=ones(msBIF,1); d_BIF_TOP_1=ones(msBIF,1); d_BIF_TOP_2=ones(msBIF,1); d_BIF_TOP_3=ones(msBIF,1); 
d_BIF_SIDE_0=ones(msBIF,1); d_BIF_SIDE_1=ones(msBIF,1); d_BIF_SIDE_2=ones(msBIF,1); d_BIF_SIDE_3=ones(msBIF,1); 
d_BIF_OB_0=ones(msBIF,1); d_BIF_OB_1=ones(msBIF,1); d_BIF_OB_2=ones(msBIF,1); d_BIF_OB_3=ones(msBIF,1);


for i =1:msTEST
    d_TEST_TOP_0(i)=minDistLine(gt_TEST_TOP(i,1:3),gt_TEST_TOP(i,4:6),pos_TEST_TOP_0(i,1:3),pos_TEST_TOP_0(i,4:6)) ;
        d_TEST_TOP_1(i)=minDistLine(gt_TEST_TOP(i,1:3),gt_TEST_TOP(i,4:6),pos_TEST_TOP_1(i,1:3),pos_TEST_TOP_1(i,4:6)) ;
            d_TEST_TOP_2(i)=minDistLine(gt_TEST_TOP(i,1:3),gt_TEST_TOP(i,4:6),pos_TEST_TOP_2(i,1:3),pos_TEST_TOP_2(i,4:6)) ;
                d_TEST_TOP_3(i)=minDistLine(gt_TEST_TOP(i,1:3),gt_TEST_TOP(i,4:6),pos_TEST_TOP_3(i,1:3),pos_TEST_TOP_3(i,4:6)) ;
    d_TEST_OB_0(i)=minDistLine(gt_TEST_OB(i,1:3),gt_TEST_OB(i,4:6),pos_TEST_OB_0(i,1:3),pos_TEST_OB_0(i,4:6)) ;
        d_TEST_OB_1(i)=minDistLine(gt_TEST_OB(i,1:3),gt_TEST_OB(i,4:6),pos_TEST_OB_1(i,1:3),pos_TEST_OB_1(i,4:6)) ;
            d_TEST_OB_2(i)=minDistLine(gt_TEST_OB(i,1:3),gt_TEST_OB(i,4:6),pos_TEST_OB_2(i,1:3),pos_TEST_OB_2(i,4:6)) ;
                d_TEST_OB_3(i)=minDistLine(gt_TEST_OB(i,1:3),gt_TEST_OB(i,4:6),pos_TEST_OB_3(i,1:3),pos_TEST_OB_3(i,4:6)) ;
    d_TEST_SIDE_0(i)=minDistLine(gt_TEST_SIDE(i,1:3),gt_TEST_SIDE(i,4:6),pos_TEST_SIDE_0(i,1:3),pos_TEST_SIDE_0(i,4:6)) ;
        d_TEST_SIDE_1(i)=minDistLine(gt_TEST_SIDE(i,1:3),gt_TEST_SIDE(i,4:6),pos_TEST_SIDE_1(i,1:3),pos_TEST_SIDE_1(i,4:6)) ;
            d_TEST_SIDE_2(i)=minDistLine(gt_TEST_SIDE(i,1:3),gt_TEST_SIDE(i,4:6),pos_TEST_SIDE_2(i,1:3),pos_TEST_SIDE_2(i,4:6)) ;
                d_TEST_SIDE_3(i)=minDistLine(gt_TEST_SIDE(i,1:3),gt_TEST_SIDE(i,4:6),pos_TEST_SIDE_3(i,1:3),pos_TEST_OB_3(i,4:6)) ;
end


for i =1:msBIF
    d_BIF_TOP_0(i)=minDistLine(gt_BIF_TOP(i,1:3),gt_BIF_TOP(i,4:6),pos_BIF_TOP_0(i,1:3),pos_BIF_TOP_0(i,4:6)) ;
        d_BIF_TOP_1(i)=minDistLine(gt_BIF_TOP(i,1:3),gt_BIF_TOP(i,4:6),pos_BIF_TOP_1(i,1:3),pos_BIF_TOP_1(i,4:6)) ;
            d_BIF_TOP_2(i)=minDistLine(gt_BIF_TOP(i,1:3),gt_BIF_TOP(i,4:6),pos_BIF_TOP_2(i,1:3),pos_BIF_TOP_2(i,4:6)) ;
                d_BIF_TOP_3(i)=minDistLine(gt_BIF_TOP(i,1:3),gt_BIF_TOP(i,4:6),pos_BIF_TOP_3(i,1:3),pos_BIF_TOP_3(i,4:6)) ;
    d_BIF_OB_0(i)=minDistLine(gt_BIF_OB(i,1:3),gt_BIF_OB(i,4:6),pos_BIF_OB_0(i,1:3),pos_BIF_OB_0(i,4:6)) ;
        d_BIF_OB_1(i)=minDistLine(gt_BIF_OB(i,1:3),gt_BIF_OB(i,4:6),pos_BIF_OB_1(i,1:3),pos_BIF_OB_1(i,4:6)) ;
            d_BIF_OB_2(i)=minDistLine(gt_BIF_OB(i,1:3),gt_BIF_OB(i,4:6),pos_BIF_OB_2(i,1:3),pos_BIF_OB_2(i,4:6)) ;
                d_BIF_OB_3(i)=minDistLine(gt_BIF_OB(i,1:3),gt_BIF_OB(i,4:6),pos_BIF_OB_3(i,1:3),pos_BIF_OB_3(i,4:6)) ;
    d_BIF_SIDE_0(i)=minDistLine(gt_BIF_SIDE(i,1:3),gt_BIF_SIDE(i,4:6),pos_BIF_SIDE_0(i,1:3),pos_BIF_SIDE_0(i,4:6)) ;
        d_BIF_SIDE_1(i)=minDistLine(gt_BIF_SIDE(i,1:3),gt_BIF_SIDE(i,4:6),pos_BIF_SIDE_1(i,1:3),pos_BIF_SIDE_1(i,4:6)) ;
            d_BIF_SIDE_2(i)=minDistLine(gt_BIF_SIDE(i,1:3),gt_BIF_SIDE(i,4:6),pos_BIF_SIDE_2(i,1:3),pos_BIF_SIDE_2(i,4:6)) ;
                d_BIF_SIDE_3(i)=minDistLine(gt_BIF_SIDE(i,1:3),gt_BIF_SIDE(i,4:6),pos_BIF_SIDE_3(i,1:3),pos_BIF_OB_3(i,4:6)) ;
end
  
%% MIN DIST TIP 2D %%

d2d_TEST_TOP_0=ones(msTEST,1); d2d_TEST_TOP_1=ones(msTEST,1); d2d_TEST_TOP_2=ones(msTEST,1); d2d_TEST_TOP_3=ones(msTEST,1);
d2d_TEST_SIDE_0=ones(msTEST,1); d2d_TEST_SIDE_1=ones(msTEST,1); d2d_TEST_SIDE_2=ones(msTEST,1); d2d_TEST_SIDE_3=ones(msTEST,1);
d2d_TEST_OB_0=ones(msTEST,1); d2d_TEST_OB_1=ones(msTEST,1); d2d_TEST_OB_2=ones(msTEST,1); d2d_TEST_OB_3=ones(msTEST,1);

d2d_BIF_TOP_0=ones(msBIF,1); d2d_BIF_TOP_1=ones(msBIF,1); d2d_BIF_TOP_2=ones(msBIF,1); d2d_BIF_TOP_3=ones(msBIF,1); 
d2d_BIF_SIDE_0=ones(msBIF,1); d2d_BIF_SIDE_1=ones(msBIF,1); d2d_BIF_SIDE_2=ones(msBIF,1); d2d_BIF_SIDE_3=ones(msBIF,1); 
d2d_BIF_OB_0=ones(msBIF,1); d2d_BIF_OB_1=ones(msBIF,1); d2d_BIF_OB_2=ones(msBIF,1); d2d_BIF_OB_3=ones(msBIF,1);


for i =1:msTEST
    d2d_TEST_TOP_0(i)=minDistLine(gt2d_TEST_TOP(i,1:2),gt2d_TEST_TOP(i,3:4),pos2d_TEST_TOP_0(i,1:2),pos2d_TEST_TOP_0(i,3:4)) ;
        d2d_TEST_TOP_1(i)=minDistLine(gt2d_TEST_TOP(i,1:2),gt2d_TEST_TOP(i,3:4),pos2d_TEST_TOP_1(i,1:2),pos2d_TEST_TOP_1(i,3:4)) ;
            d2d_TEST_TOP_2(i)=minDistLine(gt2d_TEST_TOP(i,1:2),gt2d_TEST_TOP(i,3:4),pos2d_TEST_TOP_2(i,1:2),pos2d_TEST_TOP_2(i,3:4)) ;
                d2d_TEST_TOP_3(i)=minDistLine(gt2d_TEST_TOP(i,1:2),gt2d_TEST_TOP(i,3:4),pos2d_TEST_TOP_3(i,1:2),pos2d_TEST_TOP_3(i,3:4)) ;
    d2d_TEST_OB_0(i)=minDistLine(gt2d_TEST_OB(i,1:2),gt2d_TEST_OB(i,3:4),pos2d_TEST_OB_0(i,1:2),pos2d_TEST_OB_0(i,3:4)) ;
        d2d_TEST_OB_1(i)=minDistLine(gt2d_TEST_OB(i,1:2),gt2d_TEST_OB(i,3:4),pos2d_TEST_OB_1(i,1:2),pos2d_TEST_OB_1(i,3:4)) ;
            d2d_TEST_OB_2(i)=minDistLine(gt2d_TEST_OB(i,1:2),gt2d_TEST_OB(i,3:4),pos2d_TEST_OB_2(i,1:2),pos2d_TEST_OB_2(i,3:4)) ;
                d2d_TEST_OB_3(i)=minDistLine(gt2d_TEST_OB(i,1:2),gt2d_TEST_OB(i,3:4),pos2d_TEST_OB_3(i,1:2),pos2d_TEST_OB_3(i,3:4)) ;
    d2d_TEST_SIDE_0(i)=minDistLine(gt2d_TEST_SIDE(i,1:2),gt2d_TEST_SIDE(i,3:4),pos2d_TEST_SIDE_0(i,1:2),pos2d_TEST_SIDE_0(i,3:4)) ;
        d2d_TEST_SIDE_1(i)=minDistLine(gt2d_TEST_SIDE(i,1:2),gt2d_TEST_SIDE(i,3:4),pos2d_TEST_SIDE_1(i,1:2),pos2d_TEST_SIDE_1(i,3:4)) ;
            d2d_TEST_SIDE_2(i)=minDistLine(gt2d_TEST_SIDE(i,1:2),gt2d_TEST_SIDE(i,3:4),pos2d_TEST_SIDE_2(i,1:2),pos2d_TEST_SIDE_2(i,3:4)) ;
                d2d_TEST_SIDE_3(i)=minDistLine(gt2d_TEST_SIDE(i,1:2),gt2d_TEST_SIDE(i,3:4),pos2d_TEST_SIDE_3(i,1:2),pos2d_TEST_OB_3(i,3:4)) ;
end


for i =1:msBIF
    d2d_BIF_TOP_0(i)=minDistLine(gt2d_BIF_TOP(i,1:2),gt2d_BIF_TOP(i,3:4),pos2d_BIF_TOP_0(i,1:2),pos2d_BIF_TOP_0(i,3:4)) ;
        d2d_BIF_TOP_1(i)=minDistLine(gt2d_BIF_TOP(i,1:2),gt2d_BIF_TOP(i,3:4),pos2d_BIF_TOP_1(i,1:2),pos2d_BIF_TOP_1(i,3:4)) ;
            d2d_BIF_TOP_2(i)=minDistLine(gt2d_BIF_TOP(i,1:2),gt2d_BIF_TOP(i,3:4),pos2d_BIF_TOP_2(i,1:2),pos2d_BIF_TOP_2(i,3:4)) ;
                d2d_BIF_TOP_3(i)=minDistLine(gt2d_BIF_TOP(i,1:2),gt2d_BIF_TOP(i,3:4),pos2d_BIF_TOP_3(i,1:2),pos2d_BIF_TOP_3(i,3:4)) ;
    d2d_BIF_OB_0(i)=minDistLine(gt2d_BIF_OB(i,1:2),gt2d_BIF_OB(i,3:4),pos2d_BIF_OB_0(i,1:2),pos2d_BIF_OB_0(i,3:4)) ;
        d2d_BIF_OB_1(i)=minDistLine(gt2d_BIF_OB(i,1:2),gt2d_BIF_OB(i,3:4),pos2d_BIF_OB_1(i,1:2),pos2d_BIF_OB_1(i,3:4)) ;
            d2d_BIF_OB_2(i)=minDistLine(gt2d_BIF_OB(i,1:2),gt2d_BIF_OB(i,3:4),pos2d_BIF_OB_2(i,1:2),pos2d_BIF_OB_2(i,3:4)) ;
                d2d_BIF_OB_3(i)=minDistLine(gt2d_BIF_OB(i,1:2),gt2d_BIF_OB(i,3:4),pos2d_BIF_OB_3(i,1:2),pos2d_BIF_OB_3(i,3:4)) ;
    d2d_BIF_SIDE_0(i)=minDistLine(gt2d_BIF_SIDE(i,1:2),gt2d_BIF_SIDE(i,3:4),pos2d_BIF_SIDE_0(i,1:2),pos2d_BIF_SIDE_0(i,3:4)) ;
        d2d_BIF_SIDE_1(i)=minDistLine(gt2d_BIF_SIDE(i,1:2),gt2d_BIF_SIDE(i,3:4),pos2d_BIF_SIDE_1(i,1:2),pos2d_BIF_SIDE_1(i,3:4)) ;
            d2d_BIF_SIDE_2(i)=minDistLine(gt2d_BIF_SIDE(i,1:2),gt2d_BIF_SIDE(i,3:4),pos2d_BIF_SIDE_2(i,1:2),pos2d_BIF_SIDE_2(i,3:4)) ;
                d2d_BIF_SIDE_3(i)=minDistLine(gt2d_BIF_SIDE(i,1:2),gt2d_BIF_SIDE(i,3:4),pos2d_BIF_SIDE_3(i,1:2),pos2d_BIF_OB_3(i,3:4)) ;
end




%% HAUSDORF DISTANCE 3D

hd_TEST_TOP_0=ones(msTEST,1); hd_TEST_TOP_1=ones(msTEST,1); hd_TEST_TOP_2=ones(msTEST,1); hd_TEST_TOP_3=ones(msTEST,1);
hd_TEST_SIDE_0=ones(msTEST,1); hd_TEST_SIDE_1=ones(msTEST,1); hd_TEST_SIDE_2=ones(msTEST,1); hd_TEST_SIDE_3=ones(msTEST,1);
hd_TEST_OB_0=ones(msTEST,1); hd_TEST_OB_1=ones(msTEST,1); hd_TEST_OB_2=ones(msTEST,1); hd_TEST_OB_3=ones(msTEST,1);

hd_BIF_TOP_0=ones(msBIF,1); hd_BIF_TOP_1=ones(msBIF,1); hd_BIF_TOP_2=ones(msBIF,1); hd_BIF_TOP_3=ones(msBIF,1); 
hd_BIF_SIDE_0=ones(msBIF,1); hd_BIF_SIDE_1=ones(msBIF,1); hd_BIF_SIDE_2=ones(msBIF,1); hd_BIF_SIDE_3=ones(msBIF,1); 
hd_BIF_OB_0=ones(msBIF,1); hd_BIF_OB_1=ones(msBIF,1); hd_BIF_OB_2=ones(msBIF,1); hd_BIF_OB_3=ones(msBIF,1);


for i=1:msTEST
    hd_TEST_TOP_0(i)=HausdorffDist(gt_TEST_TOP(i,:),pos_TEST_TOP_0(i,:));
        hd_TEST_TOP_1(i)=HausdorffDist(gt_TEST_TOP(i,:), pos_TEST_TOP_1(i,:)) ;
            hd_TEST_TOP_2(i)=HausdorffDist(gt_TEST_TOP(i,:),pos_TEST_TOP_2(i,:));
                hd_TEST_TOP_3(i)=HausdorffDist(gt_TEST_TOP(i,:),pos_TEST_TOP_3(i,:)) ;
    hd_TEST_OB_0(i)=HausdorffDist(gt_TEST_OB(i,:),pos_TEST_OB_0(i,:)) ;
        hd_TEST_OB_1(i)=HausdorffDist(gt_TEST_OB(i,:),pos_TEST_OB_1(i,:)) ;
            hd_TEST_OB_2(i)=HausdorffDist(gt_TEST_OB(i,:),pos_TEST_OB_2(i,:)) ;
                hd_TEST_OB_3(i)=HausdorffDist(gt_TEST_OB(i,:),pos_TEST_OB_3(i,:)) ;
    hd_TEST_SIDE_0(i)=HausdorffDist(gt_TEST_SIDE(i,:),pos_TEST_SIDE_0(i,:)) ;
        hd_TEST_SIDE_1(i)=HausdorffDist(gt_TEST_SIDE(i,:),pos_TEST_SIDE_1(i,:)) ;
            hd_TEST_SIDE_2(i)=HausdorffDist(gt_TEST_SIDE(i,:),pos_TEST_SIDE_2(i,:)) ;
                hd_TEST_SIDE_3(i)=HausdorffDist(gt_TEST_SIDE(i,:),pos_TEST_SIDE_3(i,:)) ;
end

for i=1:msBIF
    hd_BIF_TOP_0(i)=HausdorffDist(gt_BIF_TOP(i,:),pos_BIF_TOP_0(i,:));
        hd_BIF_TOP_1(i)=HausdorffDist(gt_BIF_TOP(i,:), pos_BIF_TOP_1(i,:)) ;
            hd_BIF_TOP_2(i)=HausdorffDist(gt_BIF_TOP(i,:),pos_BIF_TOP_2(i,:));
                hd_BIF_TOP_3(i)=HausdorffDist(gt_BIF_TOP(i,:),pos_BIF_TOP_3(i,:)) ;
    hd_BIF_OB_0(i)=HausdorffDist(gt_BIF_OB(i,:),pos_BIF_OB_0(i,:)) ;
        hd_BIF_OB_1(i)=HausdorffDist(gt_BIF_OB(i,:),pos_BIF_OB_1(i,:)) ;
            hd_BIF_OB_2(i)=HausdorffDist(gt_BIF_OB(i,:),pos_BIF_OB_2(i,:)) ;
                hd_BIF_OB_3(i)=HausdorffDist(gt_BIF_OB(i,:),pos_BIF_OB_3(i,:)) ;
    hd_BIF_SIDE_0(i)=HausdorffDist(gt_BIF_SIDE(i,:),pos_BIF_SIDE_0(i,:)) ;
        hd_BIF_SIDE_1(i)=HausdorffDist(gt_BIF_SIDE(i,:),pos_BIF_SIDE_1(i,:)) ;
            hd_BIF_SIDE_2(i)=HausdorffDist(gt_BIF_SIDE(i,:),pos_BIF_SIDE_2(i,:)) ;
                hd_BIF_SIDE_3(i)=HausdorffDist(gt_BIF_SIDE(i,:),pos_BIF_SIDE_3(i,:)) ;
end

%% HAUSSDORF DISTANCE 2D 
hd2d_TEST_TOP_0=ones(msTEST,1); hd2d_TEST_TOP_1=ones(msTEST,1); hd2d_TEST_TOP_2=ones(msTEST,1); hd2d_TEST_TOP_3=ones(msTEST,1);
hd2d_TEST_SIDE_0=ones(msTEST,1); hd2d_TEST_SIDE_1=ones(msTEST,1); hd2d_TEST_SIDE_2=ones(msTEST,1); hd2d_TEST_SIDE_3=ones(msTEST,1);
hd2d_TEST_OB_0=ones(msTEST,1); hd2d_TEST_OB_1=ones(msTEST,1); hd2d_TEST_OB_2=ones(msTEST,1); hd2d_TEST_OB_3=ones(msTEST,1);

hd2d_BIF_TOP_0=ones(msBIF,1); hd2d_BIF_TOP_1=ones(msBIF,1); hd2d_BIF_TOP_2=ones(msBIF,1); hd2d_BIF_TOP_3=ones(msBIF,1); 
hd2d_BIF_SIDE_0=ones(msBIF,1); hd2d_BIF_SIDE_1=ones(msBIF,1); hd2d_BIF_SIDE_2=ones(msBIF,1); hd2d_BIF_SIDE_3=ones(msBIF,1); 
hd2d_BIF_OB_0=ones(msBIF,1); hd2d_BIF_OB_1=ones(msBIF,1); hd2d_BIF_OB_2=ones(msBIF,1); hd2d_BIF_OB_3=ones(msBIF,1);
for i=1:msTEST
    hd2d_TEST_TOP_0(i)=HausdorffDist(gt2d_TEST_TOP(i,:),pos2d_TEST_TOP_0(i,:));
        hd2d_TEST_TOP_1(i)=HausdorffDist(gt2d_TEST_TOP(i,:), pos2d_TEST_TOP_1(i,:)) ;
            hd2d_TEST_TOP_2(i)=HausdorffDist(gt2d_TEST_TOP(i,:),pos2d_TEST_TOP_2(i,:));
                hd2d_TEST_TOP_3(i)=HausdorffDist(gt2d_TEST_TOP(i,:),pos2d_TEST_TOP_3(i,:)) ;
    hd2d_TEST_OB_0(i)=HausdorffDist(gt2d_TEST_OB(i,:),pos2d_TEST_OB_0(i,:)) ;
        hd2d_TEST_OB_1(i)=HausdorffDist(gt2d_TEST_OB(i,:),pos2d_TEST_OB_1(i,:)) ;
            hd2d_TEST_OB_2(i)=HausdorffDist(gt2d_TEST_OB(i,:),pos2d_TEST_OB_2(i,:)) ;
                hd2d_TEST_OB_3(i)=HausdorffDist(gt2d_TEST_OB(i,:),pos2d_TEST_OB_3(i,:)) ;
    hd2d_TEST_SIDE_0(i)=HausdorffDist(gt2d_TEST_SIDE(i,:),pos2d_TEST_SIDE_0(i,:)) ;
        hd2d_TEST_SIDE_1(i)=HausdorffDist(gt2d_TEST_SIDE(i,:),pos2d_TEST_SIDE_1(i,:)) ;
            hd2d_TEST_SIDE_2(i)=HausdorffDist(gt2d_TEST_SIDE(i,:),pos2d_TEST_SIDE_2(i,:)) ;
                hd2d_TEST_SIDE_3(i)=HausdorffDist(gt2d_TEST_SIDE(i,:),pos2d_TEST_SIDE_3(i,:)) ;
end

for i=1:msBIF
    hd2d_BIF_TOP_0(i)=HausdorffDist(gt2d_BIF_TOP(i,:),pos2d_BIF_TOP_0(i,:));
        hd2d_BIF_TOP_1(i)=HausdorffDist(gt2d_BIF_TOP(i,:), pos2d_BIF_TOP_1(i,:)) ;
            hd2d_BIF_TOP_2(i)=HausdorffDist(gt2d_BIF_TOP(i,:),pos2d_BIF_TOP_2(i,:));
                hd2d_BIF_TOP_3(i)=HausdorffDist(gt2d_BIF_TOP(i,:),pos2d_BIF_TOP_3(i,:)) ;
    hd2d_BIF_OB_0(i)=HausdorffDist(gt2d_BIF_OB(i,:),pos2d_BIF_OB_0(i,:)) ;
        hd2d_BIF_OB_1(i)=HausdorffDist(gt2d_BIF_OB(i,:),pos2d_BIF_OB_1(i,:)) ;
            hd2d_BIF_OB_2(i)=HausdorffDist(gt2d_BIF_OB(i,:),pos2d_BIF_OB_2(i,:)) ;
                hd2d_BIF_OB_3(i)=HausdorffDist(gt2d_BIF_OB(i,:),pos2d_BIF_OB_3(i,:)) ;
    hd2d_BIF_SIDE_0(i)=HausdorffDist(gt2d_BIF_SIDE(i,:),pos2d_BIF_SIDE_0(i,:)) ;
        hd2d_BIF_SIDE_1(i)=HausdorffDist(gt2d_BIF_SIDE(i,:),pos2d_BIF_SIDE_1(i,:)) ;
            hd2d_BIF_SIDE_2(i)=HausdorffDist(gt2d_BIF_SIDE(i,:),pos2d_BIF_SIDE_2(i,:)) ;
                hd2d_BIF_SIDE_3(i)=HausdorffDist(gt2d_BIF_SIDE(i,:),pos2d_BIF_SIDE_3(i,:)) ;
end
%% RMSE2d TIP 3D %%

RMSE_BIF_TOP_0=ones(msBIF,1);
RMSE_BIF_TOP_1=ones(msBIF,1);
RMSE_BIF_TOP_2=ones(msBIF,1);
RMSE_BIF_TOP_3=ones(msBIF,1);
RMSE_BIF_SIDE_0=ones(msBIF,1);
RMSE_BIF_SIDE_1=ones(msBIF,1);
RMSE_BIF_SIDE_2=ones(msBIF,1);
RMSE_BIF_SIDE_3=ones(msBIF,1);
RMSE_BIF_OB_0=ones(msBIF,1);
RMSE_BIF_OB_1=ones(msBIF,1);
RMSE_BIF_OB_2=ones(msBIF,1);
RMSE_BIF_OB_3=ones(msBIF,1);

RMSE_TEST_TOP_0=ones(msTEST,1);
RMSE_TEST_TOP_1=ones(msTEST,1);
RMSE_TEST_TOP_2=ones(msTEST,1);
RMSE_TEST_TOP_3=ones(msTEST,1);
RMSE_TEST_SIDE_0=ones(msTEST,1);
RMSE_TEST_SIDE_1=ones(msTEST,1);
RMSE_TEST_SIDE_2=ones(msTEST,1);
RMSE_TEST_SIDE_3=ones(msTEST,1);
RMSE_TEST_OB_0=ones(msTEST,1);
RMSE_TEST_OB_1=ones(msTEST,1);
RMSE_TEST_OB_2=ones(msTEST,1);
RMSE_TEST_OB_3=ones(msTEST,1);


for i =1:msBIF
 RMSE_BIF_TOP_0(i)= sqrt(mean((norm(gt_BIF_TOP(i,1:3)) - norm(pos_BIF_TOP_0(i,1:3))).^2));
    RMSE_BIF_TOP_1(i)= sqrt(mean((norm(gt_BIF_TOP(i,1:3)) - norm(pos_BIF_TOP_1(i,1:3))).^2));
       RMSE_BIF_TOP_2(i)= sqrt(mean((norm(gt_BIF_TOP(i,1:3)) - norm(pos_BIF_TOP_2(i,1:3))).^2));
       
 RMSE_BIF_OB_0(i)= sqrt(mean((norm(gt_BIF_OB(i,1:3)) - norm(pos_BIF_OB_0(i,1:3))).^2));
    RMSE_BIF_OB_1(i)= sqrt(mean((norm(gt_BIF_OB(i,1:3)) - norm(pos_BIF_OB_1(i,1:3))).^2));
       RMSE_BIF_OB_2(i)= sqrt(mean((norm(gt_BIF_OB(i,1:3)) - norm(pos_BIF_OB_2(i,1:3))).^2));
       
 RMSE_BIF_SIDE_0(i)= sqrt(mean((norm(gt_BIF_SIDE(i,1:3)) - norm(pos_BIF_SIDE_0(i,1:3))).^2));
    RMSE_BIF_SIDE_1(i)= sqrt(mean((norm(gt_BIF_SIDE(i,1:3)) - norm(pos_BIF_SIDE_1(i,1:3))).^2));
       RMSE_BIF_SIDE_2(i)= sqrt(mean((norm(gt_BIF_SIDE(i,1:3)) - norm(pos_BIF_SIDE_2(i,1:3))).^2));
end


for i =1:msTEST
 RMSE_TEST_TOP_0(i)= sqrt(mean((norm(gt_TEST_TOP(i,1:3)) - norm(pos_TEST_TOP_0(i,1:3))).^2));
    RMSE_TEST_TOP_1(i)= sqrt(mean((norm(gt_TEST_TOP(i,1:3)) - norm(pos_TEST_TOP_1(i,1:3))).^2));
       RMSE_TEST_TOP_2(i)= sqrt(mean((norm(gt_TEST_TOP(i,1:3)) - norm(pos_TEST_TOP_2(i,1:3))).^2));
       
 RMSE_TEST_OB_0(i)= sqrt(mean((norm(gt_TEST_OB(i,1:3)) - norm(pos_TEST_OB_0(i,1:3))).^2));
    RMSE_TEST_OB_1(i)= sqrt(mean((norm(gt_TEST_OB(i,1:3)) - norm(pos_TEST_OB_1(i,1:3))).^2));
       RMSE_TEST_OB_2(i)= sqrt(mean((norm(gt_TEST_OB(i,1:3)) - norm(pos_TEST_OB_2(i,1:3))).^2));
       
 RMSE_TEST_SIDE_0(i)= sqrt(mean((norm(gt_TEST_SIDE(i,1:3)) - norm(pos_TEST_SIDE_0(i,1:3))).^2));
    RMSE_TEST_SIDE_1(i)= sqrt(mean((norm(gt_TEST_SIDE(i,1:3)) - norm(pos_TEST_SIDE_1(i,1:3))).^2));
       RMSE_TEST_SIDE_2(i)= sqrt(mean((norm(gt_TEST_SIDE(i,1:3)) - norm(pos_TEST_SIDE_2(i,1:3))).^2));
end

%% RMSE TIP 2D %%

RMSE2d_BIF_TOP_0=ones(msBIF,1);
RMSE2d_BIF_TOP_1=ones(msBIF,1);
RMSE2d_BIF_TOP_2=ones(msBIF,1);
RMSE2d_BIF_TOP_3=ones(msBIF,1);
RMSE2d_BIF_SIDE_0=ones(msBIF,1);
RMSE2d_BIF_SIDE_1=ones(msBIF,1);
RMSE2d_BIF_SIDE_2=ones(msBIF,1);
RMSE2d_BIF_SIDE_3=ones(msBIF,1);
RMSE2d_BIF_OB_0=ones(msBIF,1);
RMSE2d_BIF_OB_1=ones(msBIF,1);
RMSE2d_BIF_OB_2=ones(msBIF,1);
RMSE2d_BIF_OB_3=ones(msBIF,1);

RMSE2d_TEST_TOP_0=ones(msTEST,1);
RMSE2d_TEST_TOP_1=ones(msTEST,1);
RMSE2d_TEST_TOP_2=ones(msTEST,1);
RMSE2d_TEST_TOP_3=ones(msTEST,1);
RMSE2d_TEST_SIDE_0=ones(msTEST,1);
RMSE2d_TEST_SIDE_1=ones(msTEST,1);
RMSE2d_TEST_SIDE_2=ones(msTEST,1);
RMSE2d_TEST_SIDE_3=ones(msTEST,1);
RMSE2d_TEST_OB_0=ones(msTEST,1);
RMSE2d_TEST_OB_1=ones(msTEST,1);
RMSE2d_TEST_OB_2=ones(msTEST,1);
RMSE2d_TEST_OB_3=ones(msTEST,1);


for i =1:msBIF
 RMSE2d_BIF_TOP_0(i)= sqrt(mean((norm(gt2d_BIF_TOP(i,1:3)) - norm(pos2d_BIF_TOP_0(i,1:3))).^2));
    RMSE2d_BIF_TOP_1(i)= sqrt(mean((norm(gt2d_BIF_TOP(i,1:3)) - norm(pos2d_BIF_TOP_1(i,1:3))).^2));
       RMSE2d_BIF_TOP_2(i)= sqrt(mean((norm(gt2d_BIF_TOP(i,1:3)) - norm(pos2d_BIF_TOP_2(i,1:3))).^2));
       
 RMSE2d_BIF_OB_0(i)= sqrt(mean((norm(gt2d_BIF_OB(i,1:3)) - norm(pos2d_BIF_OB_0(i,1:3))).^2));
    RMSE2d_BIF_OB_1(i)= sqrt(mean((norm(gt2d_BIF_OB(i,1:3)) - norm(pos2d_BIF_OB_1(i,1:3))).^2));
       RMSE2d_BIF_OB_2(i)= sqrt(mean((norm(gt2d_BIF_OB(i,1:3)) - norm(pos2d_BIF_OB_2(i,1:3))).^2));
       
 RMSE2d_BIF_SIDE_0(i)= sqrt(mean((norm(gt2d_BIF_SIDE(i,1:3)) - norm(pos2d_BIF_SIDE_0(i,1:3))).^2));
    RMSE2d_BIF_SIDE_1(i)= sqrt(mean((norm(gt2d_BIF_SIDE(i,1:3)) - norm(pos2d_BIF_SIDE_1(i,1:3))).^2));
       RMSE2d_BIF_SIDE_2(i)= sqrt(mean((norm(gt2d_BIF_SIDE(i,1:3)) - norm(pos2d_BIF_SIDE_2(i,1:3))).^2));
end


for i =1:msTEST
 RMSE2d_TEST_TOP_0(i)= sqrt(mean((norm(gt2d_TEST_TOP(i,1:3)) - norm(pos2d_TEST_TOP_0(i,1:3))).^2));
    RMSE2d_TEST_TOP_1(i)= sqrt(mean((norm(gt2d_TEST_TOP(i,1:3)) - norm(pos2d_TEST_TOP_1(i,1:3))).^2));
       RMSE2d_TEST_TOP_2(i)= sqrt(mean((norm(gt2d_TEST_TOP(i,1:3)) - norm(pos2d_TEST_TOP_2(i,1:3))).^2));
       
 RMSE2d_TEST_OB_0(i)= sqrt(mean((norm(gt2d_TEST_OB(i,1:3)) - norm(pos2d_TEST_OB_0(i,1:3))).^2));
    RMSE2d_TEST_OB_1(i)= sqrt(mean((norm(gt2d_TEST_OB(i,1:3)) - norm(pos2d_TEST_OB_1(i,1:3))).^2));
       RMSE2d_TEST_OB_2(i)= sqrt(mean((norm(gt2d_TEST_OB(i,1:3)) - norm(pos2d_TEST_OB_2(i,1:3))).^2));
       
 RMSE2d_TEST_SIDE_0(i)= sqrt(mean((norm(gt2d_TEST_SIDE(i,1:3)) - norm(pos2d_TEST_SIDE_0(i,1:3))).^2));
    RMSE2d_TEST_SIDE_1(i)= sqrt(mean((norm(gt2d_TEST_SIDE(i,1:3)) - norm(pos2d_TEST_SIDE_1(i,1:3))).^2));
       RMSE2d_TEST_SIDE_2(i)= sqrt(mean((norm(gt2d_TEST_SIDE(i,1:3)) - norm(pos2d_TEST_SIDE_2(i,1:3))).^2));
end


%% SAVE DATA 

save('hd2d_BIF_SIDE_0.mat','hd2d_BIF_SIDE_0'); save('hd2d_BIF_SIDE_1.mat','hd2d_BIF_SIDE_1'); save('hd2d_BIF_SIDE_2.mat','hd2d_BIF_SIDE_2'); save('hd2d_BIF_SIDE_3.mat','hd2d_BIF_SIDE_3');  
save('hd2d_BIF_TOP_0.mat','hd2d_BIF_TOP_0'); save('hd2d_BIF_TOP_1.mat','hd2d_BIF_TOP_1'); save('hd2d_BIF_TOP_2.mat','hd2d_BIF_TOP_2'); save('hd2d_BIF_TOP_3.mat','hd2d_BIF_TOP_3');  
save('hd2d_BIF_OB_0.mat','hd2d_BIF_OB_0'); save('hd2d_BIF_OB_1.mat','hd2d_BIF_OB_1'); save('hd2d_BIF_OB_2.mat','hd2d_BIF_OB_2'); save('hd2d_BIF_OB_3.mat','hd2d_BIF_OB_3');  

save('hd_BIF_SIDE_0.mat','hd_BIF_SIDE_0'); save('hd_BIF_SIDE_1.mat','hd_BIF_SIDE_1'); save('hd_BIF_SIDE_2.mat','hd_BIF_SIDE_2'); save('hd_BIF_SIDE_3.mat','hd_BIF_SIDE_3');
save('hd_BIF_TOP_0.mat','hd_BIF_TOP_0'); save('hd_BIF_TOP_1.mat','hd_BIF_TOP_1'); save('hd_BIF_TOP_2.mat','hd_BIF_TOP_2'); save('hd_BIF_TOP_3.mat','hd_BIF_TOP_3');
save('hd_BIF_OB_0.mat','hd_BIF_OB_0'); save('hd_BIF_OB_1.mat','hd_BIF_OB_1'); save('hd_BIF_OB_2.mat','hd_BIF_OB_2'); save('hd_BIF_OB_3.mat','hd_BIF_OB_3');

save('d_BIF_SIDE_0.mat','d_BIF_SIDE_0'); save('d_BIF_SIDE_1.mat','d_BIF_SIDE_1'); save('d_BIF_SIDE_2.mat','d_BIF_SIDE_2'); save('d_BIF_SIDE_3.mat','d_BIF_SIDE_3');
save('d_BIF_TOP_0.mat','d_BIF_TOP_0'); save('d_BIF_TOP_1.mat','d_BIF_TOP_1'); save('d_BIF_TOP_2.mat','d_BIF_TOP_2'); save('d_BIF_TOP_3.mat','d_BIF_TOP_3');
save('d_BIF_OB_0.mat','d_BIF_OB_0'); save('d_BIF_OB_1.mat','d_BIF_OB_1'); save('d_BIF_OB_2.mat','d_BIF_OB_2'); save('d_BIF_OB_3.mat','d_BIF_OB_3');

save('d2d_BIF_SIDE_0.mat','d2d_BIF_SIDE_0'); save('d2d_BIF_SIDE_1.mat','d2d_BIF_SIDE_1'); save('d2d_BIF_SIDE_2.mat','d2d_BIF_SIDE_2'); save('d2d_BIF_SIDE_3.mat','d2d_BIF_SIDE_3');
save('d2d_BIF_TOP_0.mat','d2d_BIF_TOP_0'); save('d2d_BIF_TOP_1.mat','d2d_BIF_TOP_1'); save('d2d_BIF_TOP_2.mat','d2d_BIF_TOP_2'); save('d2d_BIF_TOP_3.mat','d2d_BIF_TOP_3');
save('d2d_BIF_OB_0.mat','d2d_BIF_OB_0'); save('d2d_BIF_OB_1.mat','d2d_BIF_OB_1'); save('d2d_BIF_OB_2.mat','d2d_BIF_OB_2'); save('d2d_BIF_OB_3.mat','d2d_BIF_OB_3');

save('hd2d_TEST_SIDE_0.mat','hd2d_TEST_SIDE_0'); save('hd2d_TEST_SIDE_1.mat','hd2d_TEST_SIDE_1'); save('hd2d_TEST_SIDE_2.mat','hd2d_TEST_SIDE_2'); save('hd2d_TEST_SIDE_3.mat','hd2d_TEST_SIDE_3');
save('hd2d_TEST_TOP_0.mat','hd2d_TEST_TOP_0'); save('hd2d_TEST_TOP_1.mat','hd2d_TEST_TOP_1'); save('hd2d_TEST_TOP_2.mat','hd2d_TEST_TOP_2'); save('hd2d_TEST_TOP_3.mat','hd2d_TEST_TOP_3');
save('hd2d_TEST_OB_0.mat','hd2d_TEST_OB_0'); save('hd2d_TEST_OB_1.mat','hd2d_TEST_OB_1'); save('hd2d_TEST_OB_2.mat','hd2d_TEST_OB_2'); save('hd2d_TEST_OB_3.mat','hd2d_TEST_OB_3');

save('hd_TEST_SIDE_0.mat','hd_TEST_SIDE_0'); save('hd_TEST_SIDE_1.mat','hd_TEST_SIDE_1'); save('hd_TEST_SIDE_2.mat','hd_TEST_SIDE_2'); save('hd_TEST_SIDE_3.mat','hd_TEST_SIDE_3');
save('hd_TEST_TOP_0.mat','hd_TEST_TOP_0'); save('hd_TEST_TOP_1.mat','hd_TEST_TOP_1'); save('hd_TEST_TOP_2.mat','hd_TEST_TOP_2'); save('hd_TEST_TOP_3.mat','hd_TEST_TOP_3');
save('hd_TEST_OB_0.mat','hd_TEST_OB_0'); save('hd_TEST_OB_1.mat','hd_TEST_OB_1'); save('hd_TEST_OB_2.mat','hd_TEST_OB_2'); save('hd_TEST_OB_3.mat','hd_TEST_OB_3');

save('d_TEST_SIDE_0.mat','d_TEST_SIDE_0'); save('d_TEST_SIDE_1.mat','d_TEST_SIDE_1'); save('d_TEST_SIDE_2.mat','d_TEST_SIDE_2'); save('d_TEST_SIDE_3.mat','d_TEST_SIDE_3');
save('d_TEST_TOP_0.mat','d_TEST_TOP_0'); save('d_TEST_TOP_1.mat','d_TEST_TOP_1'); save('d_TEST_TOP_2.mat','d_TEST_TOP_2'); save('d_TEST_TOP_3.mat','d_TEST_TOP_3');
save('d_TEST_OB_0.mat','d_TEST_OB_0'); save('d_TEST_OB_1.mat','d_TEST_OB_1'); save('d_TEST_OB_2.mat','d_TEST_OB_2'); save('d_TEST_OB_3.mat','d_TEST_OB_3');

save('d2d_TEST_SIDE_0.mat','d2d_TEST_SIDE_0'); save('d2d_TEST_SIDE_1.mat','d2d_TEST_SIDE_1'); save('d2d_TEST_SIDE_2.mat','d2d_TEST_SIDE_2'); save('d2d_TEST_SIDE_3.mat','d2d_TEST_SIDE_3');
save('d2d_TEST_TOP_0.mat','d2d_TEST_TOP_0'); save('d2d_TEST_TOP_1.mat','d2d_TEST_TOP_1'); save('d2d_TEST_TOP_2.mat','d2d_TEST_TOP_2'); save('d2d_TEST_TOP_3.mat','d2d_TEST_TOP_3');
save('d2d_TEST_OB_0.mat','d2d_TEST_OB_0'); save('d2d_TEST_OB_1.mat','d2d_TEST_OB_1'); save('d2d_TEST_OB_2.mat','d2d_TEST_OB_2'); save('d2d_TEST_OB_3.mat','d2d_TEST_OB_3');

save('RMSE2d_TEST_SIDE_0.mat','RMSE2d_TEST_SIDE_0'); save('RMSE2d_TEST_SIDE_1.mat','RMSE2d_TEST_SIDE_1'); save('RMSE2d_TEST_SIDE_2.mat','RMSE2d_TEST_SIDE_2'); save('RMSE2d_TEST_SIDE_3.mat','RMSE2d_TEST_SIDE_3');
save('RMSE2d_TEST_TOP_0.mat','RMSE2d_TEST_TOP_0'); save('RMSE2d_TEST_TOP_1.mat','RMSE2d_TEST_TOP_1'); save('RMSE2d_TEST_TOP_2.mat','RMSE2d_TEST_TOP_2'); save('RMSE2d_TEST_TOP_3.mat','RMSE2d_TEST_TOP_3');
save('RMSE2d_TEST_OB_0.mat','RMSE2d_TEST_OB_0'); save('RMSE2d_TEST_OB_1.mat','RMSE2d_TEST_OB_1'); save('RMSE2d_TEST_OB_2.mat','RMSE2d_TEST_OB_2'); save('RMSE2d_TEST_OB_3.mat','RMSE2d_TEST_OB_3');

save('RMSE_TEST_SIDE_0.mat','RMSE_TEST_SIDE_0'); save('RMSE_TEST_SIDE_1.mat','RMSE_TEST_SIDE_1'); save('RMSE_TEST_SIDE_2.mat','RMSE_TEST_SIDE_2'); save('RMSE_TEST_SIDE_3.mat','RMSE_TEST_SIDE_3');
save('RMSE_TEST_TOP_0.mat','RMSE_TEST_TOP_0'); save('RMSE_TEST_TOP_1.mat','RMSE_TEST_TOP_1'); save('RMSE_TEST_TOP_2.mat','RMSE_TEST_TOP_2'); save('RMSE_TEST_TOP_3.mat','RMSE_TEST_TOP_3');
save('RMSE_TEST_OB_0.mat','RMSE_TEST_OB_0'); save('RMSE_TEST_OB_1.mat','RMSE_TEST_OB_1'); save('RMSE_TEST_OB_2.mat','RMSE_TEST_OB_2'); save('RMSE_TEST_OB_3.mat','RMSE_TEST_OB_3');


