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
P_BIF_TOP=[775.552 -206.837 200.739 202.551,43.6739 289.403 727.253 109.588,0.148221 -0.711592 0.68678 0.348575]; %BIFSIDE
P_BIF_SIDE=[770.277 -32.7486 -300.267 225.406,217.307 -715.345 235.834 98.5108,0.194673 -0.612749 -0.765925 0.372637]; %BIFSIDE
P_BIF_OB=[513.41 148.368 631.63 215.592,-193.229 -585.209 484.511 25.2104,0.760325 -0.628768 0.16296 0.445612]; %BIFOB
P_TEST_TOP=[19.8421 -404.504 721.763 143.041,-708.31 -335.931 -2.3288 68.0737,0.0501423 -0.998723 -0.00620961 0.362366]; %TESTSIDE
P_TEST_OB=[560.532 -169.605 584.463 149.275,-87.0633 -776.497 63.4843 14.5066,0.827775 -0.499977 -0.254581 0.286321]; %TESTOB
P_TEST_SIDE=[2281.11 -2.34947e-13 -400 273.957,-3.07529e-13 -2281.11 -300 -6.35711,-9.39116e-17 1.11022e-16 -1 0.317505]; %TESTSIDE



%load State%
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
posBIF_TOP_0=...
    cat(2,stateREAD_BIF_TOP_0(:,1:3),stateREAD_BIF_TOP_0(:,7:9),stateREAD_BIF_TOP_0(:,13:15),stateREAD_BIF_TOP_0(:,19:21),stateREAD_BIF_TOP_0(:,25:27),stateREAD_BIF_TOP_0(:,31:33),...
    stateREAD_BIF_TOP_0(:,37:39),stateREAD_BIF_TOP_0(:,43:45),stateREAD_BIF_TOP_0(:,49:51),stateREAD_BIF_TOP_0(:,55:57));
posBIF_TOP_1=...
    cat(2,stateREAD_BIF_TOP_1(:,1:3),stateREAD_BIF_TOP_1(:,7:9),stateREAD_BIF_TOP_1(:,13:15),stateREAD_BIF_TOP_1(:,19:21),stateREAD_BIF_TOP_1(:,25:27),stateREAD_BIF_TOP_1(:,31:33),...
    stateREAD_BIF_TOP_1(:,37:39),stateREAD_BIF_TOP_1(:,43:45),stateREAD_BIF_TOP_1(:,49:51),stateREAD_BIF_TOP_1(:,55:57));
posBIF_TOP_2=...
    cat(2,stateREAD_BIF_TOP_2(:,1:3),stateREAD_BIF_TOP_2(:,7:9),stateREAD_BIF_TOP_2(:,13:15),stateREAD_BIF_TOP_2(:,19:21),stateREAD_BIF_TOP_2(:,25:27),stateREAD_BIF_TOP_2(:,31:33),...
    stateREAD_BIF_TOP_2(:,37:39),stateREAD_BIF_TOP_2(:,43:45),stateREAD_BIF_TOP_2(:,49:51),stateREAD_BIF_TOP_2(:,55:57));
posBIF_TOP_3=...
    cat(2,stateREAD_BIF_TOP_3(:,1:3),stateREAD_BIF_TOP_3(:,7:9),stateREAD_BIF_TOP_3(:,13:15),stateREAD_BIF_TOP_3(:,19:21),stateREAD_BIF_TOP_3(:,25:27),stateREAD_BIF_TOP_3(:,31:33),...
    stateREAD_BIF_TOP_3(:,37:39),stateREAD_BIF_TOP_3(:,43:45),stateREAD_BIF_TOP_3(:,49:51),stateREAD_BIF_TOP_3(:,55:57));


posBIF_OB_0=...
    cat(2,stateREAD_BIF_OB_0(:,1:3),stateREAD_BIF_OB_0(:,7:9),stateREAD_BIF_OB_0(:,13:15),stateREAD_BIF_OB_0(:,19:21),stateREAD_BIF_OB_0(:,25:27),stateREAD_BIF_OB_0(:,31:33),...
    stateREAD_BIF_OB_0(:,37:39),stateREAD_BIF_OB_0(:,43:45),stateREAD_BIF_OB_0(:,49:51),stateREAD_BIF_OB_0(:,55:57));
posBIF_OB_1=...
    cat(2,stateREAD_BIF_OB_1(:,1:3),stateREAD_BIF_OB_1(:,7:9),stateREAD_BIF_OB_1(:,13:15),stateREAD_BIF_OB_1(:,19:21),stateREAD_BIF_OB_1(:,25:27),stateREAD_BIF_OB_1(:,31:33),...
    stateREAD_BIF_OB_1(:,37:39),stateREAD_BIF_OB_1(:,43:45),stateREAD_BIF_OB_1(:,49:51),stateREAD_BIF_OB_1(:,55:57));
posBIF_OB_2=...
    cat(2,stateREAD_BIF_OB_2(:,1:3),stateREAD_BIF_OB_2(:,7:9),stateREAD_BIF_OB_2(:,13:15),stateREAD_BIF_OB_2(:,19:21),stateREAD_BIF_OB_2(:,25:27),stateREAD_BIF_OB_2(:,31:33),...
    stateREAD_BIF_OB_2(:,37:39),stateREAD_BIF_OB_2(:,43:45),stateREAD_BIF_OB_2(:,49:51),stateREAD_BIF_OB_2(:,55:57));
posBIF_OB_3=...
    cat(2,stateREAD_BIF_OB_3(:,1:3),stateREAD_BIF_OB_3(:,7:9),stateREAD_BIF_OB_3(:,13:15),stateREAD_BIF_OB_3(:,19:21),stateREAD_BIF_OB_3(:,25:27),stateREAD_BIF_OB_3(:,31:33),...
    stateREAD_BIF_OB_3(:,37:39),stateREAD_BIF_OB_3(:,43:45),stateREAD_BIF_OB_3(:,49:51),stateREAD_BIF_OB_3(:,55:57));

posBIF_SIDE_0=...
    cat(2,stateREAD_BIF_SIDE_0(:,1:3),stateREAD_BIF_SIDE_0(:,7:9),stateREAD_BIF_SIDE_0(:,13:15),stateREAD_BIF_SIDE_0(:,19:21),stateREAD_BIF_SIDE_0(:,25:27),stateREAD_BIF_SIDE_0(:,31:33),...
    stateREAD_BIF_SIDE_0(:,37:39),stateREAD_BIF_SIDE_0(:,43:45),stateREAD_BIF_SIDE_0(:,49:51),stateREAD_BIF_SIDE_0(:,55:57));
posBIF_SIDE_1=...
    cat(2,stateREAD_BIF_SIDE_1(:,1:3),stateREAD_BIF_SIDE_1(:,7:9),stateREAD_BIF_SIDE_1(:,13:15),stateREAD_BIF_SIDE_1(:,19:21),stateREAD_BIF_SIDE_1(:,25:27),stateREAD_BIF_SIDE_1(:,31:33),...
    stateREAD_BIF_SIDE_1(:,37:39),stateREAD_BIF_SIDE_1(:,43:45),stateREAD_BIF_SIDE_1(:,49:51),stateREAD_BIF_SIDE_1(:,55:57));
posBIF_SIDE_2=...
    cat(2,stateREAD_BIF_SIDE_2(:,1:3),stateREAD_BIF_SIDE_2(:,7:9),stateREAD_BIF_SIDE_2(:,13:15),stateREAD_BIF_SIDE_2(:,19:21),stateREAD_BIF_SIDE_2(:,25:27),stateREAD_BIF_SIDE_2(:,31:33),...
    stateREAD_BIF_SIDE_2(:,37:39),stateREAD_BIF_SIDE_2(:,43:45),stateREAD_BIF_SIDE_2(:,49:51),stateREAD_BIF_SIDE_2(:,55:57));
posBIF_SIDE_3=...
    cat(2,stateREAD_BIF_SIDE_3(:,1:3),stateREAD_BIF_SIDE_3(:,7:9),stateREAD_BIF_SIDE_3(:,13:15),stateREAD_BIF_SIDE_3(:,19:21),stateREAD_BIF_SIDE_3(:,25:27),stateREAD_BIF_SIDE_3(:,31:33),...
    stateREAD_BIF_SIDE_3(:,37:39),stateREAD_BIF_SIDE_3(:,43:45),stateREAD_BIF_SIDE_3(:,49:51),stateREAD_BIF_SIDE_3(:,55:57));
%%

posTEST_TOP_0=...
    cat(2,stateREAD_TEST_TOP_0(:,1:3),stateREAD_TEST_TOP_0(:,7:9),stateREAD_TEST_TOP_0(:,13:15),stateREAD_TEST_TOP_0(:,19:21),stateREAD_TEST_TOP_0(:,25:27),stateREAD_TEST_TOP_0(:,31:33),...
    stateREAD_TEST_TOP_0(:,37:39),stateREAD_TEST_TOP_0(:,43:45),stateREAD_TEST_TOP_0(:,49:51),stateREAD_TEST_TOP_0(:,55:57));
posTEST_TOP_1=...
    cat(2,stateREAD_TEST_TOP_1(:,1:3),stateREAD_TEST_TOP_1(:,7:9),stateREAD_TEST_TOP_1(:,13:15),stateREAD_TEST_TOP_1(:,19:21),stateREAD_TEST_TOP_1(:,25:27),stateREAD_TEST_TOP_1(:,31:33),...
    stateREAD_TEST_TOP_1(:,37:39),stateREAD_TEST_TOP_1(:,43:45),stateREAD_TEST_TOP_1(:,49:51),stateREAD_TEST_TOP_1(:,55:57));
posTEST_TOP_2=...
    cat(2,stateREAD_TEST_TOP_2(:,1:3),stateREAD_TEST_TOP_2(:,7:9),stateREAD_TEST_TOP_2(:,13:15),stateREAD_TEST_TOP_2(:,19:21),stateREAD_TEST_TOP_2(:,25:27),stateREAD_TEST_TOP_2(:,31:33),...
    stateREAD_TEST_TOP_2(:,37:39),stateREAD_TEST_TOP_2(:,43:45),stateREAD_TEST_TOP_2(:,49:51),stateREAD_TEST_TOP_2(:,55:57));
posTEST_TOP_3=...
    cat(2,stateREAD_TEST_TOP_3(:,1:3),stateREAD_TEST_TOP_3(:,7:9),stateREAD_TEST_TOP_3(:,13:15),stateREAD_TEST_TOP_3(:,19:21),stateREAD_TEST_TOP_3(:,25:27),stateREAD_TEST_TOP_3(:,31:33),...
    stateREAD_TEST_TOP_3(:,37:39),stateREAD_TEST_TOP_3(:,43:45),stateREAD_TEST_TOP_3(:,49:51),stateREAD_TEST_TOP_3(:,55:57));


posTEST_OB_0=...
    cat(2,stateREAD_TEST_OB_0(:,1:3),stateREAD_TEST_OB_0(:,7:9),stateREAD_TEST_OB_0(:,13:15),stateREAD_TEST_OB_0(:,19:21),stateREAD_TEST_OB_0(:,25:27),stateREAD_TEST_OB_0(:,31:33),...
    stateREAD_TEST_OB_0(:,37:39),stateREAD_TEST_OB_0(:,43:45),stateREAD_TEST_OB_0(:,49:51),stateREAD_TEST_OB_0(:,55:57));
posTEST_OB_1=...
    cat(2,stateREAD_TEST_OB_1(:,1:3),stateREAD_TEST_OB_1(:,7:9),stateREAD_TEST_OB_1(:,13:15),stateREAD_TEST_OB_1(:,19:21),stateREAD_TEST_OB_1(:,25:27),stateREAD_TEST_OB_1(:,31:33),...
    stateREAD_TEST_OB_1(:,37:39),stateREAD_TEST_OB_1(:,43:45),stateREAD_TEST_OB_1(:,49:51),stateREAD_TEST_OB_1(:,55:57));
posTEST_OB_2=...
    cat(2,stateREAD_TEST_OB_2(:,1:3),stateREAD_TEST_OB_2(:,7:9),stateREAD_TEST_OB_2(:,13:15),stateREAD_TEST_OB_2(:,19:21),stateREAD_TEST_OB_2(:,25:27),stateREAD_TEST_OB_2(:,31:33),...
    stateREAD_TEST_OB_2(:,37:39),stateREAD_TEST_OB_2(:,43:45),stateREAD_TEST_OB_2(:,49:51),stateREAD_TEST_OB_2(:,55:57));
posTEST_OB_3=...
    cat(2,stateREAD_TEST_OB_3(:,1:3),stateREAD_TEST_OB_3(:,7:9),stateREAD_TEST_OB_3(:,13:15),stateREAD_TEST_OB_3(:,19:21),stateREAD_TEST_OB_3(:,25:27),stateREAD_TEST_OB_3(:,31:33),...
    stateREAD_TEST_OB_3(:,37:39),stateREAD_TEST_OB_3(:,43:45),stateREAD_TEST_OB_3(:,49:51),stateREAD_TEST_OB_3(:,55:57));

posTEST_SIDE_0=...
    cat(2,stateREAD_TEST_SIDE_0(:,1:3),stateREAD_TEST_SIDE_0(:,7:9),stateREAD_TEST_SIDE_0(:,13:15),stateREAD_TEST_SIDE_0(:,19:21),stateREAD_TEST_SIDE_0(:,25:27),stateREAD_TEST_SIDE_0(:,31:33),...
    stateREAD_TEST_SIDE_0(:,37:39),stateREAD_TEST_SIDE_0(:,43:45),stateREAD_TEST_SIDE_0(:,49:51),stateREAD_TEST_SIDE_0(:,55:57));
posTEST_SIDE_1=...
    cat(2,stateREAD_TEST_SIDE_1(:,1:3),stateREAD_TEST_SIDE_1(:,7:9),stateREAD_TEST_SIDE_1(:,13:15),stateREAD_TEST_SIDE_1(:,19:21),stateREAD_TEST_SIDE_1(:,25:27),stateREAD_TEST_SIDE_1(:,31:33),...
    stateREAD_TEST_SIDE_1(:,37:39),stateREAD_TEST_SIDE_1(:,43:45),stateREAD_TEST_SIDE_1(:,49:51),stateREAD_TEST_SIDE_1(:,55:57));
posTEST_SIDE_2=...
    cat(2,stateREAD_TEST_SIDE_2(:,1:3),stateREAD_TEST_SIDE_2(:,7:9),stateREAD_TEST_SIDE_2(:,13:15),stateREAD_TEST_SIDE_2(:,19:21),stateREAD_TEST_SIDE_2(:,25:27),stateREAD_TEST_SIDE_2(:,31:33),...
    stateREAD_TEST_SIDE_2(:,37:39),stateREAD_TEST_SIDE_2(:,43:45),stateREAD_TEST_SIDE_2(:,49:51),stateREAD_TEST_SIDE_2(:,55:57));
posTEST_SIDE_3=...
    cat(2,stateREAD_TEST_SIDE_3(:,1:3),stateREAD_TEST_SIDE_3(:,7:9),stateREAD_TEST_SIDE_3(:,13:15),stateREAD_TEST_SIDE_3(:,19:21),stateREAD_TEST_SIDE_3(:,25:27),stateREAD_TEST_SIDE_3(:,31:33),...
    stateREAD_TEST_SIDE_3(:,37:39),stateREAD_TEST_SIDE_3(:,43:45),stateREAD_TEST_SIDE_3(:,49:51),stateREAD_TEST_SIDE_3(:,55:57));

%%
msBIF=1230;
msTEST=2900;

gt2d_BIF_TOP=ones(ms,20);
gt2d_BIF_TOP=proj(gt_BIF_TOP,P_BIF_TOP,msBIF)
pos2d_BIF_TOP_0=ones(ms,20);
pos2d_BIF_TOP_0=proj(pos_BIF_TOP_0,P_BIF_TOP,msBIF)







tip_gt=gt(:,1:3);
tip_filter=pos(:,1:3);

ms=2445;
A=ones(ms,2);
RMSE=ones(ms,1);
for i =1:ms
   A(i,1)=norm(tip_gt(i,:));
   A(i,2)=norm(tip_filter(i,:));
  RMSE(i)= sqrt(mean((A(i,1) - A(i,2)).^2));
 
end

pos2d=ones(ms,20);
gt2d=ones(ms,20);


  for i =1:ms
      rx = P(1,1) * pos(i,1) + P(1,2) * pos(i,2) + P(1,3) * pos(i,3) + P(1,4);
      ry = P(2,1) * pos(i,1) + P(2,2) * pos(i,2) + P(2,3) * pos(i,3) + P(2,4);
      rz = P(3,1) * pos(i,1) + P(3,2) * pos(i,2) + P(3,3) * pos(i,3) + P(3,4);
      pos2d(i,1)=rx* (1.0/rz);
      pos2d(i,2)=ry* (1.0/rz);    
            
      grx = P(1,1) * gt(i,1) + P(1,2) * gt(i,2) + P(1,3) * gt(i,3) + P(1,4);
      gry = P(2,1) * gt(i,1) + P(2,2) * gt(i,2) + P(2,3) * gt(i,3) + P(2,4);
      grz = P(3,1) * gt(i,1) + P(3,2) * gt(i,2) + P(3,3) * gt(i,3) + P(3,4);
      gt2d(i,1)=grx* (1.0/grz);
      gt2d(i,2)=gry* (1.0/grz);               
  end


    for i =1:ms
      rx = P(1,1) * pos(i,4) + P(1,2) * pos(i,5) + P(1,3) * pos(i,6) + P(1,4);
      ry = P(2,1) * pos(i,4) + P(2,2) * pos(i,5) + P(2,3) * pos(i,6) + P(2,4);
      rz = P(3,1) * pos(i,4) + P(3,2) * pos(i,5) + P(3,3) * pos(i,6) + P(3,4);
      pos2d(i,3)=rx* (1.0/rz);
      pos2d(i,4)=ry* (1.0/rz);    
      
      grx = P(1,1) * gt(i,4) + P(1,2) * gt(i,5) + P(1,3) * gt(i,6) + P(1,4);
      gry = P(2,1) * gt(i,4) + P(2,2) * gt(i,5) + P(2,3) * gt(i,6) + P(2,4);
      grz = P(3,1) * gt(i,4) + P(3,2) * gt(i,5) + P(3,3) * gt(i,6) + P(3,4);
      gt2d(i,3)=grx* (1.0/grz);
      gt2d(i,4)=gry* (1.0/grz);               
    end
  
    
      for i =1:ms
      rx = P(1,1) * pos(i,7) + P(1,2) * pos(i,8) + P(1,3) * pos(i,9) + P(1,4);
      ry = P(2,1) * pos(i,7) + P(2,2) * pos(i,8) + P(2,3) * pos(i,9) + P(2,4);
      rz = P(3,1) * pos(i,7) + P(3,2) * pos(i,8) + P(3,3) * pos(i,9) + P(3,4);
      pos2d(i,5)=rx* (1.0/rz);
      pos2d(i,6)=ry* (1.0/rz);     
      grx = P(1,1) * gt(i,7) + P(1,2) * gt(i,8) + P(1,3) * gt(i,9) + P(1,4);
      gry = P(2,1) * gt(i,7) + P(2,2) * gt(i,8) + P(2,3) * gt(i,9) + P(2,4);
      grz = P(3,1) * gt(i,7) + P(3,2) * gt(i,8) + P(3,3) * gt(i,9) + P(3,4);
      gt2d(i,5)=grx* (1.0/grz);
      gt2d(i,6)=gry* (1.0/grz);               
      end
  
      
      for i =1:ms
      rx = P(1,1) * pos(i,10) + P(1,2) * pos(i,11) + P(1,3) * pos(i,12) + P(1,4);
      ry = P(2,1) * pos(i,10) + P(2,2) * pos(i,11) + P(2,3) * pos(i,12) + P(2,4);
      rz = P(3,1) * pos(i,10) + P(3,2) * pos(i,11) + P(3,3) * pos(i,12) + P(3,4);
      pos2d(i,7)=rx* (1.0/rz);
      pos2d(i,8)=ry* (1.0/rz);   
      
      grx = P(1,1) * gt(i,10) + P(1,2) * gt(i,11) + P(1,3) * gt(i,12) + P(1,4);
      gry = P(2,1) * gt(i,10) + P(2,2) * gt(i,11) + P(2,3) * gt(i,12) + P(2,4);
      grz = P(3,1) * gt(i,10) + P(3,2) * gt(i,11) + P(3,3) * gt(i,12) + P(3,4);
      gt2d(i,7)=grx* (1.0/grz);
      gt2d(i,8)=gry* (1.0/grz);               
      end
  
        
      for i =1:ms
      rx = P(1,1) * pos(i,13) + P(1,2) * pos(i,14) + P(1,3) * pos(i,15) + P(1,4);
      ry = P(2,1) * pos(i,13) + P(2,2) * pos(i,14) + P(2,3) * pos(i,15) + P(2,4);
      rz = P(3,1) * pos(i,13) + P(3,2) * pos(i,14) + P(3,3) * pos(i,15) + P(3,4);
      pos2d(i,9)=rx* (1.0/rz);
      pos2d(i,10)=ry* (1.0/rz);
      grx = P(1,1) * gt(i,13) + P(1,2) * gt(i,14) + P(1,3) * gt(i,15) + P(1,4);
      gry = P(2,1) * gt(i,13) + P(2,2) * gt(i,14) + P(2,3) * gt(i,15) + P(2,4);
      grz = P(3,1) * gt(i,13) + P(3,2) * gt(i,14) + P(3,3) * gt(i,15) + P(3,4);
      gt2d(i,9)=grx* (1.0/grz);
      gt2d(i,10)=gry* (1.0/grz);               
      end
  
          
          
      for i =1:ms
      rx = P(1,1) * pos(i,16) + P(1,2) * pos(i,17) + P(1,3) * pos(i,18) + P(1,4);
      ry = P(2,1) * pos(i,16) + P(2,2) * pos(i,17) + P(2,3) * pos(i,18) + P(2,4);
      rz = P(3,1) * pos(i,16) + P(3,2) * pos(i,17) + P(3,3) * pos(i,18) + P(3,4);
      pos2d(i,11)=rx* (1.0/rz);
      pos2d(i,12)=ry* (1.0/rz); 
      grx = P(1,1) * gt(i,16) + P(1,2) * gt(i,17) + P(1,3) * gt(i,18) + P(1,4);
      gry = P(2,1) * gt(i,16) + P(2,2) * gt(i,17) + P(2,3) * gt(i,18) + P(2,4);
      grz = P(3,1) * gt(i,16) + P(3,2) * gt(i,17) + P(3,3) * gt(i,18) + P(3,4);
      gt2d(i,11)=grx* (1.0/grz);
      gt2d(i,12)=gry* (1.0/grz);               
       end
  
            
      for i =1:ms
      rx = P(1,1) * pos(i,19) + P(1,2) * pos(i,20) + P(1,3) * pos(i,21) + P(1,4);
      ry = P(2,1) * pos(i,19) + P(2,2) * pos(i,20) + P(2,3) * pos(i,21) + P(2,4);
      rz = P(3,1) * pos(i,19) + P(3,2) * pos(i,20) + P(3,3) * pos(i,21) + P(3,4);
      pos2d(i,13)=rx* (1.0/rz);
      pos2d(i,14)=ry* (1.0/rz);  
      grx = P(1,1) * gt(i,19) + P(1,2) * gt(i,20) + P(1,3) * gt(i,21) + P(1,4);
      gry = P(2,1) * gt(i,19) + P(2,2) * gt(i,20) + P(2,3) * gt(i,21) + P(2,4);
      grz = P(3,1) * gt(i,19) + P(3,2) * gt(i,20) + P(3,3) * gt(i,21) + P(3,4);
      gt2d(i,13)=grx* (1.0/grz);
      gt2d(i,14)=gry* (1.0/grz);               
      end
  
  
              
 for i =1:ms
      rx = P(1,1) * pos(i,22) + P(1,2) * pos(i,23) + P(1,3) * pos(i,24) + P(1,4);
      ry = P(2,1) * pos(i,22) + P(2,2) * pos(i,23) + P(2,3) * pos(i,24) + P(2,4);
      rz = P(3,1) * pos(i,22) + P(3,2) * pos(i,23) + P(3,3) * pos(i,24) + P(3,4);
      pos2d(i,15)=rx* (1.0/rz);
      pos2d(i,16)=ry* (1.0/rz);       
      grx = P(1,1) * gt(i,22) + P(1,2) * gt(i,23) + P(1,3) * gt(i,24) + P(1,4);
      gry = P(2,1) * gt(i,22) + P(2,2) * gt(i,23) + P(2,3) * gt(i,24) + P(2,4);
      grz = P(3,1) * gt(i,22) + P(3,2) * gt(i,23) + P(3,3) * gt(i,24) + P(3,4);
      gt2d(i,15)=grx* (1.0/grz);
      gt2d(i,16)=gry* (1.0/grz);               
 end
  
  for i =1:ms
      rx = P(1,1) * pos(i,25) + P(1,2) * pos(i,26) + P(1,3) * pos(i,27) + P(1,4);
      ry = P(2,1) * pos(i,25) + P(2,2) * pos(i,26) + P(2,3) * pos(i,27) + P(2,4);
      rz = P(3,1) * pos(i,25) + P(3,2) * pos(i,26) + P(3,3) * pos(i,27) + P(3,4);
      pos2d(i,17)=rx* (1.0/rz);
      pos2d(i,18)=ry* (1.0/rz);  
      grx = P(1,1) * gt(i,25) + P(1,2) * gt(i,26) + P(1,3) * gt(i,27) + P(1,4);
      gry = P(2,1) * gt(i,25) + P(2,2) * gt(i,26) + P(2,3) * gt(i,27) + P(2,4);
      grz = P(3,1) * gt(i,25) + P(3,2) * gt(i,26) + P(3,3) * gt(i,27) + P(3,4);
      gt2d(i,17)=grx* (1.0/grz);
      gt2d(i,18)=gry* (1.0/grz);               
  end 
  
  
    for i =1:ms

      rx = P(1,1) * pos(i,28) + P(1,2) * pos(i,29) + P(1,3) * pos(i,30) + P(1,4);
      ry = P(2,1) * pos(i,28) + P(2,2) * pos(i,29) + P(2,3) * pos(i,30) + P(2,4);
      rz = P(3,1) * pos(i,28) + P(3,2) * pos(i,29) + P(3,3) * pos(i,30) + P(3,4);
      pos2d(i,19)=rx* (1.0/rz);
      pos2d(i,20)=ry* (1.0/rz);    

      grx = P(1,1) * gt(i,28) + P(1,2) * gt(i,29) + P(1,3) * gt(i,30) + P(1,4);
      gry = P(2,1) * gt(i,28) + P(2,2) * gt(i,29) + P(2,3) * gt(i,30) + P(2,4);
      grz = P(3,1) * gt(i,28) + P(3,2) * gt(i,29) + P(3,3) * gt(i,30) + P(3,4);
      gt2d(i,19)=grx* (1.0/grz);
      gt2d(i,20)=gry* (1.0/grz);                   
  end 
  
  
tip_gt2d=gt2d(:,1:2);
tip_filter2d=pos2d(:,1:2);
A2d=ones(ms,2);
RMSE2d=ones(ms,1);
for i =1:ms
   A2d(i,1)=norm(tip_gt2d(i,:));
   A2d(i,2)=norm(tip_filter2d(i,:));
  RMSE2d(i)= sqrt(mean((A2d(i,1) - A2d(i,2)).^2));
 
end


%% HAUSDORF DISTANCE

hd=ones(ms,1);
hd2d=ones(ms,1);
d=ones(ms,1);
hd2dn=ones(ms,1);
hd2dg=ones(ms,1);

% for i=1:ms
%     
% hd2d(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% hd2dn(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% hd2dn(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% 
% hd(i) = HausdorffDist(gt(i,1:8),pos(i,1:8));
% 
% end

for i=1:ms
    
hd2d(i) = HausdorffDist(pos2d(i,:),gt2d(i,:));
% hd2dn(i) = HausdorffDist(n(i,1:8),pos2d(i,1:8));
% hd2dg(i) = HausdorffDist(n(i,1:8),gt2d(i,1:8));

hd(i) = HausdorffDist(gt(i,:),pos(i,:));

end
% 
% figure
% plot(hd2d)
% hold on
% plot(hd2dn)
% hold on 
% plot(hd2dg)
% 
% % 
% figure
% plot3(pos2d(:,1),pos2d(:,2),gtREAD(1:1000,1),'*')
% hold on
% plot3(n(1:1000,1),n(1:1000,2),gtREAD(1:1000,1),'*')
% hold on 
% plot3(gt2d(1:1000,1),gt2d(1:1000,2),gtREAD(1:1000,1),'*')


TEST_SIDE_RMSE2d_1=RMSE2d;
TEST_SIDE_RMSE_1=RMSE;
TEST_SIDE_hd_1=hd;
TEST_SIDE_hd2d_1=hd2d;
save('TEST_SIDE_hd_1.mat','TEST_SIDE_hd_1');
save('TEST_SIDE_RMSE2d_1.mat','TEST_SIDE_RMSE2d_1');
save('TEST_SIDE_RMSE_1.mat','TEST_SIDE_RMSE_1');
save('TEST_SIDE_hd2d_1.mat','TEST_SIDE_hd2d_1');


