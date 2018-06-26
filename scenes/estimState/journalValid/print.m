clear 
close all
clc

gtREAD= dlmread('groundTruth_BIF_OB');
n2dREAD= dlmread('noisyObs_BIF_OB');
P=[770.277 -32.7486 -300.267 225.406; 217.307 -715.345 235.834 98.5108;0.194673 -0.612749 -0.765925 0.372637];
stateREAD= dlmread('matlab/print_state_BIF_OB_2');

gt=gtREAD(:,2:end);
n2d=n2dREAD(:,2:end);

pos=cat(2,stateREAD(:,1:3),stateREAD(:,7:9),stateREAD(:,13:15),stateREAD(:,19:21),stateREAD(:,25:27),stateREAD(:,31:33),...
    stateREAD(:,37:39),stateREAD(:,43:45),stateREAD(:,49:51),stateREAD(:,55:57));

tip_gt=gt(:,1:3);
tip_filter=pos(:,1:3);

ms=1230;
A=ones(ms,2);
RMSE=ones(ms,1);
for i =1:ms
   A(i,1)=norm(tip_gt(i,:));
   A(i,2)=norm(tip_filter(i,:));
  RMSE(i)= sqrt(mean((A(i,1) - A(i,2)).^2));
 
end

pos2d=ones(ms,2);
gt2d=ones(ms,2);


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

for i=1:ms
hd2d(i) = HausdorffDist(gt2d(i,:),pos2d(i,:));
hd(i) = HausdorffDist(gt(i,:),pos(i,:));

end


OB_RMSE2d_2=RMSE2d;
OB_RMSE_2=RMSE;
OB_hd_2=hd;
OB_hd2d_2=hd2d;
save('OB_hd_2.mat','OB_hd_2');
save('OB_RMSE2d_2.mat','OB_RMSE2d_2');
save('OB_RMSE_2.mat','OB_RMSE_2');
save('OB_hd2d_2.mat','OB_hd2d_2');


