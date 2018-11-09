clear 
close all
clc

%% Read GT and reshape as a matrix 3 x Nodes x Time Step
GTnn=dlmread('GTreal.txt');
GTn=GTnn(2871:end,:);
GTn(1,:)=GTn(3,:);
GTn(2,:)=GTn(3,:);
GT=zeros(3,11,size(GTn,1));
for i=1:size(GTn,1)
   GT(1,1,i)=GTn(i,1);    GT(2,1,i)=GTn(i,2);   GT(3,1,i)=GTn(i,3);
   GT(1,2,i)=GTn(i,4);    GT(2,2,i)=GTn(i,5);   GT(3,2,i)=GTn(i,6);   
   GT(1,3,i)=GTn(i,7);    GT(2,3,i)=GTn(i,8);   GT(3,3,i)=GTn(i,9);   
   GT(1,4,i)=GTn(i,10);    GT(2,4,i)=GTn(i,11);   GT(3,4,i)=GTn(i,12);   
   GT(1,5,i)=GTn(i,13);    GT(2,5,i)=GTn(i,14);   GT(3,5,i)=GTn(i,15);   
   GT(1,6,i)=GTn(i,16);    GT(2,6,i)=GTn(i,17);   GT(3,6,i)=GTn(i,18);   
   GT(1,7,i)=GTn(i,19);    GT(2,7,i)=GTn(i,20);   GT(3,7,i)=GTn(i,21);   
   GT(1,8,i)=GTn(i,22);    GT(2,8,i)=GTn(i,23);   GT(3,8,i)=GTn(i,24);   
   GT(1,9,i)=GTn(i,25);    GT(2,9,i)=GTn(i,26);   GT(3,9,i)=GTn(i,27);   
   GT(1,10,i)=GTn(i,28);   GT(2,10,i)=GTn(i,29);  GT(3,10,i)=GTn(i,30); 
   GT(1,11,i)=GTn(i,31);   GT(2,11,i)=GTn(i,32);  GT(3,11,i)=GTn(i,33); 
end

%% Read Filter Results as a matrix Nodes x 3 x Time Step
 
RV=dlmread('real_E_10');
V=zeros(11,3,size(GT,3));
for i=1:size(V,3)
    V(:,:,i)=cat(1,RV(i,1:3),RV(i,7:9),RV(i,13:15),...
              RV(i,19:21),RV(i,25:27),RV(i,31:33),...
              RV(i,37:39),RV(i,43:45),RV(i,49:51),...
              RV(i,55:57),RV(i,61:63));
end

RVF_QVF=dlmread('real_E_12');
VF_QVF=zeros(11,3,size(GT,3));
for i=1:size(VF_QVF,3)
    VF_QVF(:,:,i)=cat(1,RVF_QVF(i,1:3),RVF_QVF(i,7:9),RVF_QVF(i,13:15),...
              RVF_QVF(i,19:21),RVF_QVF(i,25:27),RVF_QVF(i,31:33),...
              RVF_QVF(i,37:39),RVF_QVF(i,43:45),RVF_QVF(i,49:51),...
              RVF_QVF(i,55:57),RVF_QVF(i,61:63));
end

RVF_QV=dlmread('real_E_11');
VF_QV=zeros(11,3,size(GT,3));
for i=1:size(VF_QV,3)
    VF_QV(:,:,i)=cat(1,RVF_QV(i,1:3),RVF_QV(i,7:9),RVF_QV(i,13:15),...
              RVF_QV(i,19:21),RVF_QV(i,25:27),RVF_QV(i,31:33),...
              RVF_QV(i,37:39),RVF_QV(i,43:45),RVF_QV(i,49:51),...
              RVF_QV(i,55:57),RVF_QV(i,61:63));
end


RVF_QVgrad=dlmread('../real_E_13');
VF_QVgrad=zeros(11,3,size(GT,3));
for i=1:size(VF_QVgrad,3)
    VF_QVgrad(:,:,i)=cat(1,RVF_QVgrad(i,1:3),RVF_QVgrad(i,7:9),RVF_QVgrad(i,13:15),...
              RVF_QVgrad(i,19:21),RVF_QVgrad(i,25:27),RVF_QVgrad(i,31:33),...
              RVF_QVgrad(i,37:39),RVF_QVgrad(i,43:45),RVF_QVgrad(i,49:51),...
              RVF_QVgrad(i,55:57),RVF_QVgrad(i,61:63));
end

RVFF_QVgrad=dlmread('../real_E_14');
VFF_QVgrad=zeros(11,3,size(GT,3));
for i=1:size(VFF_QVgrad,3)
    VFF_QVgrad(:,:,i)=cat(1,RVFF_QVgrad(i,1:3),RVFF_QVgrad(i,7:9),RVFF_QVgrad(i,13:15),...
              RVFF_QVgrad(i,19:21),RVFF_QVgrad(i,25:27),RVFF_QVgrad(i,31:33),...
              RVFF_QVgrad(i,37:39),RVFF_QVgrad(i,43:45),RVFF_QVgrad(i,49:51),...
              RVFF_QVgrad(i,55:57),RVFF_QVgrad(i,61:63));
end

%% Compute Interpolation 
N=100; %interpolation resampling size
% 
splGt=zeros(N,3,size(GT,3));
splFilter=zeros(N,3,size(GT,3),3);
ALL=cat(4,V,VF_QVF,VF_QV,VF_QVgrad,VFF_QVgrad);
 
h_mu_1=zeros(size(GT,3),3);
 
t=0:size(h_mu_1,1)-1;
for k=1:size(ALL,4);
for i=1:size(GT,3)
   splGt(:,:,i) = interparc(N,GT(1,:,i),GT(2,:,i),GT(3,:,i),'linear');
   splFilter(:,:,i,k) = interparc(N,ALL(:,1,i,k),ALL(:,2,i,k),ALL(:,3,i,k),'linear');
   h_mu_1(i,k)=HausdorffDist(splFilter(:,:,i,k),splGt(:,:,i));
end
end

figure
for i=1:size(ALL,4)
plot(t, h_mu_1(:,i))
hold on
end% 
%% Compute mean dist at TIP
meanALL=zeros(size(ALL,3),size(ALL,4));

d_mu_1=meanALL;

for i=1:size(ALL,3)
        for k=1:size(ALL,4)
               d_mu_1(i,k)=mean(sqrt(sum((splFilter(1:10,:,i,k)'-splGt(1:10,:,i)').^2)));
        end
end 


figure
for i=1:size(ALL,4)
plot(t, d_mu_1(:,i))
hold on
end
%% Compute RMSE Tip
% 
tGT=zeros(1,size(ALL,3));
tALL=zeros(size(ALL,3),18);
% 
RMSE_mu_1=tALL;
for i=1:size(ALL,3)
        tGT(i)=norm([GT(1,1,i) GT(2,1,i) GT(3,1,i)]);

        for k=1:size(ALL,4)
        tALL(i,k)=norm([ALL(1,1,i,k) ALL(1,2,i,k) ALL(1,3,i,k)]);
        RMSE_mu_1(i,k)=sqrt(mean((tALL(i,k) - tGT(i)).^2));
        end
end 

figure
for i=1:size(ALL,4)
plot(t, RMSE_mu_1(:,i))
hold on
end% 
% 
% 
% %% Samu_20e data
% samu_20e('h_mu_1.mat','h_mu_1');
% samu_20e('h_Wmu_1.mat','h_Wmu_1');
% samu_20e('RMSE_mu_1.mat','RMSE_mu_1');
% samu_20e('RMSE_Wmu_1.mat','RMSE_Wmu_1');
% samu_20e('d_mu_1.mat','d_mu_1');
% samu_20e('d_Wmu_1.mat','d_Wmu_1');

        