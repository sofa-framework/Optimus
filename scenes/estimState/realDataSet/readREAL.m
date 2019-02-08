clear 
close all
clc

%% Read GT and reshape as a matrix 3 x Nodes x Time Step
GTn=dlmread('rszrealAll3');
GTn=GTn(2:2:end,2:end);

GT=zeros(3,12,size(GTn,1));
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
   GT(1,12,i)=GTn(i,34);   GT(2,12,i)=GTn(i,35);  GT(3,12,i)=GTn(i,36); 

end

%% Read Filter Results as a matrix Nodes x 3 x Time Step

RQ1_R1=dlmread('Q1_R1');
Q1_R1=zeros(12,3,size(RQ1_R1,1));
for i=1:size(RQ1_R1,1)
    Q1_R1(:,:,i)=cat(1,RQ1_R1(i,1:3),RQ1_R1(i,4:6),RQ1_R1(i,7:9),...
              RQ1_R1(i,10:12),RQ1_R1(i,13:15),RQ1_R1(i,16:18),...
              RQ1_R1(i,19:21),RQ1_R1(i,22:24),RQ1_R1(i,25:27),...
              RQ1_R1(i,28:30),RQ1_R1(i,31:33),RQ1_R1(i,34:36));
end
RQ2_R1=dlmread('Q2_R1');
Q2_R1=zeros(12,3,size(RQ2_R1,1));
for i=1:size(RQ2_R1,1)
    Q2_R1(:,:,i)=cat(1,RQ2_R1(i,1:3),RQ2_R1(i,4:6),RQ2_R1(i,7:9),...
              RQ2_R1(i,10:12),RQ2_R1(i,13:15),RQ2_R1(i,16:18),...
              RQ2_R1(i,19:21),RQ2_R1(i,22:24),RQ2_R1(i,25:27),...
              RQ2_R1(i,28:30),RQ2_R1(i,31:33),RQ2_R1(i,34:36));
end
RQ3_R1=dlmread('Q3_R1');
Q3_R1=zeros(12,3,size(RQ3_R1,1));
for i=1:size(RQ3_R1,1)
    Q3_R1(:,:,i)=cat(1,RQ3_R1(i,1:3),RQ3_R1(i,4:6),RQ3_R1(i,7:9),...
              RQ3_R1(i,10:12),RQ3_R1(i,13:15),RQ3_R1(i,16:18),...
              RQ3_R1(i,19:21),RQ3_R1(i,22:24),RQ3_R1(i,25:27),...
              RQ3_R1(i,28:30),RQ3_R1(i,31:33),RQ3_R1(i,34:36));
end
RQ1_R2=dlmread('Q1_R2');
Q1_R2=zeros(12,3,size(RQ1_R2,1));
for i=1:size(RQ1_R2,1)
    Q1_R2(:,:,i)=cat(1,RQ1_R2(i,1:3),RQ1_R2(i,4:6),RQ1_R2(i,7:9),...
              RQ1_R2(i,10:12),RQ1_R2(i,13:15),RQ1_R2(i,16:18),...
              RQ1_R2(i,19:21),RQ1_R2(i,22:24),RQ1_R2(i,25:27),...
              RQ1_R2(i,28:30),RQ1_R2(i,31:33),RQ1_R2(i,34:36));
end
RQ2_R2=dlmread('Q2_R2');
Q2_R2=zeros(12,3,size(RQ2_R2,1));
for i=1:size(RQ2_R2,1)
    Q2_R2(:,:,i)=cat(1,RQ2_R2(i,1:3),RQ2_R2(i,4:6),RQ2_R2(i,7:9),...
              RQ2_R2(i,10:12),RQ2_R2(i,13:15),RQ2_R2(i,16:18),...
              RQ2_R2(i,19:21),RQ2_R2(i,22:24),RQ2_R2(i,25:27),...
              RQ2_R2(i,28:30),RQ2_R2(i,31:33),RQ2_R2(i,34:36));
end
RQ3_R2=dlmread('Q3_R2');
Q3_R2=zeros(12,3,size(RQ3_R2,1));
for i=1:size(RQ3_R2,1)
    Q3_R2(:,:,i)=cat(1,RQ3_R2(i,1:3),RQ3_R2(i,4:6),RQ3_R2(i,7:9),...
              RQ3_R2(i,10:12),RQ3_R2(i,13:15),RQ3_R2(i,16:18),...
              RQ3_R2(i,19:21),RQ3_R2(i,22:24),RQ3_R2(i,25:27),...
              RQ3_R2(i,28:30),RQ3_R2(i,31:33),RQ3_R2(i,34:36));
end
RQ1_R3=dlmread('Q1_R3');
Q1_R3=zeros(12,3,size(RQ1_R3,1));
for i=1:size(RQ1_R3,1)
    Q1_R3(:,:,i)=cat(1,RQ1_R3(i,1:3),RQ1_R3(i,4:6),RQ1_R3(i,7:9),...
              RQ1_R3(i,10:12),RQ1_R3(i,13:15),RQ1_R3(i,16:18),...
              RQ1_R3(i,19:21),RQ1_R3(i,22:24),RQ1_R3(i,25:27),...
              RQ1_R3(i,28:30),RQ1_R3(i,31:33),RQ1_R3(i,34:36));
end
RQ2_R3=dlmread('Q2_R3');
Q2_R3=zeros(12,3,size(RQ2_R3,1));
for i=1:size(RQ2_R3,1)
    Q2_R3(:,:,i)=cat(1,RQ2_R3(i,1:3),RQ2_R3(i,4:6),RQ2_R3(i,7:9),...
              RQ2_R3(i,10:12),RQ2_R3(i,13:15),RQ2_R3(i,16:18),...
              RQ2_R3(i,19:21),RQ2_R3(i,22:24),RQ2_R3(i,25:27),...
              RQ2_R3(i,28:30),RQ2_R3(i,31:33),RQ2_R3(i,34:36));
end
RQ3_R3=dlmread('Q3_R3');
Q3_R3=zeros(12,3,size(RQ3_R3,1));
for i=1:size(RQ3_R3,1)
    Q3_R3(:,:,i)=cat(1,RQ3_R3(i,1:3),RQ3_R3(i,4:6),RQ3_R3(i,7:9),...
              RQ3_R3(i,10:12),RQ3_R3(i,13:15),RQ3_R3(i,16:18),...
              RQ3_R3(i,19:21),RQ3_R3(i,22:24),RQ3_R3(i,25:27),...
              RQ3_R3(i,28:30),RQ3_R3(i,31:33),RQ3_R3(i,34:36));
end

RQ2_odl=dlmread('Q2_odlNOISE');
Q2_odl=zeros(12,3,size(RQ3_R3,1));
for i=1:size(RQ3_R3,1)
    Q2_odl(:,:,i)=cat(1,RQ2_odl(i,1:3),RQ2_odl(i,4:6),RQ2_odl(i,7:9),...
              RQ2_odl(i,10:12),RQ2_odl(i,13:15),RQ2_odl(i,16:18),...
              RQ2_odl(i,19:21),RQ2_odl(i,22:24),RQ2_odl(i,25:27),...
              RQ2_odl(i,28:30),RQ2_odl(i,31:33),RQ2_odl(i,34:36));
end

%% Compute Interpolation 
N=100; %interpolation resampling size
T=size(Q1_R1,3);

splGt=zeros(N,3,T);
ALL=cat(4,...
    Q1_R1,Q2_R1,Q3_R1,...
    Q1_R2,Q2_R2,Q3_R2,...
    Q1_R3,Q2_R3,Q3_R3,Q2_odl);

nbC=size(ALL,4);
splFilter=zeros(N,3,T,nbC);
h_mu_REAL=zeros(T,nbC);

t=0:size(h_mu_REAL,1)-1;
for k=1:nbC;
    for i=1:T
       splGt(:,:,i) = interparc(N,GT(1,:,i),GT(2,:,i),GT(3,:,i),'linear');
       splFilter(:,:,i,k) = interparc(N,ALL(:,1,i,k),ALL(:,2,i,k),ALL(:,3,i,k),'linear');
    end
end


save('splGt.mat','splGt');
save('splFilter.mat','splFilter');


t=0:size(h_mu_REAL,1)-1;
for k=1:nbC;
for i=1:T
   h_mu_REAL(i,k)=HausdorffDist(splFilter(:,:,i,k),splGt(:,:,i));
end
end

%% Compute mean dist at TIP
d_mu_REAL=zeros(T,nbC);

for i=1:T
        for k=1:nbC
               d_mu_REAL(i,k)=mean(sqrt(sum((splFilter(1:10,:,i,k)'-splGt(1:10,:,i)').^2)));
        end
end 

%% Compute RMSE Tip


RMSE_mu_REAL=zeros(T,nbC);
for i=1:T
        for k=1:nbC
        RMSE_mu_REAL(i,k)=norm([ALL(1,1,i,k) ALL(1,2,i,k) ALL(1,3,i,k)]-[GT(1,1,i) GT(2,1,i) GT(3,1,i)]);
        end
end 


Q1_R1=sofaResize(Q1_R1);
Q1_R2=sofaResize(Q1_R2);
Q1_R3=sofaResize(Q1_R3);

Q2_R1=sofaResize(Q2_R1);
Q2_R2=sofaResize(Q2_R2);
Q2_R3=sofaResize(Q2_R3);

Q3_R1=sofaResize(Q3_R1);
Q3_R2=sofaResize(Q3_R2);
Q3_R3=sofaResize(Q3_R3);

writeStateSofa(Q1_R1,'sQ1_R1');
writeStateSofa(Q1_R2,'sQ1_R2');
writeStateSofa(Q1_R3,'sQ1_R3');
writeStateSofa(Q2_R1,'sQ2_R1');
writeStateSofa(Q2_R2,'sQ2_R2');
writeStateSofa(Q2_R3,'sQ2_R3');
writeStateSofa(Q3_R1,'sQ3_R1');
writeStateSofa(Q3_R2,'sQ3_R2');
writeStateSofa(Q3_R3,'sQ3_R3');




%% Save data
save('h_mu_REAL.mat','h_mu_REAL');
save('RMSE_mu_REAL.mat','RMSE_mu_REAL');
save('d_mu_REAL.mat','d_mu_REAL');

        