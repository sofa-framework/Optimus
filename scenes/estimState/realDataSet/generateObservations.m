clear 
close all
clc

% % Initialize Projection Matrix
K=[-3320.65000000000	174.572000000000	-514.441000000000 193.451000000000;
134.409000000000	3340.13000000000	44.4847000000000	-64.7090000000000;
0.0480279000000001	0.103531000000000	-0.993466000000000	0.400192000000000];

% % Initialize 3D positions
GT=dlmread('rszrealAll3');
x=2:2:size(GT,1)-2;
    G=zeros(size(x,1),size(GT,2));
  for i=1:size(x,2)
      G(i,:)=GT(x(i),:);
  end
 
  

%% Write Observation compatible with SOFA
obs2D_23=proj(G,K);
t = 0.001:0.001:size(obs2D_23,1)*0.001';
sofaObs2D_23=cat(2,t',obs2D_23); %% Obs 2D
dlmwrite('obsNN23_x.txt',sofaObs2D_23,'delimiter',' ');
% 
