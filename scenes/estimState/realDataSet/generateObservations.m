clear 
close all
clc

% % Initialize Projection Matrix
K=[-3320.65000000000	174.572000000000	-514.441000000000 193.451000000000;
134.409000000000	3340.13000000000	44.4847000000000	-64.7090000000000;
0.0480279000000001	0.103531000000000	-0.993466000000000	0.400192];

% % Initialize 3D positions
GT=dlmread('rszrealAll3');
x=2:2:size(GT,1)-2;
    G=zeros(size(x,1),size(GT,2));
  for i=1:size(x,2)
      G(i,:)=GT(x(i),:);
  end

  %%  Resize reference to generate Nobs observations
  
  Nobs=6;
  Grsz=interpolateResize(G); %% resize ground truth to be compatible with interpolation function
  
  Ginter=zeros(Nobs,3,size(Grsz,3));
  for i=1:size(Grsz,3)
     Ginter(:,:,i)=interparc(Nobs,Grsz(1,:,i),Grsz(2,:,i),Grsz(3,:,i),'spline'); %% interpolate funtion with Nobs nodes
  end


%% Write Observation compatible with SOFA
  
Orsz=sofaResize(Ginter);
Obs2D=proj(Orsz,K);
Obs2Df=flipud(Obs2D);
t = 0.001:0.001:size(Obs2D,1)*0.001';
sofaObs2D=cat(2,t',Obs2Df); %% Obs 2D
dlmwrite(['fobsN', num2str(Nobs), 'x.txt'],sofaObs2D,'delimiter',' ');
% 
