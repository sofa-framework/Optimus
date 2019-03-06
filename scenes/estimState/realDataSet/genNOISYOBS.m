clear all
close

ori=dlmread('obs_N12_noise_x.txt');
t=ori(:,1);
obs=ori(:,2:end);

% r=0.0008*rand(size(obs));
noi=obs;

noi2=cat(2,t,noi);
noi2=noi2(1:2390,:);
% dlmwrite('../intermObs/Obs3D_massNoise_x.txt',noi2,'delimiter',' ')

asynnoi2=[];

for i=5:size(noi2,1)/20
   noi2((2*i*10):((2*i+1)*10),:);
   asynnoi2=[asynnoi2; noi2(2*i*10:(2*i+1)*10,:)];
end

% final10=[noi2(1:10,:)];
% final100=[noi2(1:100,:)];
% final1000=[noi2(1:1000,:)];
finalAsyn=[noi2(1:100,:);asynnoi2];


dlmwrite('Asynobs_N12_noise_x.txt',finalAsyn,'delimiter',' ')

% 
% % 
% dlmwrite('../intermObs/Obs3D_mass10Noise_x.txt',final10,'delimiter',' ')
% dlmwrite('../intermObs/Obs3D_mass100Noise_x.txt',final100,'delimiter',' ')
% dlmwrite('../intermObs/Obs3D_mass1000Noise_x.txt',final1000,'delimiter',' ')
% dlmwrite('../intermObs/Obs3D_massNoise_x.txt',noi2,'delimiter',' ')
% dlmwrite('../intermObs/Obs3D_massAsynNoise_x.txt',finalAsyn,'delimiter',' ')

% writeStateSofa(noi,'../intermObs/Reference_massNOISE')
% writeStateSofa(obs,'../intermObs/Reference_mass')