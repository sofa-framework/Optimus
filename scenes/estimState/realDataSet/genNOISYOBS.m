clear all
close

ori=dlmread('../intermObs/Obs3D_beamE_x.txt');
t=ori(:,1);
obs=ori(:,2:end);

r=0.0008*rand(size(obs));

noi=obs+r;

% figure
% for i=1:length(t)
%     plot(noi(i,1),noi(i,2),'bo',noi(i,3),noi(i,4),'bo',noi(i,5),noi(i,6),'bo',noi(i,7),noi(i,8),'bo');
%     hold on
%     plot(obs(i,1),obs(i,2),'rx',obs(i,3),obs(i,4),'rx',obs(i,5),obs(i,6),'rx',obs(i,7),obs(i,8),'rx');
%     pause(0.5)
%     hold off
% end

noi2=cat(2,t,noi);
noi2=noi2(1:6000,:);
dlmwrite('../intermObs/Obs3D_beamENoise_x.txt',noi2,'delimiter',' ')

% asynnoi2=[];

% for i=1:size(noi2,1)/20
%    noi2((2*i*10):((2*i+1)*10),:)
%     asynnoi2=[asynnoi2; noi2(2*i*10:(2*i+1)*10,:)];
% end

% final=[noi2(1:10,:)];
% final100=[noi2(1:100,:)];
% final1000=[noi2(1:1000,:)];
% final=[noi2(1:10,:);asynnoi2]

% 
% dlmwrite('../intermObs/SSFFObs3D_hooke100Noise_x.txt',noi2,'delimiter',' ')
% dlmwrite('../intermObs/SSFFAsyn10Obs3D_hooke100Noise_x.txt',final,'delimiter',' ')
% dlmwrite('../intermObs/SSFFAsyn100Obs3D_hooke100Noise_x.txt',final100,'delimiter',' ')
% dlmwrite('../intermObs/SSFFAsyn100Obs3D_hooke100Noise_x.txt',final1000,'delimiter',' ')
% dlmwrite('../intermObs/SSFFObs3D_hooke100Noise_x.txt',noi2,'delimiter',' ')

writeStateSofa(noi,'../intermObs/Reference_beamENOISE')
writeStateSofa(obs,'../intermObs/Reference_beamE')