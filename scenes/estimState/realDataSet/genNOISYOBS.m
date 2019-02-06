clear all
close

ori=dlmread('obs_N12_x.txt');
t=ori(:,1);
obs=ori(:,2:end);

r=0.1*randn(size(obs));

noi=obs+r;

% figure
% for i=1:length(t)
%     plot(noi(i,1),noi(i,2),'bo',noi(i,3),noi(i,4),'bo',noi(i,5),noi(i,6),'bo',noi(i,7),noi(i,8),'bo',...
%         noi(i,9),noi(i,10),'bo',noi(i,11),noi(i,12),'bo',noi(i,13),noi(i,14),'bo',noi(i,15),noi(i,16),'bo');
%     hold on
%     plot(obs(i,1),obs(i,2),'rx',obs(i,3),obs(i,4),'rx',obs(i,5),obs(i,6),'rx',obs(i,7),obs(i,8),'rx',...
%         obs(i,9),obs(i,10),'rx',obs(i,11),obs(i,12),'rx',obs(i,13),obs(i,14),'rx',obs(i,15),obs(i,16),'rx');
%     pause(0.5)
%     hold off
% end

noi2=cat(2,t,noi);
