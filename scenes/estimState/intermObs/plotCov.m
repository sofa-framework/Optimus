clear 
close all
clc

m_1_da=dlmread('m_1');
m_2_da=dlmread('m_2');
m_3_da=dlmread('m_10N');
m_4_da=dlmread('m_100N');
m_5_da=dlmread('m_1000N');
m_6_da=dlmread('m_Asyn');
se=dlmread('se_Asyn');
mE=dlmread('m_1E');


s=2400;

figure
t=[0 s-1];

% plot(m_1_da(1:s,end),'LineWidth',2)
% hold on
plot(m_2_da(1:s,end),'r','LineWidth',2,'DisplayName','Noisy Obs ALL TIME')
hold on
plot(m_3_da(1:10,end),'y','LineWidth',2,'DisplayName','Noisy Obs 10 dT')
hold on
plot([10:s-1],m_3_da(11:s,end),'y:','LineWidth',2,'DisplayName',' ')
hold on
plot(m_4_da(1:100,end),'b','LineWidth',2,'DisplayName','Noisy Obs 100 dT')
hold on
plot([100:s-1],m_4_da(101:s,end),'b:','LineWidth',2,'DisplayName',' ')
hold on
plot(m_5_da(1:1000,end),'g','LineWidth',2,'DisplayName','Noisy Obs 1000 dT')
hold on
plot([1000:s-1],m_5_da(1001:s,end),'g:','LineWidth',2,'DisplayName',' ')
hold on
plot(m_6_da(1:s,end),'m','LineWidth',2,'DisplayName','Asyn Observations')
hold on
% plot(mE(1:s,end),'LineWidth',2,'DisplayName','Incorrect Model Asyn Mass Estim')
% hold on
plot (t,[0.05 0.05], 'k','LineWidth',3,'DisplayName','Real Value=0.05')
vline(10,'k:','10 Obs')
hold on
vline(100,'k:','100 Obs')
hold on
vline(1000,'k:', '1000 Obs')
xlim([0 s-1])
legend('show');
title('Mass Estimation')

%% Evaluate DISTANCE WITH GT 


gt=dlmread('Obs3D_mass_x.txt');

obs=gt(1:s,2:end);
h_da=zeros(s,1);
h_se=zeros(s,1);
h_dae=zeros(s,1);

for i=1:s
   h_da(i)=mean(cat(2,norm(m_6_da(i,1:3)'-obs(i,1:3)'),norm(m_6_da(i,4:6)'-obs(i,4:6)'),norm(m_6_da(i,7:9)'-obs(i,7:9)')));
   h_se(i)=mean(cat(2,norm(se(i,1:3)'-obs(i,1:3)'),norm(se(i,4:6)'-obs(i,4:6)'),norm(se(i,7:9)'-obs(i,7:9)')));
   h_dae(i)=mean(cat(2,norm(mE(i,1:3)'-obs(i,1:3)'),norm(mE(i,4:6)'-obs(i,4:6)'),norm(mE(i,7:9)'-obs(i,7:9)')));

end
% 
% 
figure
plot(h_da,'r')
hold on
plot(h_se,'k')
hold on
% plot(h_dae,'b')
% hold on
vline(100,'k:')
legend('Data Assimilation','State Estimation', 'Data Assimilation with incorrect model')
xlim([0 2300])
ylim([0 0.025])
title('Mean Distance with Ground Truth')


% 
A=[];  B=[]; C=[];
for i=1:9
filename = sprintf('covDAm/cov_000%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
A=[A d];
end
for i=10:99
filename = sprintf('covDAm/cov_00%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
A=[A d];
end
for i=100:999
filename = sprintf('covDAm/cov_0%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
A=[A d];
end
for i=1000:2300
filename = sprintf('covDAm/cov_%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
A=[A d];
end

for i=1:9
filename = sprintf('covSE/cov_000%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
B=[B d];
end
for i=10:99
filename = sprintf('covSE/cov_00%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
B=[B d];
end
for i=100:999
filename = sprintf('covSE/cov_0%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
B=[B d];
end
for i=1000:2300
filename = sprintf('covSE/cov_%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
B=[B d];
end


for i=1:9
filename = sprintf('covDAE/cov_000%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
C=[C d];
end
for i=10:99
filename = sprintf('covDAE/cov_00%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
C=[C d];
end
for i=100:999
filename = sprintf('covDAE/cov_0%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
C=[C d];
end
for i=1000:2300
filename = sprintf('covDAE/cov_%d.txt',i);
c = load(filename);
d=trace(c(1:9,1:9));
C=[C d];
end


figure
plot(A,'r') 
hold on 
plot(B,'k')
% hold on
% plot(C,'b')
legend('Data Assimilation','State Estimation', 'Data Assimilation with incorrect model')
xlim([0 2300])
title('Final Covariance Trace')
