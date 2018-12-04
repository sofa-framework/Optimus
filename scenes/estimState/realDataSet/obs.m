clear
close all
clc

Q=dlmread('../debug/Q_0000.txt');
P0=dlmread('../debug/P0_0000.txt');

R=dlmread('../debug/R_0000.txt');
gen=dlmread('../cov_/gen_0001.txt');
genpos=gen(1:18,:);

pos=cat(1,genpos(1:3,:),genpos(7:9,:),genpos(13:15,:));
eul=cat(1,genpos(4:6,:),genpos(10:12,:),genpos(16:18,:));
B=cat(2, pos, std(pos')');
C=cat(2, eul, std(eul')');

gen10=dlmread('../debug10/gen_0001.txt');
genpos10=gen10(1:18,:);

pos10=cat(1,genpos10(1:3,:),genpos10(7:9,:),genpos10(13:15,:));
eul10=cat(1,genpos10(4:6,:),genpos10(10:12,:),genpos10(16:18,:));
B10=cat(2, pos10, std(pos10')');
C10=cat(2, eul10, std(eul10')');

DB=cat(1,B,B10);
DC=cat(1,C,C10);

K=dlmread('../debug/K_0001.txt');
matPxx=dlmread('../debug/matPxx_0001.txt');
matGenPxx=dlmread('../debug/matGenPxx_0001.txt');

matPxz=dlmread('../debug/matPxz_0001.txt');
matPz=dlmread('../debug/matPz_0001.txt');
K=dlmread('../debug/K_0001.txt');

% figure
% imagesc(K)
% colorbar('peer',gca);
% title({'K '}); 

% figure
% imagesc(matPxx-P0)
% colorbar('peer',gca);
% title({'matPxx '}); 
U=triu(matGenPxx);
D=diag(matGenPxx);
L=eye(size(matGenPxx,1));
for i=1:size(L,1)
    L(i,i)=D(i);
end
figure
imagesc(U-L);
colorbar('peer',gca);
title({'matGenPxx '}); 
set(gca,'Layer','top','XTick',...
   [3.500,9.500,15.50,21.50,27.50,33.50],...
    'XTickLabel',...
    {'p1','p2','p3','v1','v2','v3'},'YTick',...
   [3.500,9.500,15.50,21.50,27.50,33.50],'YTickLabel',...
{'p1','p2','p3','v1','v2','v3'});
% xlim(gca,[-4 130]);
hold on
t=0.5:6:36.5;
t1=0.5:3:36.5;
y=0.5:6:36.5;
y1=t1;
h1=vline(t1,'b--');
h=vline(t,'k');
hold on
m2=hline(y,'k');
m=hline(y1,'b--');
% figure
% imagesc(matPxz')
% colorbar('peer',gca);
% title({'matPxz'}); 

% figure
% imagesc(matPz)
% colorbar('peer',gca);
% title({'matPz '}); 


% prop=dlmread('../debug/prop_0001.txt');

% 
% st=dlmread('quaternion2');
% st=st(:,1:36);
% writeStateSofa(st,'testReadTRANSFORM');
% 
% s1=dlmread('cat1');
% s1=s1(:,2:end);
% s2=dlmread('cat2');
% s2=s2(:,2:end);
% s3=dlmread('cat3');
% s3=s3(:,2:end);
% s4=dlmread('cat4');
% s4=s4(:,2:end);
% s5=dlmread('cat5');
% s5=s5(:,2:end);
% s6=dlmread('cat6');
% s6=s6(:,2:end);
% s7=dlmread('cat7');
% s7=s7(:,2:end);
% s8=dlmread('cat8');
% s8=s8(:,2:end);
% s9=dlmread('cat9');
% s9=s9(:,2:end);
% s10=dlmread('cat10');
% s10=s10(:,2:end);
% s11=dlmread('cat11');
% s11=s11(:,2:end);
% s12=dlmread('cat12');
% s12=s12(:,2:end);
% s13=dlmread('cat13');
% s13=s13(:,2:end);
% 
% A3=cat(1,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13);
% A=A3;
% B=cat(2,A(:,1:3),A(:,8:10),A(:,15:17),...
%     A(:,22:24),A(:,29:31),A(:,36:38),...
%     A(:,43:45),A(:,50:52),A(:,57:59),...
%     A(:,64:66),A(:,71:73),A(:,78:80));
% 
% % %%Align Original Position With Sofa Reference Frame
% % B=alignSofa((size(B,2)/3),B, 0.001,roty(-pi/2));
% 
%                                   
% K=[-3320.65000000000	174.572000000000	-514.441000000000 193.451000000000;
% 134.409000000000	3340.13000000000	44.4847000000000	-64.7090000000000;
% 0.0480279000000001	0.103531000000000	-0.993466000000000	0.400192000000000];
% 
% % %% Interpolate Plus Finement la beam
% %  I=interpolateResize( B);
% %  N=23; 
% %  
% % intrPol=zeros(N,3,size(I,3));
% % for i=1:size(I,3)
% %    intrPol(:,:,i) = interparc(N,I(1,:,i),I(2,:,i),I(3,:,i),'linear');
% % end
% % 
% % rsz=sofaResize(intrPol);
% 
% % %% Write Reference Compatible With SOFA
% % writeStateSofa(rsz,'rszrealAll3');
% 
% % %% Write Observation compatible with SOFA
% % obs2D=proj(rsz,K);
% % t = 0.001:0.001:size(obs2D,1)*0.001';
% % sofaObs2D=cat(2,t',obs2D); %% Obs 2D
% % dlmwrite('rszeall3_x.txt',sofaObs2D,'delimiter',' ');
% 
% 
% 
% 
% 
