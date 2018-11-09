clear
close all
clc


o3dd = dlmread('/home/raffa/work/data_optimus/Raffa/p3d/sample5/all5.p3d');


for i=1:2781
    o3d(i,:)=o3dd(2781,:);
end
o3d=o3dd(1:4015,1:21);
B1=o3dd(2021:4015,1:21);
B=B1(2:end,1:21);
for i=1:size(B,1)
B(i,1:3)=0.001*B(i,1:3)*roty(-pi/2);
B(i,4:6)=0.001*B(i,4:6)*roty(-pi/2);
B(i,7:9)=0.001*B(i,7:9)*roty(-pi/2);
B(i,10:12)=0.001*B(i,10:12)*roty(-pi/2);
B(i,13:15)=0.001*B(i,13:15)*roty(-pi/2);
B(i,16:18)=0.001*B(i,16:18)*roty(-pi/2);
B(i,19:21)=0.001*B(i,19:21)*roty(-pi/2);
% B(i,22:24)=0.001*B(i,22:24)*roty(-pi/2);
% B(i,25:27)=0.001*B(i,25:27)*roty(-pi/2);
% B(i,28:30)=0.001*B(i,28:30)*roty(-pi/2);
% B(i,31:33)=0.001*B(i,31:33)*roty(-pi/2);

end


% 
% dlmwrite('p.P3',B);
% 
% % o= dlmread('/home/raffa/Documents/Raffa/sample/p2d/all.p2d');
% 
K=[-3320.65000000000	174.572000000000	-514.441000000000 193.451000000000;
134.409000000000	3340.13000000000	44.4847000000000	-64.7090000000000;
0.0480279000000001	0.103531000000000	-0.993466000000000	0.400192000000000];


% 
% 

A=proj(B,K,size(B,1));
% B2=B(2781:end,:);
C=zeros(1994,size(B,2)-1);

A2=A;
 
t = [0.001:0.001:size(A,1)/1000]'; 
t2 = [0.001:0.001:size(A2,1)*0.001]';
D=cat(2,t2,C);
E=zeros(1994*2,23);
E(:,1)=595959595;
E(:,2)=55555888;
for i=0:1992
%     for j=1:1236
    E(2*i+1,2)=333333;
%     E(2*i+2,3:end)=B2(i+2,:);

%     end
end


for i=0:1992
    E(2*i+1,3:end)=D(i+1,:);
    E(2*i+2,3:end)=B(i+2,:);
end
M=cat(2,t,A);
M2=cat(2,t2,A2);
% 
% test=cat(2,t3,t2,t4,P2);
% dlmwrite('M_x.txt',M);
dlmwrite('M22_x.txt',M2);
dlmwrite('E2.txt',E);
% dlmwrite('GTreal.txt',B);


% 
