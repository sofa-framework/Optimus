clear
close all
clc


o3d = dlmread('/home/raffa/Documents/Raffa/p3d/sample5/all5.p3d');


for i=1:2781
    o3d(i,:)=o3d(2781,:);
end
o3d=o3d(:,1:18);
B=o3d;

for i=1:size(B,1)
B(i,1:3)=0.001*o3d(i,1:3)*roty(-pi/2);
B(i,4:6)=0.001*o3d(i,4:6)*roty(-pi/2);
B(i,7:9)=0.001*o3d(i,7:9)*roty(-pi/2);
B(i,10:12)=0.001*o3d(i,10:12)*roty(-pi/2);
B(i,13:15)=0.001*o3d(i,13:15)*roty(-pi/2);
B(i,16:18)=0.001*o3d(i,16:18)*roty(-pi/2);
end


% 
dlmwrite('p.P3',B);

% o= dlmread('/home/raffa/Documents/Raffa/sample/p2d/all.p2d');

K=[-3320.65 174.572 -514.441 193.451;
134.409 3340.13	44.4847 -64.7090;
0.0480279 0.103531 -0.993466 0.40019];


A=proj(B,K,size(B,1));
A2=A(2780:end,:);
 P2=B(2780:end,:);

A=ones(size(P2,1)*2, size(P2,2));
% 
t = [0.001:0.001:size(A,1)*0.001]'; 
t2 = [0.001:0.001:size(A2,1)*0.001]'; 
t3 = ones (size(A2,1), 1)*556677;

t4= ones (size(A2,1), 1)*998855;


M=cat(2,t,A);
M2=cat(2,t2,A2);

test=cat(2,t3,t2,t4,P2);
dlmwrite('M_x.txt',M);
dlmwrite('M2_x.txt',M2);
dlmwrite('test.txt',test);


A=[5.781539916992187500e+01 4.846000671386718750e+01 3.944330978393554688e+01;
5.084276282755308785e+01 5.487996674207533943e+01 3.667258864673931384e+01;
4.872291734866303159e+01 6.059719836356089218e+01 2.952458767923963023e+01;
5.349635338841471821e+01 6.062654137850040570e+01 2.108586881903829635e+01;
5.893259835975186434e+01 5.637673503093537875e+01 1.403160581321208156e+01;
6.091539177193772048e+01 5.269786403677426279e+01 5.227474315179279074e+00];
BB=A;

for i=1:size(BB,1)
BB(i,:)=0.001*A(i,:)*roty(-pi/2);
end

C=ones(size(B,1)-1,18);
d=ones(size(C,1),6);

for i =1:size(B,1)-1
    C(i,1:3)=B(i+1,1:3)-B(i,1:3);
    C(i,4:6)=B(i+1,4:6)-B(i,4:6);
    C(i,7:9)=B(i+1,7:9)-B(i,7:9);
    C(i,10:12)=B(i+1,10:12)-B(i,10:12);
    C(i,13:15)=B(i+1,13:15)-B(i,13:15);
    C(i,16:18)=B(i+1,16:18)-B(i,16:18);

    d(i,1)=norm(C(i,1:3));
    d(i,2)=norm(C(i,4:6));
    d(i,3)=norm(C(i,7:9));
    d(i,4)=norm(C(i,10:12));
    d(i,5)=norm(C(i,13:15));
    d(i,6)=norm(C(i,16:18));


end




figure
for i=1:6
    plot(d(:,i))
    hold on
    grid on
    grid minor
end

R=[  0.7607938, -0.0000002, -0.6489937;
  -0.6085114, -0.3476538, -0.7133377;
  -0.2256250,  0.9376230, -0.2644930 ]