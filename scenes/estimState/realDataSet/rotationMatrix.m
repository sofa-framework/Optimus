clear 
close all
clc

P=[514.441 174.572 -3320.65 193451
-44.4847 3340.13 134.409 -64709
0.993466 0.103531 0.0480279 400.192];

% PR=eye(3)*P;

% B=ones(size(A,1),size(A,2));

%% ROTATE POINTS
% for i=1:size(B,1)
% B(i,:)=0.001*A(i,:)*roty(-pi/2);
% end
Ry=roty(-pi/2);
Ry(4,4)=1;
K=[3.34442609e+03 -7.91484993e-05  3.69669403e+02
0.00000000e+00  3.32890477e+03  3.08068338e+02
0.00000000e+00  0.00000000e+00  1.00000002e+00];



%% ROTATE and SCALE PROJECTION MATRIX TO ALIGN WITH SOFA X-AXIS
Ps=inv(K)*P*Ry;
Ps(:,4)=0.001*Ps(:,4);

Prot = K*Ps;

A=diag(0.000001*eye(3)*Ps)'