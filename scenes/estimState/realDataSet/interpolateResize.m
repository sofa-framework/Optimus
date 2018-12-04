function [ I ] = interpolateResize( B)

% B = matrix [time step x 3 N ] with 3D positions 

%% Read B and reshape as a matrix 3 x Nodes x Time Step
%% Resample 3D state
M= size(B,2)/3;
%% Read B and reshape as a matrix 3 x Nodes x Time Step
I=zeros(3,M,size(B,1));

xB=1:3:(3*M)-2;
yB=2:3:(3*M)-1;
zB=3:3:3*M;

for i=1:size(B,1)
    for k=1:M    
        I(1,k,i)=B(i,xB(k));    
        I(2,k,i)=B(i,yB(k));   
        I(3,k,i)=B(i,zB(k));
    end
end




end

