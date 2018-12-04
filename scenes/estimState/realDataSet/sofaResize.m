function [ B ] = sofaResize( I)

% B = matrix [time step x 3 N ] with 3D positions 

%% Read B and reshape as a matrix 3 x Nodes x Time Step
%% Resample 3D state
N= size(I,1);
M= size(I,2);
%% Read B and reshape as a matrix 3 x Nodes x Time Step

xB=1:3:(3*N)-2;
yB=2:3:(3*N)-1;
zB=3:3:3*N;

for i=1:size(I,3)
    for k=1:N    
        B(i,xB(k))=I(k,1,i);    
        B(i,yB(k))=I(k,2,i);   
        B(i,zB(k))=I(k,3,i);
    end
end




end