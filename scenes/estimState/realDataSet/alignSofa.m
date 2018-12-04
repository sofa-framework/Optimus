function [ Aligned ] = alignSofa( N, Ori, Scale, Rot )

%% N = beam nodes
%% Ori = matrix [time step x 3 N ] with 3D positions 
%% scale = double for scaling
%% Rot = matrix 3x3 imposing rotation to align

Aligned=Ori;
x=1:3:(3*N)-2;
y=3:3:3*N;
for i=1:size(Ori,1)
    for j=1:N
        Aligned(i,x(j):y(j))=Scale*Ori(i,x(j):y(j))*Rot;
    end
end

