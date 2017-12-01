filterType='UKFClassic';

stepS=400;
stepM=400;

matrixFile=['../assimStiffness/cyl2_138_mat/spmatM_PARD_step' sprintf('%04d', stepS) '_iter0000.txt'];
covarFile=['../assimStiffness/outCyl2_138_UKFClassic_2/covar_' sprintf('%04d', stepS) '.txt'];
nparam=2;
m2s=1+[4 5 6 7 16 17 18 19 20 21 22 23 24 25 26 27 32 33 34 35 36 37 38 39 45 46 47 48 49 55 56 57 58 59];

%==========================================================

stmat=spconvert(load(matrixFile));
stmat =stmat + stmat';
stmat=stmat-0.5*diag(diag(stmat));

stmat=inv(stmat);
stmat = stmat - diag(diag(stmat));

nsdofs=length(m2s);

ndof=size(stmat,1);

stmatX=stmat(1:3:ndof, 1:3:ndof);
stmatY=stmat(2:3:ndof, 2:3:ndof);
stmatZ=stmat(3:3:ndof, 3:3:ndof);

stsmatX=zeros(nsdofs, nsdofs);
stsmatY=zeros(nsdofs, nsdofs);
stsmatZ=zeros(nsdofs, nsdofs);

size(stmatX)
max(m2s)

for i=1:nsdofs
    for j=1:nsdofs
        stsmatX(i,j) = stmatX(m2s(i), m2s(j));
        stsmatY(i,j) = stmatY(m2s(i), m2s(j));
        stsmatZ(i,j) = stmatZ(m2s(i), m2s(j));
    end    
end

covar=load(covarFile);
ncovar=size(covar, 1)-nparam;
covar=covar(1:ncovar, 1:ncovar);

covarX = covar(1:3:ncovar, 1:3:ncovar);
covarX = covarX - diag(diag(covarX));

covarY = covar(2:3:ncovar, 2:3:ncovar);
covarY = covarY - diag(diag(covarY));

covarZ = covar(3:3:ncovar, 3:3:ncovar);
covarZ = covarZ - diag(diag(covarZ));

figure; 
surf(fliplr(abs(stsmatX)));
title('Stiffness matrix, X component');
view(0, 90);

figure; 
surf(fliplr(abs(covarX)));
title('Covariance matrix, X component');
view(0, 90);

figure; 
surf(fliplr(abs(stsmatY)));
title('Stiffness matrix, Y component');
view(0, 90);

figure; 
surf(fliplr(abs(covarY)));
title('Covariance matrix, Y component');
view(0, 90);

figure; 
surf(fliplr(abs(stsmatZ)));
title('Stiffness matrix, Z component');
view(0, 90);

figure; 
surf(fliplr(abs(covarZ)));
title('Covariance matrix, Z component');
view(0, 90);







return

figure; 
surf(abs(stsmatY));
title('Stiffness matrix, Y component');

figure; 
surf(abs(stsmatZ));
title('Stiffness matrix, Z component');

