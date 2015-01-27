function [F, K, A] = assemble_FKA(strcModel,strcFEM,strcIncompress,strcBVP,doAssembleMatrix)
%allocate the stiffness vector, tangent stiffnes matrix and load vector
K=zeros(strcBVP.nSystemSize,1);
F=zeros(strcBVP.nSystemSize,1);

if (doAssembleMatrix)
    nElems=strcModel.nElems;
    vecInd1=zeros(1, 4*4*9*nElems);
    vecInd2=zeros(1, 4*4*9*nElems);
    vecVal=zeros( 1, 4*4*9*nElems);
    sparseIndex=1;
end

if (strcIncompress.isIncompress)
    incompVecInd1=zeros(1,27*4*4*nElems);
    incompVecInd2=zeros(1,27*4*4*nElems);
    incompVecVal=zeros( 1,27*4*4*nElems);
    incompSparseIndex=1;
end

for el = 1:strcModel.nElems
    tmpVecObjectStiff=zeros(strcBVP.nSystemSize,1);
    tmpVecObjectLoad=zeros(strcBVP.nSystemSize,1);
    
    elemVertDisp = zeros(4,3);
    elemVertForce = zeros(4,3);
    
    %extract displacement and force from configuration computed before
    for i = 1:4
        k = 3*(strcModel.matElems(el,i)-1)+1;
        elemVertDisp(i,:) = strcBVP.vertDisp(k:k+2);
        elemVertForce(i,:) = strcModel.surfaceTractions(k:k+2);
    end
    
    EVD=elemVertDisp;
    
    if (strcIncompress.isIncompress)
        elemPressure=strcIncompress.vertPressure(strcModel.matElems(el,:));
    end
    
    %extract precomputed quadratures and strcFEM.vertMatrices for E
    W(:,1)=strcFEM.gaussQuad(el,:,5);
    invVertMatrix=zeros(3,4);
    invVertMatrix(1:3,1:4) = strcFEM.invVertMatrices(el,2:4,1:4);
    %replaced for parfor shapePhi(:,:) = strcFEM.shapeFunctions(el,:,:);
    %shapePhi = zeros(strcFEM.qSize, 4);
    %shapePhi(:,:) = strcFEM.shapeFunctions(el,:,:);
    
    %grad(U) matrix       
    IV=invVertMatrix;
    GradU=grad(elemVertDisp, invVertMatrix);
    GradDef = GradU+eye(3);
    
    %E, sigma and grad(sigma) for E
    Strain = strain(GradU);
    Stress = stress( strcModel, Strain);
    
    % compute principal stresses:
    
    st=Stress;
    
    I1=sum(diag(st));
    I2=st(1,1)*st(2,2) + st(2,2)*st(3,3) + st(1,1)*st(3,3) - st(1,2)^2 - st(2,3)^2 - st(1,3)^2;
    I3=det(st);
    
    r=roots([-1 I1 -I2 I3]);
    
    pn(1) = max(r);
    pn(3) = min(r);
    pn(2) = I1-pn(1)-pn(3);
    
    
    %three equivalent ways of computing vonMises:
    
    %vm1=sqrt( ( (pn(1) - pn(2))^2 + (pn(2) - pn(3))^2 + (pn(1) - pn(3))^2) /2);   
    %vm2=sqrt( ( (st(1,1) - st(2,2))^2 + (st(2,2) - st(3,3))^2 + (st(3,3) - st(1,1))^2 + 6 * (st(1,2)^2 + st(2,3)^2 + st(3,1)^2))/2);
    
    %Voigt notation:
    ss=[st(1,1) st(2,2) st(3,3) st(2,3) st(1,3) st(1,2)];   
    %vm3=sqrt(ss(1)^2 + ss(2)^2 + ss(3)^2 - ss(1)*ss(2) - ss(2)*ss(3) - ss(3)*ss(1) + 3*ss(4)^2 + 3*ss(5)^2 + 3*ss(6)^2);
    
    %some element-dependent structures for the incompressibility
    if (strcIncompress.isIncompress)
        %cofactor matrix of the deformation gradient (derivative of the determinant)
        CofMat=cofactor(GradDef);
        DetGradDef = det(GradDef);
        DetIS=1/sqrt(abs(DetGradDef));
        modCofMat = CofMat*DetIS;
        scSumPressure = dot(strcFEM.intShapeFunctions(el,:), elemPressure(:));
    end
    
    if (doAssembleMatrix)
        DStress= dStress(strcModel, Strain);
        [level1T] = tangMat_Level1(DStress, GradDef);
    end
    
    for vert1=1:4
        DNDx1=invVertMatrix(1:3, vert1);
        %actual stiffness vector of E
        elemStiff=elemStiffVec(Stress, GradDef, DNDx1, W);
        %body and surface forces
                
        elemBodyForce=elemBodyForceVec(strcModel.volumeForces, strcFEM.intShapeFunctions(el,vert1), el, strcModel.gravity, strcModel.density);
        elemSurfForce=elemSurfForceVec(elemVertForce, strcFEM.intShapeFunctions(el,vert1), vert1);
        elemLoad=elemBodyForce+elemSurfForce;
        
        %assembleFEM the stiffness vector A(u)
        k1 = 3*(strcModel.matElems(el,vert1)-1)+1;
        
        tmpVecObjectStiff(k1:k1+2) = tmpVecObjectStiff(k1:k1+2) + elemStiff;
        %assembleFEM the load vector f
        tmpVecObjectLoad(k1:k1+2) = tmpVecObjectLoad(k1:k1+2) + elemLoad;
        
        %assembleFEM the incompressibility term needed for the element STRANGE!!!!, should be improved
        termA = zeros(3,1);
        if (strcIncompress.isIncompress)
            v1 = strcModel.matElems(el,vert1);
            for i=1:3
                %sTerm=-dot(CofMat(i,:), DNDx1)*scSumPressure;
                sTerm=-(CofMat(i,:)* DNDx1)*scSumPressure;
                termA(i)=-(modCofMat(i,:)* DNDx1);
                %termA(i)=-dot(modCofMat(i,:), DNDx1);
                tmpVecObjectStiff(k1+i-1)=tmpVecObjectStiff(k1+i-1)+sTerm;
            end
            tmpVecObjectStiff(strcModel.nDOFs+v1)=tmpVecObjectStiff(strcModel.nDOFs+v1)+(1-DetGradDef)*strcFEM.intShapeFunctions(el,vert1);
        end
        
        if (doAssembleMatrix)
            sumW=sum(W);
            [level2T1, level2T2] = tangMat_Level2(level1T,Stress,DNDx1);
            
            %compute and assembleFEM the tangent stiffness matrix A'(u)
            for vert2=1:4
                k2 = 3*(strcModel.matElems(el,vert2)-1)+1;
                DNDx2=invVertMatrix(1:3, vert2);
                
                lmat=tangMat_Level3(level2T1,level2T2, DNDx2, sumW);
                lvec=reshape(lmat,1,9);
                xvec=[k1 k1+1 k1+2 k1 k1+1 k1+2 k1 k1+1 k1+2];
                yvec=[k2 k2 k2 k2+1 k2+1 k2+1 k2+2 k2+2 k2+2];
                
                vecInd1(sparseIndex:sparseIndex+8)=xvec;
                vecInd2(sparseIndex:sparseIndex+8)=yvec;
                vecVal(sparseIndex:sparseIndex+8) = lvec;
                sparseIndex=sparseIndex+9;
                
                %incompressibility conditions:
                if (strcIncompress.isIncompress)
                    %augment the tangent stiffness matrix
                    termB = zeros(3,1);
                    for i=1:3
                        termB(i)=-modCofMat(i,:) * DNDx2;
                    end
                    
                    for i=1:3
                        for j=1:3
                            sTerm = -CofMat(i,j) * DNDx1(j)*strcFEM.intShapeFunctions(el,vert2);
                            incompVecInd1(incompSparseIndex) = k1-1+i;
                            incompVecInd2(incompSparseIndex) = strcModel.nDOFs+strcModel.matElems(el,vert2);
                            incompVecVal(incompSparseIndex)=sTerm;
                            
                            incompVecInd1(incompSparseIndex+1) = strcModel.nDOFs+strcModel.matElems(el,vert2);
                            incompVecInd2(incompSparseIndex+1) = k1-1+i;
                            incompVecVal(incompSparseIndex+1)=sTerm;
                            
                            incompVecInd1(incompSparseIndex+2) = k1-1+i;
                            incompVecInd2(incompSparseIndex+2) = k2-1+j;
                            incompVecVal(incompSparseIndex+2)=(termA(j)*termB(i) - termA(i)*termB(j))*scSumPressure;
                            
                            incompSparseIndex = incompSparseIndex + 3;
                        end
                    end
                end
            end  %ver2
        end  %if doAssemble
    end  %vert1
    F = F + tmpVecObjectLoad;
    K = K + tmpVecObjectStiff;
    
    if (strcModel.massLumping)  %TODO: ignores F computed so far
        F=zeros(size(F));    
        for i=1:strcModel.nNodes
            if (strcModel.givenTotalMass)
                load=(strcModel.totalMass*strcModel.gravity)/strcModel.nNodes;
            else
                load=(strcModel.density*strcFEM.totalVolume*strcModel.gravity)/strcModel.nNodes;
            end            
            F(3*i-2 : 3*i) = load;
        end    
    end
end    %elems
if (doAssembleMatrix)
    sparseIndex=sparseIndex-1;
    A=sparse(vecInd1(1:sparseIndex),vecInd2(1:sparseIndex),vecVal(1:sparseIndex), strcBVP.nSystemSize, strcBVP.nSystemSize);
    if (strcIncompress.isIncompress)
        incompSparseIndex=incompSparseIndex-1;
        matIncompress=sparse(incompVecInd1(1:incompSparseIndex), incompVecInd2(1:incompSparseIndex), incompVecVal(1:incompSparseIndex),  strcBVP.nSystemSize, strcBVP.nSystemSize);
        A=A+matIncompress;
    end
else
    A=[];
end
end

function [cofMat] = cofactor(A)
cofMat=zeros(3);
cofMat(1,1) =  A(2,2)*A(3,3) - A(2,3)*A(3,2);
cofMat(1,2) = -A(2,1)*A(3,3) + A(2,3)*A(3,1);
cofMat(1,3) =  A(2,1)*A(3,2) - A(2,2)*A(3,1);
cofMat(2,1) = -A(1,2)*A(3,3) + A(1,3)*A(3,2);
cofMat(2,2) =  A(1,1)*A(3,3) - A(1,3)*A(3,1);
cofMat(2,3) = -A(1,1)*A(3,2) + A(1,2)*A(3,1);
cofMat(3,1) =  A(1,2)*A(2,3) - A(1,3)*A(2,2);
cofMat(3,2) = -A(1,1)*A(2,3) + A(1,3)*A(2,1);
cofMat(3,3) =  A(1,1)*A(2,2) - A(1,2)*A(2,1);
end

function [b] = elemStiffVec(DWdE,GradDef, DNDx, W)
%DWdE = tDWdE.data;
%GradU = tGradU.data;
%tDNDx = squeeze(tensor(invVertMatrix(2:4, vertIndex)));
%DNDx = invVertMatrix(2:4, vertIndex);
%tDNDx = tensor(ones(1,3));
%tF = tGradU+tensor(eye(3,3));
%F = GradU +eye(3);
%tF=tensor(eye(3,3));

% (3x3 x 3x3 kontrakce 1dim = 3x3) x 3 kontrakce 1 dim = 3
%tic
%bInt = ttt( ttt(tDWdE, tF, [1], [2]), tDNDx, [1], [1]);
%toc %0.013
%tic
tmp = DWdE' * GradDef';
Int = tmp' * DNDx;
%toc %0.000041
%b = bInt.data*sum(W);
b = Int * sum(W);

%size(elemPhi);
%b2=zeros(3,1);
%for i=1:3
%    b2(i) = W'*elemPhi*elemForce(elemIndex,i); % elemVertForce(vertIndex,i);
%end
%disp(sum(W));
%disp(sum(elemPhi));
%W'*elemPhi
%disp(b2);
%size(b1);
%b = -b1+b2;
end

function [fBody] = elemBodyForceVec(elemForce, intShapeFun, elemIndex, gravity, density)
fBody=zeros(3,1);
%for some special body forces (usually not being considered) + gravity
for i=1:3
    fBody(i) = intShapeFun*( elemForce(elemIndex,i) + gravity(i)*density ); % elemVertForce(vertIndex,i);
end

end

function [fSurf] = elemSurfForceVec(elemVertForce, intShapeFun, vertIndex)
fSurf=zeros(3,1);
for i=1:3
    fSurf(i) =  elemVertForce(vertIndex,i); %   W'*elemPhi*elemVertForce(vertIndex,i);
end
end

function [T] = tangMat(DdWddE, DWdE, GradDef, DNDx1, DNDx2, sumW)
%DdWddE = tDdWddE.data;

%DWdE = tDWdE.data;
%GradU = tGradU.data;


%ttDNDx1=squeeze(tensor(invVertMatrix(2:4, vertIndex1)));
%ttDNDx2=squeeze(tensor(invVertMatrix(2:4, vertIndex2)));
%DNDx1=invVertMatrix(2:4, vertIndex1);
%DNDx2=invVertMatrix(2:4, vertIndex2);

% 3x3 x 3 = 3x3x3
%tic
%ttF1 = ttt(tensor(eye(3,3))+tGradU, ttDNDx1);
%ttF2 = ttt(tensor(eye(3,3))+tGradU, ttDNDx2);
%toc %0.012
%tic
%GradU = GradU + eye(3);
tF1 = zeros(3,3,3);
tF2 = zeros(3,3,3);
for(i=1:3)
    tF1(:,:,i) = GradDef * DNDx1(i);
    tF2(:,:,i) = GradDef * DNDx2(i);
end
%toc %0.00010

% 3x3x3x3 x 3x3x3 kontrakce pres dve dimenze = 3x3x3
%tic
%tA=ttt(tDdWddE, ttF1, [1 2], [2 3]);
%toc %0.005
%tic
A = zeros(3,3,3);
for(i=1:3)
    for(j=1:3)
        for(k=1:3)
            for(l=1:3)
                for(m=1:3)
                    A(i,j,k) = A(i,j,k) + DdWddE(l,m,i,j)*tF1(k,l,m);
                end
            end
        end
    end
end
%toc %0.00005

% 3x3x3 x 3x3x3 kontrakce pres dve dimenze = 3x3
%tic
%ttTemp2 = ttt(tA, ttF2, [1,2], [2,3]);
%toc %0.01 to 0.005
%tic
tTemp2 = zeros(3,3);
for(i=1:3)
    for(j=1:3)
        for(k=1:3)
            for(l=1:3)
                tTemp2(i,j) = tTemp2(i,j) + A(k,l,i)*tF2(j,k,l);
            end
        end
    end
end
%toc % 0.000024

%tT=tDWdE;
T = DWdE;

% 3x3 x 3 kontrakce pres jednu dimenzi = 3
%tic
%ts1=ttt(tT,ttDNDx1,[1],[1]);
%toc %0.0035
s1=zeros(3,1);
for(i=1:3)
    for(j=1:3)
        s1(i) = s1(i) + T(j,i)*DNDx1(j);
    end
end


% 3 x 3 kontrakce pres jednu dimenzi = 0 (skalar)
%tic
%ts2=ttt(ts1,ttDNDx2,[1],[1]);
%toc % 0.003
%tic
s2 = s1' * DNDx2;
%toc % 0.000018

%tNewS = ts2.data*tensor(eye(3,3));
NewS = s2 * eye(3);

%tResult = ttTemp2 + tNewS;
Result = tTemp2 + NewS;
%tT = sum(W)*(tResult);
T = sumW*(Result);

%T=tT.data;
end

function [resT] = tangMat_Level1(DdWddE,GradDef)
temp=zeros(3,3,3,3);
%%%%%%%%%%%%%%%%%%%%%%%  PRESKLADANI OPERACI:
for i=1:3
    for j=1:3
        for k=1:3
            for l=1:3
                for m=1:3
                    temp(i,j,k,l)=temp(i,j,k,l)+DdWddE(m,j,k,l)*GradDef(i,m);
                end
            end
        end
    end
end  %243  x 1

resT=zeros(3,3,3,3);
for i=1:3
    for j=1:3
        for k=1:3
            for l=1:3
                for m=1:3
                    resT(i,j,k,l)=resT(i,j,k,l)+temp(i,j,m,l)*GradDef(k,m);
                end
            end
        end
    end
end  %243 x 1
end

function [resT1, resT2] = tangMat_Level2(inputT, DWdE, DNDx1)
resT1=zeros(3,3,3);
for i=1:3
    for j=1:3
        for k=1:3
            for l=1:3
                resT1(i,j,k) = resT1(i,j,k)+inputT(i,l,j,k)*DNDx1(l);
            end
        end
    end
end   %81 x 4

resT2=zeros(3,1);
for(i=1:3)
    for(j=1:3)
        resT2(i) = resT2(i) + DWdE(j,i)*DNDx1(j);
    end
end
end

function [resT] = tangMat_Level3(inputT1, inputT2, DNDx2, sumW)

term1=zeros(3,3);
for i=1:3
    for j=1:3
        for k=1:3
            term1(i,j) = term1(i,j)+inputT1(i,j,k)*DNDx2(k);
        end
    end
end %27 x 16

term2 = (inputT2' * DNDx2) * eye(3);

%integration
resT = sumW*(term1 + term2);

end

function [DWdE] = stress(strcModel,E)
%E=tE.data;
%tId=tensor(eye(3));

if (strcModel.typeMaterial == 'SV')   %StVenant material
    lam=strcModel.elastCoeff(1); mu=strcModel.elastCoeff(2);
    %tDWdE = trace(E)*tId*lam + 2*mu*tE
    DWdE = trace(E)*eye(3)*lam + 2*mu*E;
elseif (strcModel.typeMaterial == 'MR') %Mooney-Rivlin material
    C1=strcModel.elastCoeff(1);
    C2=strcModel.elastCoeff(2);
    DWdE = (2*C1 + 4*C2 + 4*C2*trace(E))*eye(3) - 4*C2*E;
end
%     END Mooney-Rivlin
%
%     Ciarlet:
%     p0=40; p1=50; p2=60;
%     a=p1+p2/2;
%     b=-(p1+p2)/2;
%     c=p0/4-b;
%     d=p0/2+p1;
%     mC=2*E + eye(3);
%     detC = det(inv(mC));
%     tDWdE = tensor(2*(a+b*trace(C))*eye(3) + mC*( -2*b + 2*c*detC - d));
%     END Ciarlet

end

function [DdWddE] = dStress(strcModel,E)
%E=tE.data;
%tId=tensor(eye(3,3));
%tId_ijkl=ttt(tId,tId);
Id_ijkl = zeros(3,3,3,3);
for(i=1:3)
    Id_ijkl(:,:,i,i) = eye(3);
end

%tId_ikjl=tensor(permute(tId_ijkl.data,[1,3,2,4]));
%tId_iljk=tensor(permute(tId_ijkl.data,[1,4,3,2]));
%tId_ikjl=tensor(permute(Id_ijkl,[1,3,2,4]));
%tId_iljk=tensor(permute(Id_ijkl,[1,4,3,2]));
Id_ikjl=permute(Id_ijkl,[1,3,2,4]);
Id_iljk=permute(Id_ijkl,[1,4,3,2]);
%tId_ijkl=tensor(Id_ijkl);

if (strcModel.typeMaterial == 'SV')   %StVenant material
    lam=strcModel.elastCoeff(1); mu=strcModel.elastCoeff(2);
    %tDdWddE = lam*tId_ijkl + mu*(tId_ikjl + tId_iljk);
    DdWddE = lam*Id_ijkl + mu*(Id_ikjl + Id_iljk);
    %tDdWddE=(tId_ikjl + tId_iljk);
    %tensor_as_matrix(tDdWddE,[1 3])
    %tDdWddE = mu*tDdWddE;
elseif (strcModel.typeMaterial == 'MR') %Mooney-Rivlin material
    C1=strcModel.elastCoeff(1); C2=strcModel.elastCoeff(2);
    DdWddE = 4*C2*Id_ijkl - 2*C2*(Id_ikjl + Id_iljk);
end

%     Ciarlet
%     p0=40; p1=50; p2=60;
%     b2 = -(p1 + p2);
%     c=(p0 - 2*b2) / 4;
%     d = p0/2 + p1;
%     mC=2*E + eye(3);
%     detC = det(inv(mC));
%     result = zeros(3,3,3,3);
%     for i=1:3
%         for j=1:3
%             result(i,i,j,j) = result(i,i,j,j) + 2*b2;
%             result(i,j,i,j) = result(i,j,i,j) - b2;
%             result(i,j,j,i) = result(i,j,j,i) - b2;
%             for k=1:3
%                 for l=1:3
%                     result(i,j,k,l) = result(i,j,k,l) + (mC(i,k)*mC(l,j) + mC(i,l)*mC(k,j))*(d-2*detC*c) +  ...
%                                                         (mC(i,j)*mC(k,l)) * detC * c *4;
%                 end
%             end
%         end
%     end
%     tDdWddE=tensor(result);
%     END Ciarlet
end

function [gradU] = grad(elemVertDisp,invVert)
gradU=zeros(3,3);
%disp('GRAD');
%elemVertDisp;
%invVert;

%elemVertDisp(:,1);
for i=1:3
    for j=1:3
        gradU(i,j) = invVert(j, :) * elemVertDisp(:,i);     %OK forever
    end
end
%tGradU=tensor(gradU);
end

function [E] = strain(gradU)
%gradU=tGradU.data;
E = 1/2*(gradU + gradU' + gradU' * gradU);
%E=1/2*(gradU+gradU');
%tE=tensor(E);
end

function [phi] = shape(invVert, X)
%size(X);
for i = 1:4
    phi(:,i) = invVert(1,i).*X(:,1) + invVert(2,i).*X(:,2) + invVert(3,i).*X(:,3) + invVert(4,i).*X(:,4);
end
%size(phi)
end
