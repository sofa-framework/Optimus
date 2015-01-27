function [aA] = assemble_aA(strcModel,strcFEM,strcBVP,strcSolution,vecObjectStiff)                        
    temp=sparse(strcBVP.nSystemSize,strcBVP.nSystemSize);    
    temp(1:length(vecObjectStiff), 1:length(strcBVP.vertDisp))=sparse(repmat(vecObjectStiff,[1 length(strcBVP.vertDisp)])) ;            
     
    numElems = strcModel.nElems;
    numPerturb = length(strcBVP.vertDisp);
    perturbation = strcSolution.perturbation;
    vertDisp = strcBVP.vertDisp;  
    
    for el=1:numElems
        elStr(el).mat = strcModel.matElems(el,:);
        elStr(el).sumW = sum(strcFEM.gaussQuad(el,:,5));
    end    
     
    glVecInd1=zeros(numPerturb,numElems*12);        
    glVecInd2=zeros(numPerturb,numElems*12);
    glVecVal=zeros(numPerturb,numElems*12);        
        
    %parfor pertDOF=1:min(numPerturb,50)  %numPerturb                                 
    parfor pertDOF=1:numPerturb                     
        perturbedVertDisp=vertDisp;
        perturbedVertDisp(pertDOF)=perturbedVertDisp(pertDOF)+perturbation;
        
        vecLength=numElems * 4 *3;
        vecInd1=zeros(1, vecLength);
        vecInd2=zeros(1, vecLength);
        vecVal=zeros(1, vecLength);
        index=1;   
        
        for el = 1:numElems                   
            elemVertDisp = zeros(4,3);

            %extract displacement and force from configuration computed before
            for i = 1:4 
               k = 3*(elStr(el).mat(i)-1)+1;
               elemVertDisp(i,:) = perturbedVertDisp(k:k+2);               
            end
            
            sumW=elStr(el).sumW; % sum(strcFEM.gaussQuad(el,:,5));            
            partInvVertMatrix = reshape(strcFEM.invVertMatrices(el,2:4,1:4),3,4);                        
            GradU=(partInvVertMatrix*elemVertDisp)';
            GradDef = (GradU+eye(3));

            %E, sigma and grad(sigma) for E
            %Strain = 1/2*(GradU + GradU' + GradU' * GradU);
            Strain = 1/2* (GradDef'*GradDef - eye(3));
                        
            %if (strcModel.typeMaterial == 'SV')   %StVenant material                
                Stress = trace(Strain)*eye(3)*strcModel.elastCoeff(1) + 2*strcModel.elastCoeff(2)*Strain;        
            %elseif (strcModel.typeMaterial == 'MR') %Mooney-Rivlin material
            %    C1=strcModel.elastCoeff(1); 
            %    C2=strcModel.elastCoeff(2);
            %    Stress = (2*C1 + 4*C2 + 4*C2*trace(Strain))*eye(3) - 4*C2*Strain;
            %end

            for vert1=1:4            
                DNDx1=partInvVertMatrix(1:3, vert1);
                %actual stiffness vector of E
%                 elemStiff=elemStiffVec(Stress, GradDef, DNDx1, W);              
                elemStiff= GradDef * (Stress * (DNDx1*sumW));                                                
                %assembleFEM the stiffness vector A(u)
                k1 = 3*(strcModel.matElems(el,vert1)-1)+1;                                                      
                vecInd1(index:index+2)=k1:k1+2;
                vecInd2(index:index+2)=pertDOF;
                vecVal(index:index+2) = elemStiff;
                index=index+3;                               
            end  %vert1                                              
        end    %for elems                           
        glVecInd1(pertDOF,:)=vecInd1;        
        glVecInd2(pertDOF,:)=vecInd2;
        glVecVal(pertDOF,:)=vecVal;                
    end %for pertNodes     
    finalVecInd1=reshape(glVecInd1,numPerturb*numElems*12,1);
    finalVecInd2=reshape(glVecInd2,numPerturb*numElems*12,1);
    finalVecVal=reshape(glVecVal,numPerturb*numElems*12,1);    
    aA=sparse(finalVecInd1,finalVecInd2,finalVecVal,strcBVP.nSystemSize, strcBVP.nSystemSize);
    aA=(aA-temp)./strcSolution.perturbation;      
    
    
end