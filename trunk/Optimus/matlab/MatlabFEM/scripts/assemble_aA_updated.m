function [aA] = assemble_aA_updated(model,fem,bvp,solution,vecObjectStiff)
temp=sparse(bvp.nSystemSize,bvp.nSystemSize);
temp(1:length(vecObjectStiff), 1:length(bvp.vertDisp))=sparse(repmat(vecObjectStiff,[1 length(bvp.vertDisp)])) ;
numElems = model.nElems;
numPerturb = length(bvp.vertDisp);
perturbation = solution.perturbation;
vertDisp = bvp.vertDisp;

for el=1:numElems
    elStr(el).mat = model.matElems(el,:);
    %    elStr(el).sumW = sum(fem.gaussQuad(el,:,5));
end

glVecInd1=zeros(numPerturb,numElems*12);
glVecInd2=zeros(numPerturb,numElems*12);
glVecVal=zeros(numPerturb,numElems*12);

initCoors=model.matCoorsRest;
updCoors = bvp.matCoorsUpdated;
vertListMat = model.matElems;

for pertDOF=1:numPerturb
    perturbedVertDisp=vertDisp;
    perturbedVertDisp(pertDOF)=perturbedVertDisp(pertDOF) + perturbation;
    
    if (~ model.inverseProblem)
        %pertCoors = updCoors;
        updCoors = bvp.matCoorsUpdated;
        i1=ceil(pertDOF/3);
        i2=pertDOF-3*(i1-1);
        updCoors(i1,i2) = updCoors(i1,i2) + perturbation;
    end
    
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
               
        initVertMat=[ 1 initCoors(vertListMat(el,1),:);
                      1 initCoors(vertListMat(el,2),:);
                      1 initCoors(vertListMat(el,3),:);
                      1 initCoors(vertListMat(el,4),:) ];
        shapeFunCoeffInit=inv(initVertMat);
                
        if (model.inverseProblem)
            updSumW=1/6*abs(det(initVertMat));                
        else
            updVertMat=[ 1 updCoors(vertListMat(el,1),:);
                         1 updCoors(vertListMat(el,2),:);
                         1 updCoors(vertListMat(el,3),:);
                         1 updCoors(vertListMat(el,4),:) ];
            updSumW=1/6*abs(det(updVertMat));
            shapeFunCoeffUpd=inv(updVertMat);   
        end                                       
                
        GradU=(shapeFunCoeffInit(2:4,1:4)*elemVertDisp)';
        
        GradDef = GradU+eye(3);
        if (model.inverseProblem)
            GradDef = inv(GradDef);
        end
        detGradDef=det(GradDef);                        
        
        Strain = 1/2* (GradDef'*GradDef - eye(3));
        Stress2PK = trace(Strain)*eye(3)*model.elastCoeff(1) + 2*model.elastCoeff(2)*Strain;
        StressCauchy = 1/detGradDef*GradDef*Stress2PK*GradDef';
        
        for vert1=1:4
            if (model.inverseProblem)
                dShape=shapeFunCoeffInit(2:4, vert1);
            else
                dShape=shapeFunCoeffUpd(2:4, vert1);
            end
            %actual stiffness vector of E
            %elemStiff= GradDef * (Stress * (dShape*updSumW));
            elemStiff= (StressCauchy * (dShape*updSumW));
            
            k1 = 3*(model.matElems(el,vert1)-1)+1;
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
aA=sparse(finalVecInd1,finalVecInd2,finalVecVal,bvp.nSystemSize, bvp.nSystemSize);
aA=(aA-temp)./solution.perturbation;
%full(aA(1:12,1:12))
end