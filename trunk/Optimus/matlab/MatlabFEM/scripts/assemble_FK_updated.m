function [F K] = assemble_FK_updated(model,fem,bvp)                        
    K = zeros(bvp.nSystemSize,1);   
    F = zeros(bvp.nSystemSize,1);   
    numElems = model.nElems;                
   
    initCoors=model.matCoorsRest;
    updCoors = bvp.matCoorsUpdated;
       
    vertListMat = model.matElems;
    
    for el = 1:numElems                           
        %sumW = sum(fem.gaussQuad(el,:,5));                
        %shapePhi = zeros(fem.qSize, 4);
        %shapePhi(:,:) = fem.shapeFunctions(el,:,:);
        
        %extract displacement and force from configuration computed before
        elemVertDisp = zeros(4,3);        
        for i = 1:4 
           k = 3*(model.matElems(el,i)-1)+1;
           elemVertDisp(i,:) = bvp.vertDisp(k:k+2);               
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
                        
        %updShapeFun=zeros(4,fem.qSize);
        %updIntShapeFun=zeros(4,1);
        %for i = 1:4 
        %    updShapeFun(i,:) = shapeFunCoeffUpd(1,i).*updX(:,1) + shapeFunCoeffUpd(2,i).*updX(:,2) + shapeFunCoeffUpd(3,i).*updX(:,3) + shapeFunCoeffUpd(4,i).*updX(:,4);
        %    updIntShapeFun(i)=updShapeFun(i,:)*updW;        
        %end         
                
        GradU=((shapeFunCoeffInit(2:4,1:4)*elemVertDisp))';      
        
        %GradUUpd=((shapeFunCoeffUpd(2:4,1:4)*(-elemVertDisp)))';      
        %GradDefUpd = (GradUUpd+eye(3));
        %inv(GradDefUpd)
        
        GradDef = (GradU+eye(3));
        if (model.inverseProblem)
            GradDef = inv(GradDef);
        end
        detGradDef=det(GradDef);
                        
        Strain = 1/2* (GradDef'*GradDef - eye(3));
        Stress2PK = (trace(Strain)*eye(3)*model.elastCoeff(1) + 2*model.elastCoeff(2)*Strain);        
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
            %actual load for the element
            %elemLoad = 1/detGradDef*updIntShapeFun(vert1)*model.density*model.gravity;
                        
            k1 = 3*(model.matElems(el,vert1)-1)+1;                                                      
            K(k1:k1+2) = K(k1:k1+2) + elemStiff;
            %F(k1:k1+2) = F(k1:k1+2) + elemLoad;
        end  %vert1    
       
        %updFEM(el).shapeFunCoeffInit=shapeFunCoeffInit;
        %updFEM(el).shapeFunCoeffUpd=shapeFunCoeffUpd;
        %updFEM(el).intShapeFun=updIntShapeFun;
        %updFEM(el).updSumW=updSumW;
    end    %for elems   
    if (model.massLumping)
        F=zeros(size(F));    
        for i=1:model.nNodes
            if (model.givenTotalMass)
                load=(model.totalMass*model.gravity)/model.nNodes;                
            else
                load=(model.density*fem.totalVolume*model.gravity)/model.nNodes;
            end            
            F(3*i-2 : 3*i) = load;
        end            
    end 
    %sum(updVolumes)
end