function [F K ] = assemble_FK(strcModel,strcFEM,strcBVP)                        
    K = zeros(strcBVP.nSystemSize,1);   
    F = zeros(strcBVP.nSystemSize,1);   
    numElems = strcModel.nElems;                
                 
    for el = 1:numElems                           
        sumW = sum(strcFEM.gaussQuad(el,:,5));                
        %shapePhi = zeros(strcFEM.qSize, 4);
        %shapePhi(:,:) = strcFEM.shapeFunctions(el,:,:);
        
        %extract displacement and force from configuration computed before
        elemVertDisp = zeros(4,3);        
        for i = 1:4 
           k = 3*(strcModel.matElems(el,i)-1)+1;
           elemVertDisp(i,:) = strcBVP.vertDisp(k:k+2);               
        end
                                  
        partInvVertMatrix = reshape(strcFEM.invVertMatrices(el,2:4,1:4),3,4);        
        GradU=(partInvVertMatrix*elemVertDisp)';
        
        GradDef = (GradU+eye(3));                   
        %Strain = 1/2*(GradU + GradU' + GradU' * GradU);
        Strain = 1/2* (GradDef'*GradDef - eye(3));
        Stress = trace(Strain)*eye(3)*strcModel.elastCoeff(1) + 2*strcModel.elastCoeff(2)*Strain;                            
        
        for vert1=1:4            
            DNDx1=partInvVertMatrix(1:3, vert1);
            %actual stiffness vector of E                
            elemStiff= GradDef * (Stress * (DNDx1*sumW));
            %actual load for the element            
            elemLoad = strcFEM.intShapeFunctions(el,vert1)*strcModel.density*strcModel.gravity;
            %elemLoad=[0 -166.77 0]';
            %assembleFEM the stiffness vector A(u)
            k1 = 3*(strcModel.matElems(el,vert1)-1)+1;                                                      
            K(k1:k1+2) = K(k1:k1+2) + elemStiff;
            F(k1:k1+2) = F(k1:k1+2) + elemLoad;
        end  %vert1           
    end    %for elems 
    if (strcModel.massLumping)
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
end