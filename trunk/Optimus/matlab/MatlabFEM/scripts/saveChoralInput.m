function saveChoralInput(Model,FEM)
    rootDir='~/Work/MUNI/Choral2/tetra2/input';
    vertFile=sprintf('%s/FVertMatrices',rootDir);
    invFile=sprintf('%s/FInvVertMatrices',rootDir);
    quadFile=sprintf('%s/FQuadWeights',rootDir);
    shapeFile=sprintf('%s/FShapeFunctions',rootDir);
    elmFile=sprintf('%s/FElements',rootDir);    
           
    vertID=fopen(vertFile,'w');
    invID=fopen(invFile,'w');
    quadID=fopen(quadFile,'w');
    shapeID=fopen(shapeFile,'w');
    elmID=fopen(elmFile, 'w');
    
    
    if (vertID & invID & quadID & shapeID)
        disp('Files opened');
    end            
    
    for el=1:Model.nElems                
        
        
        vert=reshape(FEM.vertMatrices(el,:,:),4,4);
        inv=reshape(FEM.invVertMatrices(el,:,:),4,4);
        shape = reshape(FEM.shapeFunctions(el,:,:), 64, 4);
        wghts = reshape(FEM.gaussQuad(el,:,5), 64, 1);
        elm = Model.matElems(el,:);
        
        fprintf(elmID, '%d %d %d %d\n' , elm(1), elm(2), elm(3), elm(4));
        
        for i=1:4            
            fprintf(vertID, '%f %f %f %f\n', vert(i,1), vert(i,2), vert(i,3), vert(i,4));
            fprintf(invID, '%f %f %f %f\n', inv(i,1), inv(i,2), inv(i,3), inv(i,4));            
        end
        
        for i=1:64
            fprintf(quadID, '%f\n', wghts(i));
            fprintf(shapeID, '%f %f %f %f\n', shape(i,1), shape(i,2), shape(i,3), shape(i,4));
        end        
    end
    
    fclose(vertID);
    fclose(invID);
    fclose(quadID);
    fclose(shapeID);

end