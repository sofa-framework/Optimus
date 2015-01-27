function saveAsText(filename, array) 
    fID = fopen(filename,'w');
    
    for i=1:size(array,1)
        for j=1:size(array,2)            
            fprintf(fID,' %g', array(i,j));
        end
        fprintf(fID,'\n');        
    end
    fclose(fID);
end