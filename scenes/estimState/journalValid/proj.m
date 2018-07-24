function [ pos2d ] = proj( pos, P, ms)

  for i =1:ms
      rx = P(1,1) * pos(i,1) + P(1,2) * pos(i,2) + P(1,3) * pos(i,3) + P(1,4);
      ry = P(2,1) * pos(i,1) + P(2,2) * pos(i,2) + P(2,3) * pos(i,3) + P(2,4);
      rz = P(3,1) * pos(i,1) + P(3,2) * pos(i,2) + P(3,3) * pos(i,3) + P(3,4);
      pos2d(i,1)=rx* (1.0/rz);
      pos2d(i,2)=ry* (1.0/rz);    
                 
  end


    for i =1:ms
      rx = P(1,1) * pos(i,4) + P(1,2) * pos(i,5) + P(1,3) * pos(i,6) + P(1,4);
      ry = P(2,1) * pos(i,4) + P(2,2) * pos(i,5) + P(2,3) * pos(i,6) + P(2,4);
      rz = P(3,1) * pos(i,4) + P(3,2) * pos(i,5) + P(3,3) * pos(i,6) + P(3,4);
      pos2d(i,3)=rx* (1.0/rz);
      pos2d(i,4)=ry* (1.0/rz);    
            
    end
  
    
      for i =1:ms
      rx = P(1,1) * pos(i,7) + P(1,2) * pos(i,8) + P(1,3) * pos(i,9) + P(1,4);
      ry = P(2,1) * pos(i,7) + P(2,2) * pos(i,8) + P(2,3) * pos(i,9) + P(2,4);
      rz = P(3,1) * pos(i,7) + P(3,2) * pos(i,8) + P(3,3) * pos(i,9) + P(3,4);
      pos2d(i,5)=rx* (1.0/rz);
      pos2d(i,6)=ry* (1.0/rz);     
              
      end
  
      
      for i =1:ms
      rx = P(1,1) * pos(i,10) + P(1,2) * pos(i,11) + P(1,3) * pos(i,12) + P(1,4);
      ry = P(2,1) * pos(i,10) + P(2,2) * pos(i,11) + P(2,3) * pos(i,12) + P(2,4);
      rz = P(3,1) * pos(i,10) + P(3,2) * pos(i,11) + P(3,3) * pos(i,12) + P(3,4);
      pos2d(i,7)=rx* (1.0/rz);
      pos2d(i,8)=ry* (1.0/rz);   
            
      end
  
        
      for i =1:ms
      rx = P(1,1) * pos(i,13) + P(1,2) * pos(i,14) + P(1,3) * pos(i,15) + P(1,4);
      ry = P(2,1) * pos(i,13) + P(2,2) * pos(i,14) + P(2,3) * pos(i,15) + P(2,4);
      rz = P(3,1) * pos(i,13) + P(3,2) * pos(i,14) + P(3,3) * pos(i,15) + P(3,4);
      pos2d(i,9)=rx* (1.0/rz);
      pos2d(i,10)=ry* (1.0/rz);
              
      end
  
          
          
      for i =1:ms
      rx = P(1,1) * pos(i,16) + P(1,2) * pos(i,17) + P(1,3) * pos(i,18) + P(1,4);
      ry = P(2,1) * pos(i,16) + P(2,2) * pos(i,17) + P(2,3) * pos(i,18) + P(2,4);
      rz = P(3,1) * pos(i,16) + P(3,2) * pos(i,17) + P(3,3) * pos(i,18) + P(3,4);
      pos2d(i,11)=rx* (1.0/rz);
      pos2d(i,12)=ry* (1.0/rz); 
              
       end
  
            
      for i =1:ms
      rx = P(1,1) * pos(i,19) + P(1,2) * pos(i,20) + P(1,3) * pos(i,21) + P(1,4);
      ry = P(2,1) * pos(i,19) + P(2,2) * pos(i,20) + P(2,3) * pos(i,21) + P(2,4);
      rz = P(3,1) * pos(i,19) + P(3,2) * pos(i,20) + P(3,3) * pos(i,21) + P(3,4);
      pos2d(i,13)=rx* (1.0/rz);
      pos2d(i,14)=ry* (1.0/rz);  
            
      end
  
  
              
 for i =1:ms
      rx = P(1,1) * pos(i,22) + P(1,2) * pos(i,23) + P(1,3) * pos(i,24) + P(1,4);
      ry = P(2,1) * pos(i,22) + P(2,2) * pos(i,23) + P(2,3) * pos(i,24) + P(2,4);
      rz = P(3,1) * pos(i,22) + P(3,2) * pos(i,23) + P(3,3) * pos(i,24) + P(3,4);
      pos2d(i,15)=rx* (1.0/rz);
      pos2d(i,16)=ry* (1.0/rz);       
            
 end
  
  for i =1:ms
      rx = P(1,1) * pos(i,25) + P(1,2) * pos(i,26) + P(1,3) * pos(i,27) + P(1,4);
      ry = P(2,1) * pos(i,25) + P(2,2) * pos(i,26) + P(2,3) * pos(i,27) + P(2,4);
      rz = P(3,1) * pos(i,25) + P(3,2) * pos(i,26) + P(3,3) * pos(i,27) + P(3,4);
      pos2d(i,17)=rx* (1.0/rz);
      pos2d(i,18)=ry* (1.0/rz);  
             
  end 
  
  
    for i =1:ms

      rx = P(1,1) * pos(i,28) + P(1,2) * pos(i,29) + P(1,3) * pos(i,30) + P(1,4);
      ry = P(2,1) * pos(i,28) + P(2,2) * pos(i,29) + P(2,3) * pos(i,30) + P(2,4);
      rz = P(3,1) * pos(i,28) + P(3,2) * pos(i,29) + P(3,3) * pos(i,30) + P(3,4);
      pos2d(i,19)=rx* (1.0/rz);
      pos2d(i,20)=ry* (1.0/rz);    
               
  end 


end

