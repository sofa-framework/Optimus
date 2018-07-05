clear 
% close all
clc

gtREAD= dlmread('groundTruth_TEST_SIDE');
n2dREAD= dlmread('TESTNOISY');
gt=gtREAD(:,2:end);
n2d=n2dREAD(:,2:end);
P=[2281.11 -2.34947e-13 -400 273.957;-3.07529e-13 -2281.11 -300 -6.35711;-9.39116e-17 1.11022e-16 -1 0.317505];

gt2d=ones(690,20);

  for i =1:690

      grx = P(1,1) * gt(i,1) + P(1,2) * gt(i,2) + P(1,3) * gt(i,3) + P(1,4);
      gry = P(2,1) * gt(i,1) + P(2,2) * gt(i,2) + P(2,3) * gt(i,3) + P(2,4);
      grz = P(3,1) * gt(i,1) + P(3,2) * gt(i,2) + P(3,3) * gt(i,3) + P(3,4);
      gt2d(i,1)=grx* (1.0/grz);
      gt2d(i,2)=gry* (1.0/grz);               
  end

    for i =1:690

      grx = P(1,1) * gt(i,4) + P(1,2) * gt(i,5) + P(1,3) * gt(i,6) + P(1,4);
      gry = P(2,1) * gt(i,4) + P(2,2) * gt(i,5) + P(2,3) * gt(i,6) + P(2,4);
      grz = P(3,1) * gt(i,4) + P(3,2) * gt(i,5) + P(3,3) * gt(i,6) + P(3,4);
      gt2d(i,3)=grx* (1.0/grz);
      gt2d(i,4)=gry* (1.0/grz);               
    end
  
    
      for i =1:690

      grx = P(1,1) * gt(i,7) + P(1,2) * gt(i,8) + P(1,3) * gt(i,9) + P(1,4);
      gry = P(2,1) * gt(i,7) + P(2,2) * gt(i,8) + P(2,3) * gt(i,9) + P(2,4);
      grz = P(3,1) * gt(i,7) + P(3,2) * gt(i,8) + P(3,3) * gt(i,9) + P(3,4);
      gt2d(i,5)=grx* (1.0/grz);
      gt2d(i,6)=gry* (1.0/grz);               
      end
  
      
        for i =1:690

      grx = P(1,1) * gt(i,10) + P(1,2) * gt(i,11) + P(1,3) * gt(i,12) + P(1,4);
      gry = P(2,1) * gt(i,10) + P(2,2) * gt(i,11) + P(2,3) * gt(i,12) + P(2,4);
      grz = P(3,1) * gt(i,10) + P(3,2) * gt(i,11) + P(3,3) * gt(i,12) + P(3,4);
      gt2d(i,7)=grx* (1.0/grz);
      gt2d(i,8)=gry* (1.0/grz);               
        end
  
        
      for i =1:690

      grx = P(1,1) * gt(i,13) + P(1,2) * gt(i,14) + P(1,3) * gt(i,15) + P(1,4);
      gry = P(2,1) * gt(i,13) + P(2,2) * gt(i,14) + P(2,3) * gt(i,15) + P(2,4);
      grz = P(3,1) * gt(i,13) + P(3,2) * gt(i,14) + P(3,3) * gt(i,15) + P(3,4);
      gt2d(i,9)=grx* (1.0/grz);
      gt2d(i,10)=gry* (1.0/grz);               
      end
  
          
          
      for i =1:690

      grx = P(1,1) * gt(i,16) + P(1,2) * gt(i,17) + P(1,3) * gt(i,18) + P(1,4);
      gry = P(2,1) * gt(i,16) + P(2,2) * gt(i,17) + P(2,3) * gt(i,18) + P(2,4);
      grz = P(3,1) * gt(i,16) + P(3,2) * gt(i,17) + P(3,3) * gt(i,18) + P(3,4);
      gt2d(i,11)=grx* (1.0/grz);
      gt2d(i,12)=gry* (1.0/grz);               
       end
  
            
              for i =1:690

      grx = P(1,1) * gt(i,19) + P(1,2) * gt(i,20) + P(1,3) * gt(i,21) + P(1,4);
      gry = P(2,1) * gt(i,19) + P(2,2) * gt(i,20) + P(2,3) * gt(i,21) + P(2,4);
      grz = P(3,1) * gt(i,19) + P(3,2) * gt(i,20) + P(3,3) * gt(i,21) + P(3,4);
      gt2d(i,13)=grx* (1.0/grz);
      gt2d(i,14)=gry* (1.0/grz);               
              end
  
  
              
 for i =1:690

      grx = P(1,1) * gt(i,22) + P(1,2) * gt(i,23) + P(1,3) * gt(i,24) + P(1,4);
      gry = P(2,1) * gt(i,22) + P(2,2) * gt(i,23) + P(2,3) * gt(i,24) + P(2,4);
      grz = P(3,1) * gt(i,22) + P(3,2) * gt(i,23) + P(3,3) * gt(i,24) + P(3,4);
      gt2d(i,15)=grx* (1.0/grz);
      gt2d(i,16)=gry* (1.0/grz);               
 end
  
  for i =1:690

      grx = P(1,1) * gt(i,25) + P(1,2) * gt(i,26) + P(1,3) * gt(i,27) + P(1,4);
      gry = P(2,1) * gt(i,25) + P(2,2) * gt(i,26) + P(2,3) * gt(i,27) + P(2,4);
      grz = P(3,1) * gt(i,25) + P(3,2) * gt(i,26) + P(3,3) * gt(i,27) + P(3,4);
      gt2d(i,17)=grx* (1.0/grz);
      gt2d(i,18)=gry* (1.0/grz);               
  end
  
  
    for i =1:690

      grx = P(1,1) * gt(i,28) + P(1,2) * gt(i,29) + P(1,3) * gt(i,30) + P(1,4);
      gry = P(2,1) * gt(i,28) + P(2,2) * gt(i,29) + P(2,3) * gt(i,30) + P(2,4);
      grz = P(3,1) * gt(i,28) + P(3,2) * gt(i,29) + P(3,3) * gt(i,30) + P(3,4);
      gt2d(i,19)=grx* (1.0/grz);
      gt2d(i,20)=gry* (1.0/grz);               
    end
    
    
ms=690;
tip_gt2d=gt2d(:,1:2);
tip_filter2d=n2d(:,1:2);
A2d=ones(ms,2);
RMSE2d=ones(ms,1);

for i =1:ms
   A2d(i,1)=norm(tip_gt2d(i,:));
   A2d(i,2)=norm(tip_filter2d(i,:));
   RMSE2d(i)= sqrt((tip_gt2d(i,1)-tip_filter2d(i,1)).^2+   (tip_gt2d(i,2)-tip_filter2d(i,2)).^2  );
end

  
figure
plot(RMSE2d)
hold on
% plot(n2d(1:100:end,1),n2d(1:100:end,2),'*')
grid on
xlabel('h - [simulation step]') % x-axis label
ylabel('RMSE [mm] ') % y-axis label
title( {'Root Mean Square Error'; ' 3D Ground Truth Tip - 3D  Filter Tip' }) ;