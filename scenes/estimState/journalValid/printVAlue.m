clear 
close all
clc

gtREAD= dlmread('groundTruth_TEST_TOP');
n2dREAD= dlmread('TESTNOISY');
% P=[775.552 -206.837 200.739 202.551;43.6739 289.403 727.253 109.588;0.148221 -0.711592 0.68678 0.348575]; %BIFTOP
% P=[770.277 -32.7486 -300.267 225.406;217.307 -715.345 235.834 98.5108;0.194673 -0.612749 -0.765925 0.372637]; %BIFSIDE
% P=[513.41 148.368 631.63 215.592;-193.229 -585.209 484.511 25.2104;0.760325 -0.628768 0.16296 0.445612]; %BIFOB

% P=[2281.11 -2.34947e-13 -400 273.957;-3.07529e-13 -2281.11 -300 -6.35711;-9.39116e-17 1.11022e-16 -1 0.317505]; %TESTSIDE
% P=[560.946 -169.854 584.336 149.418;-87.0629 -776.497 63.4844 14.5067;0.827776 -0.499976 -0.25458 0.286322]; %TESTOB
P=[19.8421 -404.504 721.763 143.041;-708.31 -335.931 -2.3288 68.0737;0.0501423 -0.998723 -0.00620961 0.362366]; %TESTTOP




stateREAD= dlmread('matlab/print_state_TEST_TOP_2');

gt=gtREAD(:,2:end);
n=n2dREAD(:,2:end)
% n2d=n2dREAD(:,2:end);

pos=cat(2,stateREAD(:,1:3),stateREAD(:,7:9),stateREAD(:,13:15),stateREAD(:,19:21),stateREAD(:,25:27),stateREAD(:,31:33),...
    stateREAD(:,37:39),stateREAD(:,43:45),stateREAD(:,49:51),stateREAD(:,55:57));

tip_gt=gt(:,1:3);
tip_filter=pos(:,1:3);

ms=1000;
A=ones(ms,2);
RMSE=ones(ms,1);
for i =1:ms
   A(i,1)=norm(tip_gt(i,:));
   A(i,2)=norm(tip_filter(i,:));
  RMSE(i)= sqrt(mean((A(i,1) - A(i,2)).^2));
 
end

pos2d=ones(ms,20);
gt2d=ones(ms,20);


  for i =1:ms
      rx = P(1,1) * pos(i,1) + P(1,2) * pos(i,2) + P(1,3) * pos(i,3) + P(1,4);
      ry = P(2,1) * pos(i,1) + P(2,2) * pos(i,2) + P(2,3) * pos(i,3) + P(2,4);
      rz = P(3,1) * pos(i,1) + P(3,2) * pos(i,2) + P(3,3) * pos(i,3) + P(3,4);
      pos2d(i,1)=rx* (1.0/rz);
      pos2d(i,2)=ry* (1.0/rz);    
            
      grx = P(1,1) * gt(i,1) + P(1,2) * gt(i,2) + P(1,3) * gt(i,3) + P(1,4);
      gry = P(2,1) * gt(i,1) + P(2,2) * gt(i,2) + P(2,3) * gt(i,3) + P(2,4);
      grz = P(3,1) * gt(i,1) + P(3,2) * gt(i,2) + P(3,3) * gt(i,3) + P(3,4);
      gt2d(i,1)=grx* (1.0/grz);
      gt2d(i,2)=gry* (1.0/grz);               
  end


    for i =1:ms
      rx = P(1,1) * pos(i,4) + P(1,2) * pos(i,5) + P(1,3) * pos(i,6) + P(1,4);
      ry = P(2,1) * pos(i,4) + P(2,2) * pos(i,5) + P(2,3) * pos(i,6) + P(2,4);
      rz = P(3,1) * pos(i,4) + P(3,2) * pos(i,5) + P(3,3) * pos(i,6) + P(3,4);
      pos2d(i,3)=rx* (1.0/rz);
      pos2d(i,4)=ry* (1.0/rz);    
      
      grx = P(1,1) * gt(i,4) + P(1,2) * gt(i,5) + P(1,3) * gt(i,6) + P(1,4);
      gry = P(2,1) * gt(i,4) + P(2,2) * gt(i,5) + P(2,3) * gt(i,6) + P(2,4);
      grz = P(3,1) * gt(i,4) + P(3,2) * gt(i,5) + P(3,3) * gt(i,6) + P(3,4);
      gt2d(i,3)=grx* (1.0/grz);
      gt2d(i,4)=gry* (1.0/grz);               
    end
  
    
      for i =1:ms
      rx = P(1,1) * pos(i,7) + P(1,2) * pos(i,8) + P(1,3) * pos(i,9) + P(1,4);
      ry = P(2,1) * pos(i,7) + P(2,2) * pos(i,8) + P(2,3) * pos(i,9) + P(2,4);
      rz = P(3,1) * pos(i,7) + P(3,2) * pos(i,8) + P(3,3) * pos(i,9) + P(3,4);
      pos2d(i,5)=rx* (1.0/rz);
      pos2d(i,6)=ry* (1.0/rz);     
      grx = P(1,1) * gt(i,7) + P(1,2) * gt(i,8) + P(1,3) * gt(i,9) + P(1,4);
      gry = P(2,1) * gt(i,7) + P(2,2) * gt(i,8) + P(2,3) * gt(i,9) + P(2,4);
      grz = P(3,1) * gt(i,7) + P(3,2) * gt(i,8) + P(3,3) * gt(i,9) + P(3,4);
      gt2d(i,5)=grx* (1.0/grz);
      gt2d(i,6)=gry* (1.0/grz);               
      end
  
      
      for i =1:ms
      rx = P(1,1) * pos(i,10) + P(1,2) * pos(i,11) + P(1,3) * pos(i,12) + P(1,4);
      ry = P(2,1) * pos(i,10) + P(2,2) * pos(i,11) + P(2,3) * pos(i,12) + P(2,4);
      rz = P(3,1) * pos(i,10) + P(3,2) * pos(i,11) + P(3,3) * pos(i,12) + P(3,4);
      pos2d(i,7)=rx* (1.0/rz);
      pos2d(i,8)=ry* (1.0/rz);   
      
      grx = P(1,1) * gt(i,10) + P(1,2) * gt(i,11) + P(1,3) * gt(i,12) + P(1,4);
      gry = P(2,1) * gt(i,10) + P(2,2) * gt(i,11) + P(2,3) * gt(i,12) + P(2,4);
      grz = P(3,1) * gt(i,10) + P(3,2) * gt(i,11) + P(3,3) * gt(i,12) + P(3,4);
      gt2d(i,7)=grx* (1.0/grz);
      gt2d(i,8)=gry* (1.0/grz);               
      end
  
        
      for i =1:ms
      rx = P(1,1) * pos(i,13) + P(1,2) * pos(i,14) + P(1,3) * pos(i,15) + P(1,4);
      ry = P(2,1) * pos(i,13) + P(2,2) * pos(i,14) + P(2,3) * pos(i,15) + P(2,4);
      rz = P(3,1) * pos(i,13) + P(3,2) * pos(i,14) + P(3,3) * pos(i,15) + P(3,4);
      pos2d(i,9)=rx* (1.0/rz);
      pos2d(i,10)=ry* (1.0/rz);
      grx = P(1,1) * gt(i,13) + P(1,2) * gt(i,14) + P(1,3) * gt(i,15) + P(1,4);
      gry = P(2,1) * gt(i,13) + P(2,2) * gt(i,14) + P(2,3) * gt(i,15) + P(2,4);
      grz = P(3,1) * gt(i,13) + P(3,2) * gt(i,14) + P(3,3) * gt(i,15) + P(3,4);
      gt2d(i,9)=grx* (1.0/grz);
      gt2d(i,10)=gry* (1.0/grz);               
      end
  
          
          
      for i =1:ms
      rx = P(1,1) * pos(i,16) + P(1,2) * pos(i,17) + P(1,3) * pos(i,18) + P(1,4);
      ry = P(2,1) * pos(i,16) + P(2,2) * pos(i,17) + P(2,3) * pos(i,18) + P(2,4);
      rz = P(3,1) * pos(i,16) + P(3,2) * pos(i,17) + P(3,3) * pos(i,18) + P(3,4);
      pos2d(i,11)=rx* (1.0/rz);
      pos2d(i,12)=ry* (1.0/rz); 
      grx = P(1,1) * gt(i,16) + P(1,2) * gt(i,17) + P(1,3) * gt(i,18) + P(1,4);
      gry = P(2,1) * gt(i,16) + P(2,2) * gt(i,17) + P(2,3) * gt(i,18) + P(2,4);
      grz = P(3,1) * gt(i,16) + P(3,2) * gt(i,17) + P(3,3) * gt(i,18) + P(3,4);
      gt2d(i,11)=grx* (1.0/grz);
      gt2d(i,12)=gry* (1.0/grz);               
       end
  
            
      for i =1:ms
      rx = P(1,1) * pos(i,19) + P(1,2) * pos(i,20) + P(1,3) * pos(i,21) + P(1,4);
      ry = P(2,1) * pos(i,19) + P(2,2) * pos(i,20) + P(2,3) * pos(i,21) + P(2,4);
      rz = P(3,1) * pos(i,19) + P(3,2) * pos(i,20) + P(3,3) * pos(i,21) + P(3,4);
      pos2d(i,13)=rx* (1.0/rz);
      pos2d(i,14)=ry* (1.0/rz);  
      grx = P(1,1) * gt(i,19) + P(1,2) * gt(i,20) + P(1,3) * gt(i,21) + P(1,4);
      gry = P(2,1) * gt(i,19) + P(2,2) * gt(i,20) + P(2,3) * gt(i,21) + P(2,4);
      grz = P(3,1) * gt(i,19) + P(3,2) * gt(i,20) + P(3,3) * gt(i,21) + P(3,4);
      gt2d(i,13)=grx* (1.0/grz);
      gt2d(i,14)=gry* (1.0/grz);               
      end
  
  
              
 for i =1:ms
      rx = P(1,1) * pos(i,22) + P(1,2) * pos(i,23) + P(1,3) * pos(i,24) + P(1,4);
      ry = P(2,1) * pos(i,22) + P(2,2) * pos(i,23) + P(2,3) * pos(i,24) + P(2,4);
      rz = P(3,1) * pos(i,22) + P(3,2) * pos(i,23) + P(3,3) * pos(i,24) + P(3,4);
      pos2d(i,15)=rx* (1.0/rz);
      pos2d(i,16)=ry* (1.0/rz);       
      grx = P(1,1) * gt(i,22) + P(1,2) * gt(i,23) + P(1,3) * gt(i,24) + P(1,4);
      gry = P(2,1) * gt(i,22) + P(2,2) * gt(i,23) + P(2,3) * gt(i,24) + P(2,4);
      grz = P(3,1) * gt(i,22) + P(3,2) * gt(i,23) + P(3,3) * gt(i,24) + P(3,4);
      gt2d(i,15)=grx* (1.0/grz);
      gt2d(i,16)=gry* (1.0/grz);               
 end
  
  for i =1:ms
      rx = P(1,1) * pos(i,25) + P(1,2) * pos(i,26) + P(1,3) * pos(i,27) + P(1,4);
      ry = P(2,1) * pos(i,25) + P(2,2) * pos(i,26) + P(2,3) * pos(i,27) + P(2,4);
      rz = P(3,1) * pos(i,25) + P(3,2) * pos(i,26) + P(3,3) * pos(i,27) + P(3,4);
      pos2d(i,17)=rx* (1.0/rz);
      pos2d(i,18)=ry* (1.0/rz);  
      grx = P(1,1) * gt(i,25) + P(1,2) * gt(i,26) + P(1,3) * gt(i,27) + P(1,4);
      gry = P(2,1) * gt(i,25) + P(2,2) * gt(i,26) + P(2,3) * gt(i,27) + P(2,4);
      grz = P(3,1) * gt(i,25) + P(3,2) * gt(i,26) + P(3,3) * gt(i,27) + P(3,4);
      gt2d(i,17)=grx* (1.0/grz);
      gt2d(i,18)=gry* (1.0/grz);               
  end 
  
  
    for i =1:ms

      rx = P(1,1) * pos(i,28) + P(1,2) * pos(i,29) + P(1,3) * pos(i,30) + P(1,4);
      ry = P(2,1) * pos(i,28) + P(2,2) * pos(i,29) + P(2,3) * pos(i,30) + P(2,4);
      rz = P(3,1) * pos(i,28) + P(3,2) * pos(i,29) + P(3,3) * pos(i,30) + P(3,4);
      pos2d(i,19)=rx* (1.0/rz);
      pos2d(i,20)=ry* (1.0/rz);    

      grx = P(1,1) * gt(i,28) + P(1,2) * gt(i,29) + P(1,3) * gt(i,30) + P(1,4);
      gry = P(2,1) * gt(i,28) + P(2,2) * gt(i,29) + P(2,3) * gt(i,30) + P(2,4);
      grz = P(3,1) * gt(i,28) + P(3,2) * gt(i,29) + P(3,3) * gt(i,30) + P(3,4);
      gt2d(i,19)=grx* (1.0/grz);
      gt2d(i,20)=gry* (1.0/grz);                   
  end 
  
  
tip_gt2d=gt2d(:,1:2);
tip_filter2d=pos2d(:,1:2);
A2d=ones(ms,2);
RMSE2d=ones(ms,1);
for i =1:ms
   A2d(i,1)=norm(tip_gt2d(i,:));
   A2d(i,2)=norm(tip_filter2d(i,:));
  RMSE2d(i)= sqrt(mean((A2d(i,1) - A2d(i,2)).^2));
 
end


%% HAUSDORF DISTANCE

hd=ones(ms,1);
hd2d=ones(ms,1);
d=ones(ms,1);
hd2dn=ones(ms,1);
hd2dg=ones(ms,1);

% for i=1:ms
%     
% hd2d(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% hd2dn(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% hd2dn(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
% 
% hd(i) = HausdorffDist(gt(i,1:8),pos(i,1:8));
% 
% end

for i=1:ms
    
hd2d(i) = HausdorffDist(pos2d(i,1:8),gt2d(i,1:8));
hd2dn(i) = HausdorffDist(n(i,1:8),pos2d(i,1:8));
hd2dg(i) = HausdorffDist(n(i,1:8),gt2d(i,1:8));

hd(i) = HausdorffDist(gt(i,1:8),pos(i,1:8));

end

figure
plot(hd2d)
hold on
plot(hd2dn)
hold on 
plot(hd2dg)

% 
% figure
% plot3(pos2d(:,1),pos2d(:,2),gtREAD(1:1000,1),'*')
% hold on
% plot3(n(1:1000,1),n(1:1000,2),gtREAD(1:1000,1),'*')
% hold on 
% plot3(gt2d(1:1000,1),gt2d(1:1000,2),gtREAD(1:1000,1),'*')


TEST_TOP_RMSE2d_2=RMSE2d;
TEST_TOP_RMSE_2=RMSE;
TEST_TOP_hd_2=hd;
TEST_TOP_hd2d_2=hd2d;
save('TEST_TOP_hd_2.mat','TEST_TOP_hd_2');
save('TEST_TOP_RMSE2d_2.mat','TEST_TOP_RMSE2d_2');
save('TEST_TOP_RMSE_2.mat','TEST_TOP_RMSE_2');
save('TEST_TOP_hd2d_2.mat','TEST_TOP_hd2d_2');


