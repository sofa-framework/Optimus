
% A=rand(3,3);
% 
% 
%     
%     determinant =  A(1,1)*(A(3,3)*A(2,2) - A(3,2)*A(2,3)) - ...
%                    A(2,1)*(A(3,3)*A(1,2) - A(3,2)*A(1,3)) + ...
%                    A(3,1)*(A(2,3)*A(1,2) - A(2,2)*A(1,3));
%     
%     
%     invdet = 1/determinant;
%     result(1,1) =  (A(2,2)*A(3,3)-A(3,2)*A(2,3))*invdet;
%     result(1,2) = -(A(1,2)*A(3,3)-A(3,2)*A(1,3))*invdet;
%     result(1,3) =  (A(1,2)*A(2,3)-A(1,3)*A(2,2))*invdet;
%     
%     result(2,1) = -(A(2,1)*A(3,3)-A(3,1)*A(2,3))*invdet;
%     result(2,2) =  (A(1,1)*A(3,3)-A(1,3)*A(3,1))*invdet;
%     result(2,3) = -(A(1,1)*A(2,3)-A(1,3)*A(2,1))*invdet;
%     
%     result(3,1) =  (A(2,1)*A(3,2)-A(3,1)*A(2,2))*invdet;     
%     result(3,2) = -(A(1,1)*A(3,2)-A(1,2)*A(3,1))*invdet;        
%     result(3,3) =  (A(1,1)*A(2,2)-A(1,2)*A(2,1))*invdet;
%     
%     
%     result
%     inv(A)
%     
%     inv(A)-result


A=rand(3,3);
B=rand(3,3);

for i=1:3
    for j=1:3
        C(i,j) = 0;
        for k=1:3
            C(i,j) = C(i,j)+ A(i,k)*B(j,k);
        end
    end
end
C

A*B'