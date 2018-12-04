function writeStateSofa( M,filename)
% write a state compatible with sofa, to be modified
% in kate to add T= and X=

t = 0.001:0.001:size(M,1)*0.001';


C=zeros(size(M ,1),size(M ,2)-1);
D=cat(2,t',C);
sofaM=zeros(size(M,1)*2,size(M,2)+2);
sofaM(:,1)=595959595;
sofaM(:,2)=55555888;
for i=0:(size(M,1)-2)
    sofaM(2*i+1,2)=333333;
% 
end
for i=0:(size(M,1)-2)
    sofaM(2*i+1,3:end)=D(i+1,:);
    sofaM(2*i+2,3:end)=M(i+2,:);
end
% 


dlmwrite(filename,sofaM,'delimiter',' ');


end

