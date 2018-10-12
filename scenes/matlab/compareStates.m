n=143;
%m=n+1  %UKF
m=3;  % ROUKF

pth='../assimStiffness/states_ROUKF';
statesIN=zeros(n, n+1);
statesOUT=zeros(n, n+1);
for i = 1:m
    nm = sprintf('%s/SP_%d_IN.txt', pth, i-1);
    a=load(nm);
    statesIN(:,i) = a;
    
    nm = sprintf('%s/SP_%d_OUT.txt', pth, i-1);
    a=load(nm);
    statesOUT(:,i) = a;
end

format short g

disp('IN:');
for i= 1:m
    df = statesIN(:,i) - statesIN(:,1);
    statesIN(:,i)
    %disp(statesIN(142:143,i)');
    %if (i == 20)
        disp(i);
        %disp(max(df));
        %df
    %end
end

return

disp('OUT:');
ndfs=zeros(1,n);
for i= 2:m
    %disp(i);
    df = statesOUT(:,i) - statesOUT(:,1);    
    ndfs(i)=norm(df);
    %if (ndfs(i) > 1e-8)
        fprintf('%d: %g\n', i, ndfs(i));
        %disp(nonzeros(df))
    %end
end