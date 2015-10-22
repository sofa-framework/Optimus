min=0;
max=998;

pert=zeros(max-min,5,22); 
pth='clamped_bar/outmat';

Pf=zeros(max-min,22,22);


for i=min:max
    %pert(i+1,:,:)=load(sprintf('%s/Pert_%04d.txt', pth, i));
    Pf(i+1,:,:)=load(sprintf('%s/P_%04d.txt', pth, i));
end

m=10; n=11;

figure; 
hold on
plot(Pf(:,m,m));
%figure; 
plot(Pf(:,n,n));
%figure; 
plot(Pf(:,m,n));
%figure; plot(Pf(:,n,m));


%figure; plot(pert(:,3,n));
%figure; plot(pert(:,4,n));
%figure; plot(pert(:,5,n));

