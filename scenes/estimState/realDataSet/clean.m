clear 
close all

%% Read Ground Truth Shape = matrix [timeStep x 3Nc] ( Nc = catheter's nodes)
simuStep=2110;
GT=dlmread('rszrealAll3');
x=2:2:size(GT,1)-2;
    G=zeros(size(x,1),size(GT,2));
  for i=1:size(x,2)
      G(i,:)=GT(x(i),:);
  end
  G=G(1:simuStep,:);
%% Read Filter Reconstruction

q1=dlmread('test1');
q1=q1(1:simuStep,1:36); % read positions from filter final State Vector
q2=dlmread('test2');
q2=q2(1:simuStep,1:36);
q3=dlmread('test3');
q3=q3(1:simuStep,1:36);

Nc=size(q1,2)/3;
ALL=cat(3,q1,q2,q3);
NbCat=size(ALL,3);

%% Interpolate Plus Finement la beam and Compute Hausdorff Distance
IG=interpolateResize(G); % rewrite as 3D tensor [3 x Nc x timeStep]

IF=zeros(3,Nc,simuStep,NbCat);
for k=1:NbCat
IF(:,:,:,k)=interpolateResize(ALL(:,:,k)); % rewrite as 3D tensor [3 x Nc x timeStep]
end

N=100; 
splFilter=zeros(N,3,simuStep);
splGt=zeros(N,3,simuStep);


h=zeros(simuStep,NbCat);

for i=1:simuStep
   splGt(:,:,i) = interparc(N,IG(1,:,i),IG(2,:,i),IG(3,:,i),'linear');
   for k=1:NbCat
      splFilter(:,:,i,k) = interparc(N,IF(1,:,i,k),IF(2,:,i,k),IF(3,:,i,k),'linear');
      h(i,k)=HausdorffDist(splFilter(:,:,i,k),splGt(:,:,i));
   end
end

%% Compute Mean Distance at the Distal Segment 

d=zeros(simuStep,NbCat);
for i=1:simuStep
        for k=1:NbCat
               d(i,k)=mean(sqrt(sum((splFilter(1:10,:,i,k)'-splGt(1:10,:,i)').^2)));
        end
end 


%% Compute RMSE at the Tip

tGT=zeros(simuStep,3);
tALL=zeros(simuStep,3,NbCat);
RMSE=zeros(simuStep,NbCat);
for i=1:simuStep
        tGT(i,:)=[G(i,1) G(i,2) G(i,3)];
        for k=1:NbCat
        tALL(i,:,k)=[ALL(i,1,k) ALL(i,2,k) ALL(i,3,k)];
        RMSE(i,k)=sqrt(mean((tALL(i,:,k) - tGT(i,:)).^2,2));
        end
end 

% %% Compute mean and std
% 
% hM=mean(mean(h))*1000;
% hStd=std(mean(h))*1000;
% dM=mean(mean(d))*1000;
% dStd=std(mean(d))*1000;
% RMSEM=mean(mean(RMSE))*1000;
% RMSEStd=std(mean(RMSE))*1000;



%% Plot  Hausdorff Distance
figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',figure0);
hold on
grid(gca,'minor')
grid on
plot(h(:,1)*1000,'r','DisplayName','Q - R','Linewidth',3)
hold on
plot(h(:,2)*1000,'b','DisplayName','Q_{inf}- R_{inf}' ,'Linewidth',3)
hold on
plot(h(:,3)*1000,'g','DisplayName','Q_{sup}- R_{sup}','Linewidth',3)
set(figure0,'Units','Inches');
pos = get(figure0,'Position');
set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(axes1,'FontSize',50)
xlabel('T','FontSize',50);
ylabel('[mm]','FontSize',50);
axis([0 2130 0 8])
legend1 = legend(axes1,'show');
set(legend1,'FontSize',60);    
print(figure0,'/home/rtrivi/Documents/real_h','-dpdf','-r0')



%% Plot  Mean Distance at the distal segment
figure1 = figure('units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',figure1);
hold on
grid(gca,'minor')
grid on
plot(d(:,1)*1000,'r','DisplayName','Q - R','Linewidth',3)
hold on
plot(d(:,2)*1000,'b','DisplayName','Q_{inf}- R_{inf}' ,'Linewidth',3)
hold on
plot(d(:,3)*1000,'g','DisplayName','Q_{sup}- R_{sup}','Linewidth',3)
set(figure1,'Units','Inches');
pos = get(figure1,'Position');
set(figure1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(axes1,'FontSize',50)
xlabel('T','FontSize',50);
ylabel('[mm]','FontSize',50);
axis([0 2130 0 8])
legend1 = legend(axes1,'show');
set(legend1,'FontSize',60);    
print(figure0,'/home/rtrivi/Documents/real_d','-dpdf','-r0')

%% Plot  Mean Distance at the distal segment

figure2 = figure('units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',figure2);
hold on
grid(gca,'minor')
grid on
plot(RMSE(:,1)*1000,'r','DisplayName','Q - R','Linewidth',3)
hold on
plot(RMSE(:,2)*1000,'b','DisplayName','Q_{inf}- R_{inf}' ,'Linewidth',3)
hold on
plot(RMSE(:,3)*1000,'g','DisplayName','Q_{sup}- R_{sup}','Linewidth',3)
set(figure2,'Units','Inches');
pos = get(figure2,'Position');
set(figure2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(axes1,'FontSize',50)
xlabel('T','FontSize',50);
ylabel('[mm]','FontSize',50);
axis([0 2130 0 8])
legend1 = legend(axes1,'show');
set(legend1,'FontSize',60);    
print(figure0,'/home/rtrivi/Documents/real_RMSE','-dpdf','-r0')