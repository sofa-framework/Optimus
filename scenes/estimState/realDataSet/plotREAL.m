% clear
% close all
% 
% simuStep=2100;
% GT=dlmread('rszrealAll3');
% x=2:2:size(GT,1)-2;
%     G=zeros(size(x,1),size(GT,2));
%   for i=1:size(x,2)
%       G(i,:)=GT(x(i),:);
%   end
% 
% q1=dlmread('Q1_R1');
% q1=q1(:,1:36);
% q2=dlmread('Q1_R2');
% q2=q2(:,1:36);
% q3=dlmread('Q1_R3');
% q3=q3(:,1:36);
% 
% q4=dlmread('Q2_R1');
% q4=q4(:,1:36);
% q5=dlmread('Q2_R2');
% q5=q5(:,1:36);
% q6=dlmread('Q2_R3');
% q6=q6(:,1:36);
% 
% 
% q7=dlmread('Q3_R1');
% q7=q7(:,1:36);
% q8=dlmread('Q3_R2');
% q8=q8(:,1:36);
% q9=dlmread('Q3_R3');
% q9=q9(:,1:36);
% 
% 
% ALLvisu=cat(3,q1,q2,q3,q4,q5,q6,q7,q8,q9);
%  
% %% Interpolate Plus Finement la beam
% I=interpolateResize(G);
% 
% IfilterVisu=zeros(3,size(q3,2)/3,simuStep,size(ALLvisu,3));
% 
% ALLreal=ALLvisu;
% for k=1:size(ALLreal,3)
% IfilterVisu(:,:,:,k)=interpolateResize(ALLvisu(:,:,k));
% 
% end
% N=100;
% 
% 
% 
% splFilterVisu=zeros(N,3,simuStep);
% splGt=zeros(N,3,simuStep);
% h=zeros(simuStep,size(ALLreal,3));
% h_visu=zeros(simuStep,size(ALLvisu,3));
% 
% for i=1:simuStep
%    splGt(:,:,i) = interparc(N,I(1,:,i),I(2,:,i),I(3,:,i),'linear');
%    for k=1:size(ALLreal,3)
%       splFilterVisu(:,:,i,k) = interparc(N,IfilterVisu(1,:,i,k),IfilterVisu(2,:,i,k),IfilterVisu(3,:,i,k),'linear');
%       h_visu(i,k)=HausdorffDist(splFilterVisu(1:50,:,i,k),splGt(1:50,:,i));
% 
%    end
% end

%% PLOT HAUSDORFF DISTANCE 

%% S 
figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',figure0);
grid(gca,'minor')
grid on
h = animatedline('Parent',axes1,'LineWidth',2,...
    'Color',[0 0 1],'DisplayName','s - σ_{obs}=0.01');
axis([0 2130 0 8])
x = linspace(0,2100,2100);
y = h_visu(:,2)*1000;
for k = 1:length(x)
    addpoints(h,x(k),y(k));
    drawnow

end

   figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',figure0);
hold on
grid(gca,'minor')
grid on
plot(h_visu(:,1)*1000,'r','DisplayName','s - σ_{obs}=0.1','Linewidth',3)
hold on
plot(h_visu(:,2)*1000,'b','DisplayName','s - σ_{obs}=0.01','Linewidth',3)
hold on
plot(h_visu(:,3)*1000,'g','DisplayName','s - σ_{obs}=0.001','Linewidth',3)
set(figure0,'Units','Inches');
pos = get(figure0,'Position');
set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(axes1,'FontSize',50)
xlabel('T h','FontSize',50);
ylabel('[mm]','FontSize',50);
xlim([0 2130])
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
    'FontSize',50);
% print(figure0,'/home/raffa/Documents/h_realS','-dpdf','-r0')

%  %% S_{inf} 
%  
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(h_visu(:,4)*1000,'r','DisplayName','s_{inf} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(h_visu(:,5)*1000,'b','DisplayName','s_{inf} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(h_visu(:,6)*1000,'g','DisplayName','s_{inf} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T h','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% % print(figure0,'/home/raffa/Documents/h_realSinf','-dpdf','-r0')
% 
% %% S_{sup} 
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(h_visu(:,7)*1000,'r','DisplayName','s_{sup} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(h_visu(:,8)*1000,'b','DisplayName','s_{sup} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(h_visu(:,9)*1000,'g','DisplayName','s_{sup} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T h','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% % print(figure0,'/home/raffa/Documents/h_realSSup','-dpdf','-r0')



%% Compute mean dist at TIP

ALL=ALLvisu;
% d_visu=zeros(simuStep,size(ALL,3));
% for i=1:simuStep
%         for k=1:1:size(ALL,3)
%             V=splFilterVisu(1:10,:,i,k)-splGt(1:10,:,i);
%                d_visu(i,k)=mean(sqrt(diag(V*V')));
%         end
% end
% 
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(d_visu(:,1)*1000,'r','DisplayName','s - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(d_visu(:,2)*1000,'b','DisplayName','s - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(d_visu(:,3)*1000,'g','DisplayName','s - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T d','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/d_realS','-dpdf','-r0')
% 
%  %% S_{inf} 
%  
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(d_visu(:,4)*1000,'r','DisplayName','s_{inf} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(d_visu(:,5)*1000,'b','DisplayName','s_{inf} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(d_visu(:,6)*1000,'g','DisplayName','s_{inf} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T d','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/d_realSinf','-dpdf','-r0')
% 
% %% S_{sup} 
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(d_visu(:,7)*1000,'r','DisplayName','s_{sup} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(d_visu(:,8)*1000,'b','DisplayName','s_{sup} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(d_visu(:,9)*1000,'g','DisplayName','s_{sup} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T d','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/d_realSSup','-dpdf','-r0')
% 
% %% Compute DIST Tip
% tGT=zeros(simuStep,3);
% tALL=zeros(simuStep,3,size(ALL,3));
% RMSE_visu=zeros(simuStep,size(ALL,3));
% for i=1:simuStep
%         tGT(i,:)=[G(i,1) G(i,2) G(i,3)];
%         for k=1:size(ALL,3)
%         tALL(i,:,k)=[ALL(i,1,k) ALL(i,2,k) ALL(i,3,k)];
%         RMSE_visu(i,k)=norm(tALL(i,:,k) - tGT(i,:));
%         end
% end
% 
% 
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(RMSE_visu(:,1)*1000,'r','DisplayName','s - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(RMSE_visu(:,2)*1000,'b','DisplayName','s - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(RMSE_visu(:,3)*1000,'g','DisplayName','s - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T RMSE','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/RMSE_realS','-dpdf','-r0')
% 
%  %% S_{inf} 
%  
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(RMSE_visu(:,4)*1000,'r','DisplayName','s_{inf} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(RMSE_visu(:,5)*1000,'b','DisplayName','s_{inf} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(RMSE_visu(:,6)*1000,'g','DisplayName','s_{inf} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T RMSE','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/RMSE_realSinf','-dpdf','-r0')
% 
% %% S_{sup} 
% figure0 = figure('units','normalized','outerposition',[0 0 1 1]);
% axes1 = axes('Parent',figure0);
% hold on
% grid(gca,'minor')
% grid on
% plot(RMSE_visu(:,7)*1000,'r','DisplayName','s_{sup} - σ_{obs}=0.1','Linewidth',3)
% hold on
% plot(RMSE_visu(:,8)*1000,'b','DisplayName','s_{sup} - σ_{obs}=0.01','Linewidth',3)
% hold on
% plot(RMSE_visu(:,9)*1000,'g','DisplayName','s_{sup} - σ_{obs}=0.001','Linewidth',3)
% set(figure0,'Units','Inches');
% pos = get(figure0,'Position');
% set(figure0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% set(axes1,'FontSize',50)
% xlabel('T RMSE','FontSize',50);
% ylabel('[mm]','FontSize',50);
% xlim([0 2130])
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.151171102146082 0.516524695744691 0.398921821654004 0.397602385901786],...
%     'FontSize',50);
% print(figure0,'/home/raffa/Documents/RMSE_realSSup','-dpdf','-r0')
% 
% % %% PLOT DIST AT TIP 

