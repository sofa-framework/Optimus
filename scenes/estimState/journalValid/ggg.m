clear
close all
clc 

readS_0= dlmread('matlab/print_state_real_0'); %PARAM ESTIMATION R = 0.001
readS_1= dlmread('matlab/print_state_real_1'); %PARAM ESTIMATION R >>
readS_2= dlmread('matlab/print_state_real_2'); %PARAM ESTIMATION R <<
readS_3= dlmread('matlab/print_state_real_3'); %% STATE ESTIM ***BEST***
readS_4= dlmread('matlab/print_state_real_4'); %% STATE ESTIM Q_posvel <<
readS_5= dlmread('matlab/print_state_real_5');%% STATE ESTIM Q_posvel >>
readS_6= dlmread('matlab/print_state_real_6');%% STATE ESTIM Q_forces <<
readS_7= dlmread('matlab/print_state_real_17');%% STATE ESTIM Q_forces >>
readS_8= dlmread('matlab/print_state_real_15');%% STATE ESTIM Q_forces >> Q_posvel>>
readS_9= dlmread('matlab/print_state_real_16'); %%ONLY POSITION AND VELOCITY

readG=dlmread('matlabreferenceReal');

g=readG(:,2:19);

s_0=...
    cat(2,...
    readS_0(1:1236,1:3),readS_0(1:1236,7:9),...
    readS_0(1:1236,13:15),readS_0(1:1236,19:21),...
    readS_0(1:1236,25:27),readS_0(1:1236,31:33));

s_1=...
    cat(2,...
    readS_1(1:1236,1:3),readS_1(1:1236,7:9),...
    readS_1(1:1236,13:15),readS_1(1:1236,19:21),...
    readS_1(1:1236,25:27),readS_1(1:1236,31:33));
% 
s_2=...
    cat(2,...
    readS_2(1:1236,1:3),readS_2(1:1236,7:9),...
    readS_2(1:1236,13:15),readS_2(1:1236,19:21),...
    readS_2(1:1236,25:27),readS_2(1:1236,31:33));

s_3=...
    cat(2,...
    readS_3(1:1236,1:3),readS_3(1:1236,7:9),...
    readS_3(1:1236,13:15),readS_3(1:1236,19:21),...
    readS_3(1:1236,25:27),readS_3(1:1236,31:33));

s_4=...
    cat(2,...
    readS_4(1:1236,1:3),readS_4(1:1236,7:9),...
    readS_4(1:1236,13:15),readS_4(1:1236,19:21),...
    readS_4(1:1236,25:27),readS_4(1:1236,31:33));


s_5=...
    cat(2,...
    readS_5(1:1236,1:3),readS_5(1:1236,7:9),...
    readS_5(1:1236,13:15),readS_5(1:1236,19:21),...
    readS_5(1:1236,25:27),readS_5(1:1236,31:33));

s_6=...
    cat(2,...
    readS_6(1:1236,1:3),readS_6(1:1236,7:9),...
    readS_6(1:1236,13:15),readS_6(1:1236,19:21),...
    readS_6(1:1236,25:27),readS_6(1:1236,31:33));

s_7=...
    cat(2,...
    readS_7(1:1236,1:3),readS_7(1:1236,7:9),...
    readS_7(1:1236,13:15),readS_7(1:1236,19:21),...
    readS_7(1:1236,25:27),readS_7(1:1236,31:33));

s_8=...
    cat(2,...
    readS_8(1:1236,1:3),readS_8(1:1236,7:9),...
    readS_8(1:1236,13:15),readS_8(1:1236,19:21),...
    readS_8(1:1236,25:27),readS_8(1:1236,31:33));

s_9=...
    cat(2,...
    readS_9(1:1236,1:3),readS_9(1:1236,7:9),...
    readS_9(1:1236,13:15),readS_9(1:1236,19:21),...
    readS_9(1:1236,25:27),readS_9(1:1236,31:33));

S=cat(3,s_8,s_9,s_7); 

A_0=ones(size(g,1),6,size(S,3));
H=ones(size(g,1),size(S,3));

for j=1:size(S,3);
    for i=1:size(S,1);
       A_0(i,1,j)=norm(S(i,1:3,j)-g(i,1:3));
       A_0(i,2,j)=norm(S(i,4:6,j)-g(i,4:6));
       A_0(i,3,j)=norm(S(i,7:9,j)-g(i,7:9));
       A_0(i,4,j)=norm(S(i,10:12,j)-g(i,10:12));
       A_0(i,5,j)=norm(S(i,13:15,j)-g(i,13:15));
       A_0(i,6,j)=norm(S(i,16:18,j)-g(i,16:18));   
       
       H(i,j)=hausd(S(i,:,j)',g(i,:)');
         ylim([0 0.0045])

    end
end

% for j=1:size(S,3);
figure
% for i=1:6 
  grid minor
  plot(A_0(:,6,1),'LineWidth',2,'Color',[0 0.5 0.5])
  hold on
  plot(A_0(:,1,1),'LineWidth',2,'Color',[0.5 0.5 0.5])
  hold on
  plot(A_0(:,2,1),'LineWidth',2,'Color',[0.5 0 0.5])
  hold on
  plot(A_0(:,3,1),'LineWidth',2,'Color',[0.5 0.5 0])
  hold on
  plot(A_0(:,4,1),'LineWidth',2,'Color',[1 0.5 0.7])
  hold on
  plot(A_0(:,5,1),'LineWidth',2,'Color',[0.5 1 0.2])
  hold on
  grid on
  plot(H(:,1),'LineWidth',2,'Color',[1 0 0])
  hold on
  ylim([0 0.0045])
    legend('1°','2°','3°','4°','5°','6°','Hausdorff')

% end
% hold on
% for i=1:6 
figure
  plot(A_0(:,6,2),'LineWidth',2,'Color',[0 0.5 0.5],'LineStyle','--')
  hold on
  plot(A_0(:,1,2),'LineWidth',2,'Color',[0.5 0.5 0.5],'LineStyle','--')
  hold on
  plot(A_0(:,2,2),'LineWidth',2,'Color',[0.5 0 0.5],'LineStyle','--')
  hold on
  plot(A_0(:,3,2),'LineWidth',2,'Color',[0.5 0.5 0],'LineStyle','--')
  hold on
  plot(A_0(:,4,2),'LineWidth',2,'Color',[1 0.5 0.7],'LineStyle','--')
  hold on
  plot(A_0(:,5,2),'LineWidth',2,'Color',[0.5 1 0.2],'LineStyle','--')
  hold on
  grid on
  plot(H(:,2),'LineWidth',2,'Color',[1 0 0],'LineStyle','--')
     legend('1°','2°','3°','4°','5°','6°','Hausdorff')
  ylim([0 0.0045])
  
  
  figure
  plot(A_0(:,6,3),'LineWidth',2,'Color',[0 0.5 0.5],'LineStyle',':')
  hold on
  plot(A_0(:,1,3),'LineWidth',2,'Color',[0.5 0.5 0.5],'LineStyle',':')
  hold on
  plot(A_0(:,2,3),'LineWidth',2,'Color',[0.5 0 0.5],'LineStyle',':')
  hold on
  plot(A_0(:,3,3),'LineWidth',2,'Color',[0.5 0.5 0],'LineStyle',':')
  hold on
  plot(A_0(:,4,3),'LineWidth',2,'Color',[1 0.5 0.7],'LineStyle',':')
  hold on
  plot(A_0(:,5,3),'LineWidth',2,'Color',[0.5 1 0.2],'LineStyle',':')
  hold on
  grid on
  plot(H(:,3),'LineWidth',2,'Color',[1 0 0],'LineStyle',':')
     legend('1°','2°','3°','4°','5°','6°','Hausdorff')
  ylim([0 0.0045])
% hold off
% end
