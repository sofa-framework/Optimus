%% PRINTING VELOCITY AMPLITUDE 
clear all
close all

s_0= dlmread('print/print_state_nocoll');
s_1= dlmread('print/print_state_coll');
g= dlmread('LMgSpring_0');
n= dlmread('noisySpring_0');




P_0=s_0(:,1:3);
V_0=s_0(:,7:9);

ms=size(P_0,1);

G=g(1:ms, 2:4);
N=n(1:ms, 2:4);
P_1=s_1(1:ms,1:3);
V_1=s_1(1:ms,7:9);
n_v0=ones(ms,1);
n_p0=n_v0;
n_G=n_p0;
n_N=n_G;

n_p1=n_p0;
n_v1=n_v0;


for i =1:ms
   n_G(i)=norm(G(i,:));
   n_N(i)=norm(N(i,:));
   n_p0(i)=norm(P_0(i,:));
   n_v0(i)=norm(V_0(i,:));

   n_v1(i)=norm(V_1(i,:));
   n_p1(i)=norm(P_1(i,:));
end

figure1=figure('NumberTitle', 'off', 'Name', ' Compare Position Magnitude')
plot(n_G, 'LineWidth',1,'LineStyle','--','Color',[0 1 0])
grid on
hold on
plot(n_p0, 'Linewidth',0.5, 'MarkerSize',1,'Color',[0.0784313753247261 0.168627455830574 0.549019634723663])
hold on
plot(n_N, 'Linewidth',0.5,'MarkerSize',0.5,'Color',[1 0 0])
hold on 
plot(n_p1, 'Linewidth',0.5,'MarkerSize',1,'Color',[0 0.600000023841858 0.600000023841858])
% Create textbox
annotation(figure1,'textbox',...
    [0.138614678899084 0.881536819637144 0.152096330275229 0.0330843116328716],...
    'String',{'Ground Truth Position Norm'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[0 1 0]);

% Create textbox
annotation(figure1,'textbox',...
    [0.139188073394496 0.846318036286022 0.152096330275229 0.0330843116328716],...
    'String',{'Noisy Observation Norm'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[1 0 0]);

% Create textbox
annotation(figure1,'textbox',...
    [0.731504587155965 0.209178228388475 0.152096330275229 0.0330843116328718],...
    'Color',[0.941176474094391 0.941176474094391 0.941176474094391],...
    'String',{'Position Norm - COLLISION'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[0.0784313753247261 0.168627455830574 0.549019634723663]);

% Create textbox
annotation(figure1,'textbox',...
    [0.731504587155965 0.172892209178232 0.152096330275229 0.0330843116328716],...
    'Color',[0.941176474094391 0.941176474094391 0.941176474094391],...
    'String',{'Position Norm -NO COLLISION'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[0 0.600000023841858 0.600000023841858]);










figure2=figure('NumberTitle', 'off', 'Name', ' Compare Velocity Magnitude')
plot(n_v1, 'Linewidth',0.5,'MarkerSize',1,'Color',[0.0784313753247261 0.168627455830574 0.549019634723663])
hold on
grid on
plot(n_v0, 'Linewidth',0.5, 'MarkerSize',1,'Color',[0 0.600000023841858 0.600000023841858])
% Create textbox
annotation(figure2,'textbox',...
    [0.141481651376147 0.88153681963714 0.152096330275229 0.0330843116328716],...
    'Color',[0.941176474094391 0.941176474094391 0.941176474094391],...
    'String',{'Velocity Norm - COLLISION'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[0.0784313753247261 0.168627455830574 0.549019634723663]);

% Create textbox
annotation(figure2,'textbox',...
    [0.141481651376147 0.843116328708645 0.152096330275229 0.0330843116328716],...
    'Color',[0.941176474094391 0.941176474094391 0.941176474094391],...
    'String',{'Velocity Norm -NO COLLISION'},...
    'FitBoxToText','off',...
    'EdgeColor','none',...
    'BackgroundColor',[0 0.600000023841858 0.600000023841858]);