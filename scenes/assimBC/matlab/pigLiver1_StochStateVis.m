suffix='psd0.01#osd1e-06#ctr3#MJED';
%suffix='psd0.0003#osd1e-06#ctr0#MJED'

directory='../outLiver';
numStep = 140;
%close all

s=load(sprintf('%s/state_%s.txt', directory, suffix));
v=load(sprintf('%s/variance_%s.txt', directory, suffix));
cv=load(sprintf('%s/covariance_%s.txt', directory, suffix));


figure1 = figure;
set(figure1,'Renderer','OpenGL');
set(figure1,'Position', [0 0 850 250]);
%set(figure1,'OuterPosition', [0 0 850 250]);
set(figure1,'Color',[1 1 1]);
set(figure1,'InvertHardcopy','off');
axes1 = axes('Parent',figure1, 'Position',[0.0558722919042189 0.173333333333333 0.921584948688712 0.803809523809524]);
hold(axes1,'on');
errorbar(s(1:numStep,:), sqrt(v(1:numStep,:)));
title(sprintf('%s %s',directory,suffix));
xlim([0 numStep])
grid on
xlabel('Time')
ylabel('Stiffness [N/m]')
set(axes1,'FontSize',14);
