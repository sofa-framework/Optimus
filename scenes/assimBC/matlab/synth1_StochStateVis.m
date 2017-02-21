suffix='psd2#osd0.0001#ogrid4';
%suffix='psd0.0003#osd1e-06#ctr0#MJED'

directory='../outSynth1';
numStep = 300;
%close all

s=load(sprintf('%s/state_%s.txt', directory, suffix));
v=load(sprintf('%s/variance_%s.txt', directory, suffix));
cv=load(sprintf('%s/covariance_%s.txt', directory, suffix));
tf=load(sprintf('%s/toolForce_%s.txt', directory, suffix));
gtf=load('../observations/toolForce.txt');

npar=16;
showForces=1

figure1 = figure;
set(figure1,'Renderer','OpenGL');
set(figure1,'Position', [0 0 850 250]);
%set(figure1,'OuterPosition', [0 0 850 250]);
set(figure1,'Color',[1 1 1]);
set(figure1,'InvertHardcopy','off');
axes1 = axes('Parent',figure1, 'Position',[0.0558722919042189 0.173333333333333 0.921584948688712 0.803809523809524]);
hold(axes1,'on');

for i=1:npar
    cls(i) = 'c';
end

cls(1) = 'r';
cls(4) = 'r';
cls(10) = 'g';
cls(16) = 'g';
cls(9) = 'b';
cls(15) = 'b';
cls(8) = 'm';
cls(14) = 'm';

hold on
for i=1:npar
    errorbar(s(1:numStep,i),sqrt(v(1:numStep,i)), 'Color', cls(i));
end

%errorbar(s(1:numStep,:), sqrt(v(1:numStep,:)));
title(sprintf('%s %s',directory,suffix));
xlim([0 numStep])
grid on
xlabel('Time')
ylabel('Stiffness [N/m]')
set(axes1,'FontSize',14);

if showForces
    figure; 
    plot(1:numStep, abs(tf(1:numStep,:)), 'r-', 1:numStep, abs(gtf(1:numStep,:)), 'b-');
end
