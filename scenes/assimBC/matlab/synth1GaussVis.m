suffix='psd5#osd0.0001#ogrid4';
%suffix='psd0.0003#osd1e-06#ctr0#MJED'

directory='../outSynth1';
%close all

s=load(sprintf('%s/state_%s.txt', directory, suffix));
v=load(sprintf('%s/variance_%s.txt', directory, suffix));

ns = length(s);

s=s(ns, :);
v=v(ns, :);

lowS=find(s<0.1);
nls = length(lowS);

x=-15:0.1:180;
close all

figure1 = figure;
set(figure1,'Renderer','OpenGL');
set(figure1,'Position', [0 0 400 130]);
%set(figure1,'OuterPosition', [0 0 850 250]);
set(figure1,'Color',[1 1 1]);
set(figure1,'InvertHardcopy','off');
axes1 = axes('Parent',figure1); %, 'Position',[0.0558722919042189 0.173333333333333 0.921584948688712 0.803809523809524]);
hold(axes1,'on');

plot(x, normpdf(x, 0, 5), 'Color', 'k', 'LineWidth', 3);
for i=1:length(s) 
    if v(i) < 0.1
        v(i)=0.1;
    end
    pdf=normpdf(x, s(i), v(i));
    if i==1 || i == 4
        col = 'r';
    elseif i == 10 || i == 16
        col = [0 0.8 0];
    elseif i == 9 || i == 15
        col = [1 0.64 0];
    elseif i == 8 || i == 14
        col = 'c';
    else
        col = 'c';
    end
    plot(x,pdf, 'Color', col, 'LineWidth', 2);
end
xlim([-15,170]);
ylim([0 1]);

grid on
box on
xlabel('Spring stiffness [N]')
ylabel('Probability') 
set(axes1,'FontSize',14);

[s' v']

