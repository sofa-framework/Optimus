directory='../outLiver';
numStep = 140;
%close all

cp=load(sprintf('%s/toolPoints.txt', directory));
op=load(sprintf('%s/obsPoints.txt', directory));
ap=load(sprintf('%s/assessPoints.txt', directory));


figure1 = figure;
set(figure1,'Renderer','OpenGL');
set(figure1,'Position', [0 0 500 500]);
%set(figure1,'OuterPosition', [0 0 850 250]);
set(figure1,'Color',[1 1 1]);
set(figure1,'InvertHardcopy','off');
axes1 = axes('Parent',figure1,'Position',[0.0239099859353024 0.0040650406504065 0.957805907172996 0.981707317073173]);
hold(axes1,'on');

set(axes1,'XTickLabel',{'','','','','','','','',''},'YTickLabel',{'','','','','','','','','',''});


ocp=size(op,2)/3
for i = 1:ocp 
    if i ~= 10
        x=op(1:numStep,3*(i-1)+1);
        y=-op(1:numStep,3*(i-1)+2);
        plot(x,y, 'Color' , [0 0.5+0.015*i 0], 'LineWidth', 1);
        p=plot(x(1),y(1), 'Color' , [0 0.6+0.01*i 0], 'LineWidth', 1, 'LineStyle','none');
        p.Marker='o';
        p.MarkerSize=6;
        p.MarkerFaceColor=[0 0.6+0.01*i 0];
    end    
end
nap=size(ap,2)/3
for i = 1:nap
    x=ap(1:numStep,3*(i-1)+1);
    y=-ap(1:numStep,3*(i-1)+2);
    plot(x,y, 'Color' , 'm', 'LineWidth', 3);
    p=plot(x(1),y(1), 'Color' , 'm', 'LineWidth', 1, 'LineStyle','none');
    p.Marker='o';
    p.MarkerSize=8;
    p.MarkerFaceColor='m';
end

ncp=size(cp,2)/3

for i = 1:ncp
    x=cp(1:numStep,3*(i-1)+1);
    y=-cp(1:numStep,3*(i-1)+2);
        
    if i==ncp
        plot(x,y, 'Color' , [0.2 0.2 1], 'LineWidth', 2, 'LineStyle', '--');
    else
        plot(x,y, 'Color' , [0.2 0.2 1], 'LineWidth', 2);
    end
        
    p=plot(x(1),y(1), 'Color' , [0 0 1], 'LineWidth', 1, 'LineStyle','none');
    p.Marker='o';
    p.MarkerSize=8;
    p.MarkerFaceColor='b';
        
end
box(axes1,'on');
grid(axes1,'on');



