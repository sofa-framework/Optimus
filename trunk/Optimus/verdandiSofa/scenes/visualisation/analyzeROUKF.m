clear 

prefix='cylTest/p6';

Xf=sprintf('%s_estim.out', prefix);
Pf=sprintf('%s_var.out', prefix);

defVals=[10000 5000 7000];

maxT=400;


%================================================

close all

X=load(Xf);
numPar=size(X,2);

P=load(Pf);

X=X(1:maxT,:);
P=P(1:maxT,:);

Xd=repmat(defVals,size(X,1),1);



%show parameter assimilation
maxY=max(max([X; Xd]));
maxY=1.05*maxY;
minY=min(min([X; Xd]));
minY=0.95*minY;
figure1 = figure('XVisual',...
    '0x27 (TrueColor, depth 24, RGB mask 0xff0000 0xff00 0x00ff)',...
    'InvertHardcopy','off',...
    'Color',[1 1 1]);
axes('Ylim',[minY maxY],  'FontSize',16);
xlabel('#iteration', 'FontSize',16);
ylabel('parameter value',  'FontSize',16);
hold on
plot(X, 'LineWidth', 2); grid on; 
%leg1=legend('P1', 'P2', 'P3')
%set(leg1,'Position',[0.736160714285711 0.160714285714285 0.138392857142857 0.229166666666667]);
plot(Xd, 'LineWidth', 2, 'LineStyle','--'); 

saveas(gcf,sprintf('%s_estim.eps', prefix), 'psc2');

%show evolution of uncertainty
numVar=numPar;
numCovar=nchoosek(numVar, 2);
numLines=numVar+numCovar;

varP=(abs(P(:,1:numLines)));
for i=1:numVar
    lg{i} = sprintf('Var P%d', i);
end

k=numVar+1;
for i=2:numVar
    for j=1:i-1
        lg{k} = sprintf('Covar P%d-P%d', j,i);
        k=k+1;
    end
end;


%covarP=sqrt(abs(P(:,numPar+1:2*numPar)));
figure1 = figure('XVisual',...
    '0x27 (TrueColor, depth 24, RGB mask 0xff0000 0xff00 0x00ff)',...
    'InvertHardcopy','off',...
    'Color',[1 1 1]);
axes('FontSize',16);
xlabel('#iteration', 'FontSize',16);
ylabel('variance', 'FontSize',16);
hold on
plot(varP, 'LineWidth', 2); grid on; 
%plot(-varP, 'LineWidth', 1);
legend(lg);
%plot(covarP, 'LineWidth', 1, 'LineStyle','--'); 
%plot(-covarP, 'LineWidth', 1, 'LineStyle','--');  legend('covar P1-P2', 'covar P2-P3', 'covar P1-P3');

saveas(gcf,sprintf('%s_var.eps', prefix), 'psc2');
