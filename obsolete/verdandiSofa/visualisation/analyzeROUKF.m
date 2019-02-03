clear 

%prefixForward='../daHeteroCylinder2150Constant/pHardSmoothIm';
%prefix='../daHeteroCylinder2150Constant/pHardSmoothIm3';
%cc=hsv(3);

%cylinder10:
prefixForward='../daHeteroCylinder4245Constant/pHardSmoothIm';
prefix='../../python_scenes/test_Parallel/daCyl10Par/surfNoise2Ab20';
%prefix='../daCyl10/param';

%cylinder3:
%prefixForward='../daHeteroCylinder770Constant/params';
%prefix='../daHeteroCylinder770Constant/paramsNS';

cc=hsv(10);

Xf=sprintf('%s_params.out', prefix);
Pf=sprintf('%s_vars.out', prefix);
Xdf=sprintf('%s_forward.out', prefixForward);


maxT=300;
dt=1;

doStd=1;


%================================================

%close all
Xf
X=0.001*load(Xf);
numPar=size(X,2);
size(X)

P=load(Pf);

size(X)

X=X(1:maxT,:);
P=P(1:maxT,:);

%Xd=repmat(defVals,size(X,1),1);
Xd=0.001*load(Xdf);
size(Xd)
Xd=Xd(1:maxT,:);

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
xlabel('#step', 'FontSize',16);
ylabel('Young''s modulus [kPa]',  'FontSize',16);
hold on
set(gca, 'ColorOrder', cc);
plot(dt*(1:maxT),X, 'LineWidth', 2); grid on; 
%leg1=legend('P1', 'P2', 'P3')
%set(leg1,'Position',[0.736160714285711 0.160714285714285 0.138392857142857 0.229166666666667]);
set(gca, 'ColorOrder', cc);
plot(dt*(1:maxT),Xd, 'LineWidth', 2, 'LineStyle','--');
set(gca, 'ColorOrder', cc);

saveas(gcf,sprintf('%s_estim.eps', prefix), 'psc2');

%show evolution of uncertainty
numVar=numPar;
numCovar=nchoosek(numVar, 2);
numLines=numVar+numCovar;

varP=(abs(P(:,1:numLines)));
size(varP)

if (doStd)
    varP=sqrt(varP);
end


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
xlabel('#step', 'FontSize',16);
ylabel('standard deviation [kPa]', 'FontSize',16);
hold on
plot(dt*(1:maxT), 0.001*varP(:,1:numVar), 'LineWidth', 2); grid on; 
%set(gcf, 'ColorOrder', cc);
%set(gca, 'ColorOrder', cc);
%plot(-varP, 'LineWidth', 1);
%legend(lg);
%plot(covarP, 'LineWidth', 1, 'LineStyle','--'); 
%plot(-covarP, 'LineWidth', 1, 'LineStyle','--');  legend('covar P1-P2', 'covar P2-P3', 'covar P1-P3');

%saveas(gcf,sprintf('%s_var.eps', prefix), 'psc2');
