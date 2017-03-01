suffix='psd0.01#osd1e-06#ctr3#MJED';
%suffix='psd0.0003#osd1e-06#ctr0#MJED'

directory='../outLiver';
numStep = 140;
%close all

showSynthStoch=0;
showLiverStoch=1;
showLiverPredicted=0;
showLiverFeatError=0;

if showLiverStoch + showLiverPredicted > 0
    s=load(sprintf('%s/state_%s.txt', directory, suffix));
    v=load(sprintf('%s/variance_%s.txt', directory, suffix));
    cv=load(sprintf('%s/covariance_%s.txt', directory, suffix));
end
%tf=load(sprintf('%s/toolForce_%s.txt', directory, suffix));
fe=load(sprintf('%s/featError_%s.txt', directory, suffix));

    
if showLiverStoch == 1
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
end

% lastCV=cv(size(cv,1), :);
% covmat=zeros(npar, npar
% gli=1;
% for i=1:npar
%     for j=i+1:npar
%         covmat(i,j) = lastCV(gli);
%         covmat(j,i) = lastCV(gli);
%         gli=gli+1;
%     end
% end


if showSynthStoch == 1
    npar=size(v,2);

    figure;
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
        errorbar(s(:,i),sqrt(v(:,i)), 'Color', cls(i));
    end
    title([directory '-' suffix]);
    %legend('1','2');        
end

if showLiverPredicted== 1            
    figure; 
    plot(s(numStep,:), 'x');
    grid on
    title(sprintf('Expected values: %s %s Step: %d',directory,suffix, numStep));
    
    figure; 
    plot(v(numStep,:), 'x');
    grid on
    title(sprintf('Standard deviation: %s %s Step: %d',directory,suffix, numStep));
end



