%visualization of the feature error for pig: requires featError
%file to be exported (3 features) by the pigLiver scene


% suffix1='psd0.01#osd1e-06#ctr3#MJED';
ctr=3;
suffix2='psd0.005#osd1e-06#ctr3#MJED';
suffix3='givenStiffness0#ctr3#MJED';
suffix4='givenStiffness100#ctr3#MJED';
ylimit=12;

%suffix1='psd0.000#osd1e-06#ctr0#MJED-estimated130';
% ctr=0;
% suffix2='psd0.0002#osd1e-06#ctr0#MJED';
% suffix3='givenStiffness0#ctr0#MJED';
% suffix4='givenStiffness100#ctr0#MJED';
% ylimit=18;

directory='../outLiver';
numStep = 140;

%close all

showSynthStoch=0;
showLiverStoch=0;
showLiverPredicted=0;
showLiverFeatError=1;

%fe1=load(sprintf('%s/featError_%s.txt', directory, suffix1));
fe2=1000*load(sprintf('%s/featError_%s.txt', directory, suffix2));
fe3=1000*load(sprintf('%s/featError_%s.txt', directory, suffix3));
fe4=1000*load(sprintf('%s/featError_%s.txt', directory, suffix4));

leg1='Stochastic BC';
    leg2='No BC (free nodes)';
    leg3='Fixed BC';


for i=1:3
    
    figure1 = figure;
    set(figure1,'Renderer','OpenGL');
    set(figure1,'Position', [0 0 850 250]);
    %set(figure1,'OuterPosition', [0 0 850 250]);
    set(figure1,'Color',[1 1 1]);
    set(figure1,'InvertHardcopy','off');
    axes1 = axes('Parent',figure1, 'Position',[0.0558722919042189 0.173333333333333 0.921584948688712 0.803809523809524]);
    hold(axes1,'on');

    %plot(fe1(1:numStep,1), 'r-');
    plot(fe2(1:numStep,i), 'r-','LineWidth',2);
    plot(fe3(1:numStep,i), 'g-','LineWidth',2);
    plot(fe4(1:numStep,i), 'b-','LineWidth',2);
    legend1=legend(leg1, leg2, leg3);
    grid on
    %title(sprintf('Feature %d', i))
    xlim([0 numStep])
    ylim([0 ylimit])    
    xlabel('Time')
    ylabel('Error [mm]')
    set(legend1,'Position',[0.143272983470998 0.636170011359333 0.244583808437856 0.239208633093525],'FontSize',20);
    set(axes1,'FontSize',14);
    
    %saveas(figure1, sprintf('ctr%d_feat%d.png', ctr, i));
end

return

figure;
hold on
%plot(fe1(1:numStep,2), 'r-');
plot(fe2(1:numStep,2), 'r-','LineWidth',2);
plot(fe3(1:numStep,2), 'g-','LineWidth',2);
plot(fe4(1:numStep,2), 'b-','LineWidth',2);
%legend(suffix1, suffix2, suffix3, suffix4);
legend(leg1, leg2, leg3);
grid on
title('Feature 2')

figure;
hold on
%plot(fe1(1:numStep,3), 'r-');
plot(fe2(1:numStep,3), 'r-','LineWidth',2);
plot(fe3(1:numStep,3), 'g-','LineWidth',2);
plot(fe4(1:numStep,3), 'b-','LineWidth',2);
%legend(suffix1, suffix2, suffix3, suffix4);
legend(leg1, leg2, leg3);
grid on
title('Feature 3')
%title(sprintf('Feature error: %s %s Step: %d',directory,suffix, numStep));    


