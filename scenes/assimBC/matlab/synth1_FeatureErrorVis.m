%visualization of the feature error for pig: requires featError
%file to be exported (3 features) by the pigLiver scene

directory='../outSynth1';
file1='assessmentPositionsGT_allBC.txt';
file2='assess_psd5#osd0.0001#ogrid4.txt';

fe1=1000*load(sprintf('%s/%s', directory, file1));
fe2=1000*load(sprintf('%s/%s', directory, file2));

ns=200;
nf=size(fe1,2)/3;

meanErr=zeros(1,ns);
maxErr=zeros(1,ns);

for i=1:ns
    f1=fe1(i,:);
    f2=fe2(i,:);
    
    for j=1:nf
        a=[f1((j-1)*3+1) f1((j-1)*3+2)];
        b=[f2((j-1)*3+1) f2((j-1)*3+2)];
        err(j) = norm(a-b);
    end
    maxErr(i) = max(err);
    meanErr(i) = mean(err);    
end



plot(1:ns,maxErr,'r-', 1:ns, meanErr, 'b-')

return

suffix1='psd0.01#osd1e-06#ctr3#MJED';
ctr=3;
suffix2='psd0.005#osd1e-06#ctr3#MJED';
suffix3='givenStiffness0#ctr3#MJED';
suffix4='givenStiffness100#ctr3#MJED';
ylimit=12;
scen=2;

%suffix1='psd0.000#osd1e-06#ctr0#MJED-estimated130';
% ctr=0;
% suffix2='psd0.0002#osd1e-06#ctr0#MJED';
% suffix3='givenStiffness0#ctr0#MJED';
% suffix4='givenStiffness100#ctr0#MJED';
% ylimit=18;
% scen=1;

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
    leg2='No BC';
    leg3='Fixed BC';


for i=1:3
    
    figure1 = figure;
    set(figure1,'Renderer','OpenGL');
    set(figure1,'Position', [0 0 850 260]);
    %set(figure1,'OuterPosition', [0 0 850 250]);
    set(figure1,'Color',[1 1 1]);
    set(figure1,'InvertHardcopy','off');
    axes1 = axes('Parent',figure1, 'Position',[0.0980615735461802 0.323809523809524 0.868871151653363 0.471428571428571]);
    hold(axes1,'on');

    %plot(fe1(1:numStep,1), 'r-');
    plot(fe2(1:numStep,i), 'r-','LineWidth',3);
    plot(fe3(1:numStep,i), 'Color', [0 0.8 0],'LineWidth',3);
    plot(fe4(1:numStep,i), 'b-','LineWidth',3);
    
    grid on
    %title(sprintf('Feature %d', i))
    xlim([0 numStep])
    ylim([0 ylimit])    
    xlabel('Time')
    ylabel('Error [mm]')
    if i == 1
        legend1=legend(leg1, leg2, leg3);
        set(legend1,'Position'); %,[0.143272983470998 0.636170011359333 0.244583808437856 0.239208633093525],'FontSize',20);
    end
    title(sprintf('Scenario %d, feature %d', scen, i), 'FontSize', 30);
    set(axes1,'FontSize',30);
    
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


