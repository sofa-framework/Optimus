ec=load('../assimStiffness/cylinder2_results/end_compress.txt');
ei=load('../assimStiffness/cylinder2_results/end_incompress.txt');

mc=load('../assimStiffness/cylinder2_results/mid_compress.txt');
mi=load('../assimStiffness/cylinder2_results/mid_incompress.txt');

mir=load('../assimStiffness/cylinder2_results/mid_incompress_rigid.txt');
mie=load('../assimStiffness/cylinder2_results/mid_incompress_equal.txt');
mis=load('../assimStiffness/cylinder2_results/mid_incompress_soft.txt');


ei42=load('../assimStiffness/cylinder2_results/end_incompress_4200.txt');
ei32=load('../assimStiffness/cylinder2_results/end_incompress_3230.txt');
ei36=load('../assimStiffness/cylinder2_results/end_incompress_3620.txt');
ei50=load('../assimStiffness/cylinder2_results/end_incompress_5000.txt');
ei70=load('../assimStiffness/cylinder2_results/end_incompress_7000.txt');

ns=250;
nss=1:250;

figure('InvertHardcopy','off','Color',[1 1 1],'Position', [0 0 1000 800]);

%========================================================================
% plot(nss, ec(nss,4)-0.15, 'r-', nss, ei(nss,4)-0.15, 'r--', nss, mc(nss,4), ... 
%     'b-', nss, mi(nss,4), 'b--', nss, mcr(nss,4), 'm--');
% legend({'end compress', 'end incompress', 'mid compress', 'mid incompress', 'mid incompress rigid'}, ... 
%         'FontSize', 12, 'Location', 'northwest');
% title('Mid/end point trajectories, compressible/incompressible');
% 
% fileName = 'traject_midEnd_comp'
% saveas(gcf, fileName, 'png')

%========================================================================
% plot(nss, mc(nss,4), 'b-', nss, mi(nss,4), 'b--', nss, mir(nss,4), 'm--', nss, mie(nss,4), 'c--', ...
%     nss, mis(nss,4), 'k--');
% legend({'mid compress 3000 7000', 'mid incompress 3000 7000', 'mid incompress rigid 3000 10000', 'mid incompress equal 3000 3000', 'mid incompress soft 3000 2000'}, ... 
%         'FontSize', 12, 'Location', 'northwest');
% title('Midpoint trajectory in direct simulation, different parameters')
% fileName = 'traject_midpoint_comp'
% saveas(gcf, fileName, 'png')

%========================================================================

plot(nss, ei(nss,4), 'r-', nss, ei42(nss,4), 'g-', nss, ei50(nss,4), 'b-', nss, ei70(nss,4), 'y-', nss, ei36(nss,4), 'k-', nss, ei32(nss,4), 'b--');
legend({'end incompress 3000 7000', 'end incompress 4200 4200','end incompress 5000 3620', 'end incompress 7000 3000', 'end incompress 3620 5000', 'end incompress 3230 6000'}, ... 
       'FontSize', 12, 'Location', 'northwest');
title('End point trajectory in direct simulation, different E1, E2, constant Eeff')
fileName = 'traject_endpoint_comp'
saveas(gcf, fileName, 'png')

