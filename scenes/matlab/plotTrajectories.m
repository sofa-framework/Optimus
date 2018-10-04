ec=load('../assimStiffness/cylinder2_results/end_compress.txt');
ei=load('../assimStiffness/cylinder2_results/end_incompress.txt');

mc=load('../assimStiffness/cylinder2_results/mid_compress.txt');
mi=load('../assimStiffness/cylinder2_results/mid_incompress.txt');

mcr=load('../assimStiffness/cylinder2_results/mid_incompress_rigid.txt');
mce=load('../assimStiffness/cylinder2_results/mid_incompress_equal.txt');
mcs=load('../assimStiffness/cylinder2_results/mid_incompress_soft.txt');


mc42=load('../assimStiffness/cylinder2_results/end_incompress_4200.txt');
mc32=load('../assimStiffness/cylinder2_results/end_incompress_3230.txt');
mc36=load('../assimStiffness/cylinder2_results/end_incompress_3620.txt');
mc50=load('../assimStiffness/cylinder2_results/end_incompress_5000.txt');
mc70=load('../assimStiffness/cylinder2_results/end_incompress_7000.txt');

ns=250;
nss=1:250;

figure;

%========================================================================
%plot(nss, ec(nss,4)-0.15, 'r-', nss, ei(nss,4)-0.15, 'r--', nss, mc(nss,4), ... 
%     'b-', nss, mi(nss,4), 'b--', nss, mcr(nss,4), 'm--');
% legend('end compress', 'end incompress', 'mid compress', 'mid incompress', 'mid incompress rigid');
% title('Mid/end point trajectories, compressible/incompressible');

%========================================================================
% plot(nss, mc(nss,4), 'b-', nss, mi(nss,4), 'b--', nss, mcr(nss,4), 'm--', nss, mce(nss,4), 'c--', ...
%     nss, mcs(nss,4), 'k--');
% legend('mid compress 3000 7000', 'mid incompress 3000 7000', 'mid incompress rigid 3000 10000', 'mid incompress equal 3000 3000', 'mid incompress soft 3000 2000');
% title('Midpoint trajectory in direct simulation, different parameters')
%========================================================================

plot(nss, mc30(nss,4), 'r-', nss, mc42(nss,4), 'g-', nss, mc50(nss,4), 'b-', nss, mc70(nss,4), 'c-', nss, mc36(nss,4), 'k-', nss, mc32(nss,4), 'b--');
legend('end incompress 3000 7000', 'end incompress 4200 4200','end incompress 5000 3620', 'end incompress 7000 3000', 'end incompress 3620 5000', 'end incompress 3230 6000')
title('End point trajectory in direct simulation, different E1, E2, constant Eeff')
