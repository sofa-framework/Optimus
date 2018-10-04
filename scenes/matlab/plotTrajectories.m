ec=load('../assimStiffness/cylinder2_results/end_compress.txt');
ei=load('../assimStiffness/cylinder2_results/end_incompress.txt');

mc=load('../assimStiffness/cylinder2_results/mid_compress.txt');
mi=load('../assimStiffness/cylinder2_results/mid_incompress.txt');

mcr=load('../assimStiffness/cylinder2_results/mid_incompress_rigid.txt');
mce=load('../assimStiffness/cylinder2_results/mid_incompress_equal.txt');
mcs=load('../assimStiffness/cylinder2_results/mid_incompress_soft.txt');


ns=250;
nss=1:250;

figure;

%========================================================================
%plot(nss, ec(nss,4)-0.15, 'r-', nss, ei(nss,4)-0.15, 'r--', nss, mc(nss,4), ... 
%     'b-', nss, mi(nss,4), 'b--', nss, mcr(nss,4), 'm--');
% legend('end compress', 'end incompress', 'mid compress', 'mid incompress', 'mid incompress rigid');
% title('Mid/end point trajectories, compressible/incompressible');

%========================================================================
plot(nss, mc(nss,4), 'b-', nss, mi(nss,4), 'b--', nss, mcr(nss,4), 'm--', nss, mce(nss,4), 'c--', ...
    nss, mcs(nss,4), 'k--');
legend('mid compress 3000 7000', 'mid incompress 3000 7000', 'mid incompress rigid 3000 10000', 'mid incompress equal 3000 3000', 'mid incompress soft 3000 2000');
title('Midpoint trajectory in direct simulation, different parameters')
%========================================================================