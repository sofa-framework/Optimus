clear 
close all
clc

da_12=dlmread('E_1');

figure
plot(da_12(1:6000,end))
hold on
% plot(da_15(1:2000,end))
% hold on
% plot(da_16(1:2000,end))
% hold on
t=[0 5999]
plot (t,[10000 10000], 'r')
xlim([0 5999])
legend('Estimated Value','Real Value =1e4')
% da_100=dlmread('da_100');
% da_1000=dlmread('da_1000');
% 
% gt=dlmread('SSFFObs3D_hooke_x.txt');
% t=gt(:,1);
% obs=gt(:,2:end);
% h10=zeros(2300,1);
% h100=zeros(2300,1);
% h1000=zeros(2300,1);
% 
% 
% for i=1:size(h10,1)
%    h10(i)=mean(cat(2,norm(da_10(i,1:3)'-obs(i,1:3)'),norm(da_10(i,4:6)'-obs(i,4:6)'),norm(da_10(i,7:9)'-obs(i,7:9)')));
%    h100(i)=mean(cat(2,norm(da_100(i,1:3)'-obs(i,1:3)'),norm(da_100(i,4:6)'-obs(i,4:6)'),norm(da_100(i,7:9)'-obs(i,7:9)')));
%    h1000(i)=mean(cat(2,norm(da_1000(i,1:3)'-obs(i,1:3)'),norm(da_1000(i,4:6)'-obs(i,4:6)'),norm(da_1000(i,7:9)'-obs(i,7:9)')));
% 
% end
% 
% 
% figure
% t=0:2299;
% subplot(2,1,1)
% plot(t,h10,'r',t,h100,'k',t,h1000,'b')
% legend('10 Obs','100 Obs', '1000 Obs')
% xlim([0 2300])
% title('Distance with Ground Truth')
% subplot(2,1,2)
% plot(t,da_10(1:2300,19),'r',t,da_100(1:2300,19),'k',t,da_1000(1:2300,19),'b')
% hold on
% plot([0 2299], [0.05 0.05] ,'g', 'Linewidth', 3)
% xlim([0 2300])
% legend('10 Obs','100 Obs', '1000 Obs', 'Real Mass=0.05')
% title('Parameter Estimation')
% 

% 
% A=[];  B=[]; C=[];
% for i=1:9
% filename = sprintf('covDAAsyn_/cov_000%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% A=[A d];
% end
% for i=10:99
% filename = sprintf('covDAAsyn_/cov_00%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% A=[A d];
% end
% for i=100:999
% filename = sprintf('covDAAsyn_/cov_0%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% A=[A d];
% end
% for i=1000:2300
% filename = sprintf('covDAAsyn_/cov_%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% A=[A d];
% end
% 
% for i=1:9
% filename = sprintf('covSEAsyn_/cov_000%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% B=[B d];
% end
% for i=10:99
% filename = sprintf('covSEAsyn_/cov_00%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% B=[B d];
% end
% for i=100:999
% filename = sprintf('covSEAsyn_/cov_0%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% B=[B d];
% end
% for i=1000:2300
% filename = sprintf('covSEAsyn_/cov_%d.txt',i);
% c = load(filename);
% d=trace(c(1:18,1:18));
% B=[B d];
% end
% % 
% % 
% % for i=1:9
% % filename = sprintf('covSyn_/cov_000%d.txt',i);
% % c = load(filename);
% % d=trace(c);
% % C=[C d];
% % end
% % for i=10:99
% % filename = sprintf('covSyn_/cov_00%d.txt',i);
% % c = load(filename);
% % d=trace(c);
% % C=[C d];
% % end
% % for i=100:999
% % filename = sprintf('covSyn_/cov_0%d.txt',i);
% % c = load(filename);
% % d=trace(c);
% % C=[C d];
% % end
% % for i=1000:3270
% % filename = sprintf('covSyn_/cov_%d.txt',i);
% % c = load(filename);
% % d=trace(c);
% % C=[C d];
% % end
% % 
% % 
% figure
% plot(A) 
% hold on 
% plot(B,'r')
% legend('Parameter Assimilation', 'State Estimation' )
% % % hold on
% % % plot(C,'k')
% xlim([0 2300])
% % % for i=10:99
% % filename = sprintf('covSE_/cov_00%d.txt',i);
% % c = load(filename);
% % imagesc(c(:,:))
% % title({'FinalCovariance Force Estimation = '  num2str(i) });
% % set(gca,'Layer','top','XTick',...
% %    [3.500,9.500,15.50,21.50,27.50,33.50,39.50,45.50,51.50,57.50,63.50,69.50,75.50,81.50,87.50,93.50,99.50,105.5,111.5,117.5,123.5,129.5,135.5,141.5],...
% %     'XTickLabel',...
% %     {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11','p12','v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12'},'YTick',...
% %     [3.500,9.500,15.50,21.50,27.50,33.50,39.50,45.50,51.50,57.50,63.50,69.50,75.50,81.50,87.50,93.50,99.50,105.5,111.5,117.5,123.5,129.5,135.5,141.5],'YTickLabel',...
% %     {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11','p12','v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12'});
% % % xlim(gca,[-4 130]);
% % hold on
% % t=0.5:6:144.5;
% % t1=0.5:3:144.5;
% % y=0.5:6:144.5;
% % y1=t1;
% % h1=vline(t1,'b--');
% % h=vline(t,'k');
% % hold on
% % m2=hline(y,'k');
% % m=hline(y1,'b--');
% % colorbar('peer',gca);
% % caxis([0 10e-8 ])
% % pause(0.3);
% % end
% % 
% % 
% % 
% % for i=100:999
% % filename = sprintf('covSE_/cov_0%d.txt',i);
% % c = load(filename);
% % imagesc(c(:,:))
% % title({'FinalCovariance Force Estimation = '  num2str(i) });
% % set(gca,'Layer','top','XTick',...
% %    [3.500,9.500,15.50,21.50,27.50,33.50,39.50,45.50,51.50,57.50,63.50,69.50,75.50,81.50,87.50,93.50,99.50,105.5,111.5,117.5,123.5,129.5,135.5,141.5],...
% %     'XTickLabel',...
% %     {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11','p12','v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12'},'YTick',...
% %     [3.500,9.500,15.50,21.50,27.50,33.50,39.50,45.50,51.50,57.50,63.50,69.50,75.50,81.50,87.50,93.50,99.50,105.5,111.5,117.5,123.5,129.5,135.5,141.5],'YTickLabel',...
% %     {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11','p12','v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12'});
% % % xlim(gca,[-4 130]);
% % hold on
% % t=0.5:6:144.5;
% % t1=0.5:3:144.5;
% % y=0.5:6:144.5;
% % y1=t1;
% % h1=vline(t1,'b--');
% % h=vline(t,'k');
% % hold on
% % m2=hline(y,'k');
% % m=hline(y1,'b--');
% % colorbar('peer',gca);
% % caxis([0 10e-8 ])
% % pause(0.3);
% % end