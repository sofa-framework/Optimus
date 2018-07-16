h=openfig('hd2d_BIF_OB.fig','visible');
set(h,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos = get(h,'Position');
print(h,'hd2d_BIF_OB','-dpdf','-r0')

h1=openfig('hd2d_TEST_TOP.fig','invisible');
set(h1,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos2 = get(h1,'Position');
print(h1,'hd2d_TEST_TOP','-dpdf','-r0')

h2=openfig('RMSE2d_TEST_SIDE.fig','invisible');
set(h2,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos3 = get(h2,'Position');
print(h2,'RMSE2d_TEST_SIDE','-dpdf','-r0')
