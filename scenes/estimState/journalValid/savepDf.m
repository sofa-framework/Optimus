h=openfig('bif3DHD_OB.fig','invisible');
set(h,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos = get(h,'Position');
print(h,'bif3DHD_OB','-dpdf','-r0')

h1=openfig('bif3DHD_TOP.fig','invisible');
set(h1,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos2 = get(h1,'Position');
print(h1,'bif3DHD_TOP','-dpdf','-r0')

h2=openfig('bif3DHD_SIDE.fig','invisible');
set(h2,'Position',[50 50 1800 1000]);
set(gcf,'papersize',[20 10])
pos3 = get(h2,'Position');
print(h2,'bif3DHD_SIDE','-dpdf','-r0')
