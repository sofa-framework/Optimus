%integ = 'Euler1';
%integ = 'Newton3';
integ = 'VarSym3';

filter='ROUKF';
%filter='UKFSimCorr';


da1=['brickD_FP1_OPogrid4_INT' integ '_TR1/DA_' filter '/DAobject'];
da2=['brickD_FP1_OPogrid4_INT' integ '_TR1/observations/object'];

comp = 0:300;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ix = 1;
for fi = comp 
    f1=['../assimBC/' da1 '_' int2str(fi) '.vtk'];
    %fprintf('Reading file %s\n', f1);
    [v1, t1, tr1] = readVTK(f1);
    
    f1=['../assimBC/' da2 '_' int2str(fi) '.vtk'];
    %fprintf('Reading file %s\n', f1);
    [v2, t2, tr2] = readVTK(f1);
    
    posDiff = abs(v1 - v2);
    
    maxDf(ix) = max(max(posDiff));
    meanDf(ix) = mean(mean(posDiff));            
    
    ix = ix + 1;
end


figure; 
axes('XLim', [1,length(comp)]);
hold on
plot(1:n, meanDf, 1:n, maxDf); title(['Error ' filter ' ' integ]);




