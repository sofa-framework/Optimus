close all
inDir='/home/ip/Work/sofa/MyPlugins/Optimus/verdandiSofa/estimateBoundary/estimationPython';

expFile='exp1.out';
varFile='exp1_vars.out';

%chosen=[1 500 1000 1500];
chosen=[1:50:1000];

exps=load(sprintf('%s/%s', inDir, expFile));
ivars=load(sprintf('%s/%s', inDir, varFile));

nest=size(exps,1);
npar=size(exps,2);

figure;
plot(exps);

vars=zeros(nest,npar,npar);

for ie=1:nest
    gli=0;
    for i=1:npar
        vars(ie,i,i) = ivars(ie,i);        
        for j=1:i-1
            gli=gli+1;
            vars(ie,i,j) = ivars(ie,npar+gli);
            vars(ie,j,i) = ivars(ie,npar+gli);
        end
    end
end



for i=1:length(chosen)
    figure;
    ch=chosen(i);    
    vv=squeeze(vars(ch,:,:));
    surf(sqrt(abs(vv)));
    view(0,90);
    colorbar
end

