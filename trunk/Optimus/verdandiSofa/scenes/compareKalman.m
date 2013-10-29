clear all
%close all

%it=10; nParam=2; nColl=2; nDim=1;
%indir='/home/ipeterlik/Work/Sofa/applications-dev/plugins/Optimus/verdandiSofa/verdandi-1.5/example/clamped_bar/result'

it=10; nParam=2; nColl=1; nDim=3; nElem=363;

indir=sprintf('/home/ipeterlik/Work/Sofa/applications-dev/plugins/Optimus/verdandiSofa/scenes/result%d_%d', nParam, nElem);


%T=load(sprintf('%s/truth-forecast_state.dat',indir));
%P=load(sprintf('%s/ukf-forecast_state.dat',indir));
%A=load(sprintf('%s/ukf-analysis_state.dat',indir));
R=load(sprintf('%s/roukf-forecast_state.dat',indir));

nState=size(R,2)
nObs=size(R,1)

nDof=(nState-nParam)/nColl;
nNode=nDof/nDim;

desiredObs = nObs;

if (nParam == 2)
    figure; plot(1:1:desiredObs, R(1:1:desiredObs,nState-1), 1:1:desiredObs, R(1:1:desiredObs,nState));
    fprintf('Parameters found: %f %f\n', R(desiredObs, nState-1), R(desiredObs,nState));
    xlabel('Time step');
    ylabel('Young modulus [Pa]');
elseif (nParam == 3)
    figure; plot(1:desiredObs, R(1:desiredObs,nState-2), 1:desiredObs, R(1:desiredObs,nState-1), 1:desiredObs, R(1:desiredObs,nState));
    fprintf('Parameters found: %f %f %f\n', R(desiredObs, nState-2), R(desiredObs, nState-1), R(desiredObs,nState));
end


return

t=reshape(T(it,1:nDof), nDim, nNode)';
p=reshape(P(it,1:nDof), nDim, nNode)';
a=reshape(A(it,1:nDof), nDim, nNode)';

return

if (nDim == 3) 
    figure; 
    hold on
    daspect([1 1 1]);
    scatter3(t(:,1), t(:,2), t(:,3),'.r', 'MarkerSize',5);
    scatter3(p(:,1), p(:,2), p(:,3),'*g');
    scatter3(a(:,1), a(:,2), a(:,3),'ob');
end

if (nDim == 1)
    figure; 
    hold on
    %daspect([1 1 1]);
    plot(t, 'r+', 'MarkerSize',10);
    plot(p, 'g*');
    plot(a, 'bo');
    
end

figure; 
%plot(A(:,nState-1));
%plot(A(:,nState-1));

    





