for i=1:107
    K=load(sprintf('K_%03d.txt', i));
    z=load(sprintf('z_%03d.txt', i));
    
    cr = K*z';
    ll = size(cr,1);
    fprintf('%d: %f %f %f\n', i, cr(ll-2), cr(ll-1), cr(ll));
end