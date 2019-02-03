clear all
%upper face:
%x=0.01:0.02:0.15;
%y=0.06;
%z=0.01:0.03:0.07;

%front face:
x=0.18;
y=0.01:0.02:0.06;
z=0.01:0.03:0.07;


for ix=1:length(x)
    for iy=1:length(y)
        for iz=1:length(z)
            fprintf('%.2f %.2f %.2f  ', x(ix), y(iy), z(iz));
        end
    end
end
fprintf('\n');
