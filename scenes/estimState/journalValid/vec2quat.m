function [ quat ] = vec2quat( vector )
a0 = vector(1);
a1 = vector(2);
a2 = vector(3);
        quat(4) = cos(a0/2)*cos(a1/2)*cos(a2/2) + sin(a0/2)*sin(a1/2)*sin(a2/2);
        quat(1) = sin(a0/2)*cos(a1/2)*cos(a2/2) - cos(a0/2)*sin(a1/2)*sin(a2/2);
        quat(2) = cos(a0/2)*sin(a1/2)*cos(a2/2) + sin(a0/2)*cos(a1/2)*sin(a2/2);
        quat(3) = cos(a0/2)*cos(a1/2)*sin(a2/2) - sin(a0/2)*sin(a1/2)*cos(a2/2);


end

