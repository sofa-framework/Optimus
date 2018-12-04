function [ pos2d ] = proj( pos, P)

ms=size(pos,1);
N=size(pos,2)/3;
x2D=1:2:(2*N)-1;
y2D=2:2:(2*N);


x=1:3:(3*N)-2;
y=2:3:(3*N)-1;
z=3:3:3*N;


  for i =1:ms
      for j=1:N
      rx = P(1,1) * pos(i,x(j)) + P(1,2) * pos(i,y(j)) + P(1,3) * pos(i,z(j)) + P(1,4);
      ry = P(2,1) * pos(i,x(j)) + P(2,2) * pos(i,y(j)) + P(2,3) * pos(i,z(j)) + P(2,4);
      rz = P(3,1) * pos(i,x(j)) + P(3,2) * pos(i,y(j)) + P(3,3) * pos(i,z(j)) + P(3,4);
      pos2d(i,x2D(j))=rx* (1.0/rz);
      pos2d(i,y2D(j))=ry* (1.0/rz);    
                 
  end

end

