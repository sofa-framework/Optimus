%    Copyright (C) 2017, see doc/COPYING, doc/AUTHORS  for licencing details
function [vertex,tetra,tria] = readVTK(filename, ~)

% readVTK - read data from VTK file.
%
%   [vertex,tetra, tria] = read_vtk(filename, verbose);
%
%   'vertex' is a '|nodes| x 3' array specifying the position of the vertices.
%   'tetra' is a '|tetras| x 4' array specifying the connectivity of a volume mesh.
%        note: filled only if present in the VTK file, otherwise empty array      
%   'tria' is a '|trias| x 3' array specifying the connectivity of a surface mesh
%        note: filled only if present in the VTK file, otherwise empty array
%

%if nargin<2
%    verbose = 1;
%end

fid = fopen(filename,'r');
if( fid==-1 )
    error('Can''t open the file.');    
end

str = fgets(fid);   % -1 if eof
if ~strcmp(str(3:5), 'vtk')
    error('The file is not a valid VTK one.');    
end

%%% read header %%%
str = fgets(fid);
while length(str) < 6 || ~strcmp(str(1:6), 'POINTS')
    str = fgets(fid);
end
nvert = sscanf(str,'%*s %d %*s', 1);

% read vertices
[A,cnt] = fscanf(fid,'%f %f %f', 3*nvert);
if cnt~=3*nvert
    warning('Problem in reading vertices.');
end
A = reshape(A, 3, cnt/3)';
vertex = A;

str = fgets(fid);
while str ~= -1 
    if length(str) > 4 && (strcmp(str(1:5), 'CELLS') || strcmp(str(1:8), 'POLYGONS'))      
        break;            
    end
    str = fgets(fid);
end
nelem = sscanf(str,'%*s %d %*s', 1);
nnum = sscanf(str,'%*s %*s %d', 1);

ntria=0;
ntetra=0;
tetra=[];
tria=[];
for ei=1:nelem
    str = fgets(fid);
    ln=str2num(str);
    
    if ln(1) == 3
        ntria=ntria+1;
        tria(ntria,1:3) = ln(2:4)+1;
    elseif ln(1) == 4
        ntetra=ntetra+1;
        tetra(ntetra,1:4) = ln(2:5)+1;
    end
end


fclose(fid);

return
