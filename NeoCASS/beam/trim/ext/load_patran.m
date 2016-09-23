%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
% Load PATRAN data
%
function [C, F] = load_patran(filename, extra, symm, Frame)
%
fid = 1;
FIELD = 8;
% define allowed Nastran cards
keyword = {
    'INCLUDE'
    'GRID'
    'CTRIA3'
    'CQUAD4'
    'PLOAD4'
    'PLOAD4*'
};
%
PARAM.INCLUDE = {};
PARAM.INCLUDE = filename;
ninclude = length(filename);
%
ngrid = 0;
nelem = 0;
npload = 0;
% NODE struct
NODE  = [];
NODE.ID = [];
NODE.CS = [];
NODE.Coord = [];
NODE.CD = [];
% ELEM struct
ELEM = [];
ELEM.ID = [];
ELEM.Conn = [];
ELEM.Type = [];
% PLOAD struct
PLOAD = [];
PLOAD.Elem = [];
PLOAD.Value = [];
%
READ_INCLUDE = true;
NFILE = 0;
%
while (READ_INCLUDE)

    NFILE = NFILE + 1;
    fp = fopen(PARAM.INCLUDE{NFILE}, 'r');
    skip_line = false;
    fprintf(fid,'\nReading %s file...', PARAM.INCLUDE{NFILE});

    while ~feof(fp)
        if ~skip_line
            tline = fgetl(fp);
        else
            skip_line = false;
        end

        CARD = strtok(tline);

        switch CARD

            case keyword{1} % INCLUDE card for secondary files

                ninclude = ninclude +1;
                name_assembly = tline(9:end);
                PARAM.INCLUDE{ninclude} = name_assembly;

            case keyword{2} % GRID card

                ngrid = ngrid +1;

                NODE.ID(ngrid) = int32(num_field_parser(tline, 2));
                NODE.CS(ngrid) = int32(num_field_parser(tline, 3));
                NODE.Coord(ngrid, 1) = num_field_parser(tline, 4);
                NODE.Coord(ngrid, 2) = num_field_parser(tline, 5);
                NODE.Coord(ngrid, 3) = num_field_parser(tline, 6);
                NODE.CD(ngrid) = int32(num_field_parser(tline, 7));
                skip_line = false;

            case keyword{3} % CTRIA3 card

                nelem = nelem +1;

                ELEM.ID(nelem) = int32(num_field_parser(tline, 2));
%                   CTRIA3.PID(nelem) = int32(num_field_parser(tline, 3));
                c1 = int32(num_field_parser(tline, 4));
                c2 = int32(num_field_parser(tline, 5));
                c3 = int32(num_field_parser(tline, 6));
                ELEM.Conn(nelem,:) = [c1,c2,c3,c1];
                ELEM.Type(nelem) = 3;
                skip_line = false;

            case keyword{4} % CQUAD4 card

                nelem = nelem +1;

                ELEM.ID(nelem) = int32(num_field_parser(tline, 2));
                c1 = int32(num_field_parser(tline, 4));
                c2 = int32(num_field_parser(tline, 5));
                c3 = int32(num_field_parser(tline, 6));
                c4 = int32(num_field_parser(tline, 7));
                ELEM.Conn(nelem,:) = [c1,c2,c3,c4];
                ELEM.Type(nelem) = 4;
                skip_line = false;

            case keyword{5} % PLOAD card

                npload = npload +1;

                PLOAD.Elem(npload) = int32(num_field_parser(tline, 3));
                PLOAD.Value(npload) = num_field_parser(tline, 4);
                skip_line = false;

            case keyword{6} % PLOAD* card

                npload = npload +1;
                data = sscanf(tline(9:end),'%d%d%f');
                PLOAD.Elem(npload) = int32(data(2));
                PLOAD.Value(npload) = data(3);
                skip_line = false;

        end % end of switch
%
    end % end of while
    fclose(fp);
%
    if (length(PARAM.INCLUDE) == NFILE)
        READ_INCLUDE = false;
    else
        if ~exist(PARAM.INCLUDE{NFILE+1}, 'file')
            error(['Unable to find file %s.', PARAM.INCLUDE{NFILE+1},'.']);
        end
    end
    %
end % INCLUDE
%
fprintf(fid, '\n\tSorting Node database...');
[NODE.ID, index] = sort(NODE.ID);
[labels, i] = unique(NODE.ID);
% check for duplicated labels
if (length(labels) ~= ngrid)
    n = [1 : ngrid];
    dof = NODE.ID(setdiff(n, i));
    for k=1:length(dof)
        fprintf(fid, '\n\t### Warning: duplicated labels for grid: %d.', NODE.ID(dof(k)));
    end
    error('Grid entries have duplicated labels.');
end
NODE.CS = NODE.CS(index);
NODE.Coord = NODE.Coord(index,1:3);
NODE.CD = NODE.CD(index);
fprintf(fid, 'done.');
%
fprintf(fid, '\n\tSorting Elem database...');
[ELEM.ID, index] = sort(ELEM.ID);
[labels, i] = unique(ELEM.ID);
% check for duplicated labels
if (length(labels) ~= nelem)
    n = [1 : nelem];
    dof = ELEM.ID(setdiff(n, i));
    for k=1:length(dof)
        fprintf(fid, '\n\t### Warning: duplicated labels for element: %d.', ELEM.ID(dof(k)));
    end
    error('Element entries have duplicated labels.');
end
ELEM.Conn = ELEM.Conn(index,:);
ELEM.Type = ELEM.Type(index);
%
fprintf(fid, 'done.');
for i=1:nelem
  for k=1:4
    n = find(NODE.ID == ELEM.Conn(i,k));
    if isempty(n)
        error('Unable to determine node %d position in element %d.', ELEM.Conn(i,k), ELEM.ID(i));
    end
    ELEM.Conn(i,k) = n;
  end
end
%
if npload
  [PLOAD.Elem, index] = sort(PLOAD.Elem);
  [labels, i] = unique(PLOAD.Elem);
  % check for duplicated labels
  if (length(labels) ~= npload)
      n = [1 : npload];
      dof = PLOAD.Elem(setdiff(n, i));
      for k=1:length(dof)
          fprintf(fid, '\n\t### Warning: duplicated PLOAD card for element: %d.', PLOAD.Elem(dof(k)));
      end
      error('PLOAD cards refer to multiple elements.');
  end
  PLOAD.Value = PLOAD.Value(index);
end
%
% centroids
%
C = zeros(nelem,3);
A = zeros(nelem,3);
F = zeros(nelem,3);
for i=1:nelem
  n1 =  ELEM.Conn(i,1);
  n2 =  ELEM.Conn(i,2);
  n3 =  ELEM.Conn(i,3);
  n4 =  ELEM.Conn(i,4);
%
  switch ELEM.Type(i)
    case {3}
      C(i,:) = mean([NODE.Coord(n1,:); NODE.Coord(n2,:); NODE.Coord(n3,:)]);
    case {4}
      C(i,:) = mean([NODE.Coord(n1,:); NODE.Coord(n2,:); NODE.Coord(n3,:); NODE.Coord(n4,:)]);
  end
%
end
if npload
  for i=1:nelem
    n1 =  ELEM.Conn(i,1);
    n2 =  ELEM.Conn(i,2);
    n3 =  ELEM.Conn(i,3);
    n4 =  ELEM.Conn(i,4);

    d1 = NODE.Coord(n4,:) - NODE.Coord(n2,:);
    d2 = NODE.Coord(n3,:) - NODE.Coord(n1,:);
    A(i,:) = 0.5.*cross(d1, d2);
    n = find(PLOAD.Elem == ELEM.ID(i)); 
    if ~isempty(n)
      F(i,:) = A(i,:) .* PLOAD.Value(n) .* extra.QREF;
    end
  end
end
%
%
if ~isempty(Frame)
  if Frame.Coord
    for i=1:nelem
      C(i,:) = (Frame.Rmat * C(i,:)')' + Frame.ORIGIN;
      F(i,:) = Frame.Rmat * F(i,:)';
    end
    for i=1:ngrid
      NODE.Coord(i,:) = (Frame.Rmat * NODE.Coord(i,:)')' + Frame.ORIGIN;
    end
  end
end
%
p = patch('Faces',ELEM.Conn,'Vertices',NODE.Coord,'FaceColor','none','EdgeColor','r');
axis equal;
%
if (symm)
  F(:,2) = -F(:,2);
  C(:,2) = -C(:,2);
end
%
fprintf(fid, '\nForces resultants along aero mesh: ');
fprintf(fid, '\n\tFx: %g.', sum(F(:,1)));
fprintf(fid, '\n\tFy: %g.', sum(F(:,2)));
fprintf(fid, '\n\tFz: %g.', sum(F(:,3)));
fprintf(fid, '\n');
%
F = reshape(F', nelem*3,1);
%
end

function num = num_field_parser(line, index)

FIELD = 8;

if length(line) < FIELD * (index-1)
    
    num = 0;
    
else
    
    minc = min(length(line), index * FIELD);
    field = strtok(line((index-1) * FIELD+1:minc));
    mindex = find(field == '-');
    if length(mindex)>1
      field = [field(1:mindex(end)-1),'E',field(mindex(end):end)];
    end
    if ~isempty(field)
        
        num = str2num(field);
        
    else
        
        num = 0;
        
    end
    
end

end
