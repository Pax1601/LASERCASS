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
%--------------------------------------------------------------------------------------------------
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%   CAEROB      ID         X        Y          Z         CP        L         NE
%                FSi       Ri
% 
% 
% Called by:
% 
% Calls:        cnvt2_8chs.m
% 
%--------------------------------------------------------------------------------------------------
function [] = BULKdataCAEROB(fid, ID, ORIGIN, CP, L, NE, SET, FS, R)

ns = length(FS);
%--------------------------------------------------------------------------
% Define field 1: CAEROB
%
fprintf(fid, 'CAEROB  ');

%--------------------------------------------------------------------------
% Define field 2: ID
% 
if isempty(ID)  
    fprintf(fid, '        ');    
else    
    [IDstr] = cnvt2_8chs(ID);
    fprintf(fid, '%c', IDstr);    
end

%--------------------------------------------------------------------------
% Define field 3,4,5: ORIGIN
% 
if isempty(ORIGIN)
  ORIGIN = zeros(3,1);
end
for k=1:3
  [Ostr] = cnvt2_8chs(ORIGIN(k));
  fprintf(fid, '%c', Ostr);    
end

%--------------------------------------------------------------------------
% Define field 6: CP
% 
if isempty(CP)    
    fprintf(fid, '        ');    
else    
    [CPstr] = cnvt2_8chs(CP);
    fprintf(fid, '%c', CPstr);    
end

%--------------------------------------------------------------------------
% Define field 7: L
% 
[Lstr] = cnvt2_8chs(L);
fprintf(fid, '%c', Lstr);

%--------------------------------------------------------------------------
% Define field 8: NE
% 
[NEstr] = cnvt2_8chs(NE);
fprintf(fid, '%c', NEstr);

%--------------------------------------------------------------------------
% Define field 9: SET
% 
[SETstr] = cnvt2_8chs(SET);
fprintf(fid, '%c', SETstr);

%--------------------------------------------------------------------------
% Export section fraction and radius
nr = floor(ns/4); % n of rows
nrr = mod(ns, 4); % n of extra rows
cont = 0;
for k=1:nr
  fprintf(fid, '\n        ');
  for j=1:4
    cont = cont+1;
    [Sstr] = cnvt2_8chs(FS(cont));
    [Rstr] = cnvt2_8chs(R(cont));
    fprintf(fid, '%c', Sstr);
    fprintf(fid, '%c', Rstr);
  end
end
if (nrr>0)
  fprintf(fid, '\n        ');
  for j=1:nrr
    cont = cont+1;
    [Sstr] = cnvt2_8chs(FS(cont));
    [Rstr] = cnvt2_8chs(R(cont));
    fprintf(fid, '%c', Sstr);
    fprintf(fid, '%c', Rstr);
  end
  fprintf(fid, 'ENDT');
else
  fprintf(fid, '\n        ENDT');
end

%
fprintf(fid, '\n');
