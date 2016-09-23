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
% 2008-04-06
% 
% BULKdataGRID.m writes fields necessary to define geometric grid point
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%    GRID       ID        CP        X1        X2        X3        CD        PS       SEID
% 
% 
% Called by:    writeGRID2file.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataGRID(fid, ID, CP, X1, X2, X3, CD, PS, SEID)

%--------------------------------------------------------------------------
% Define field 1: GRID
%
fprintf(fid, 'GRID    ');

%--------------------------------------------------------------------------
% Define field 2: ID
%
if (ID > 0 & ID < 100000000)    
    [IDstr] = cnvt2_8chs(ID);
    fprintf(fid, '%c', IDstr);    
else    
    fprintf('\n\tWARNING: GRID POINT IDENTIFICATION NUMBER MUST BE INTEGER AND 0 < ID < 100000000.\n');    
end

%--------------------------------------------------------------------------
% Define field 3: CP
%
[CPstr] = cnvt2_8chs(CP);
fprintf(fid, '%c', CPstr);

%--------------------------------------------------------------------------
% Define field 4: X1
%
[X1str] = cnvt2_8chs(X1);
fprintf(fid, '%c', num2str8(X1str));

%--------------------------------------------------------------------------
% Define field 5: X2
%
[X2str] = cnvt2_8chs(X2);
fprintf(fid, '%c', num2str8(X2str));

%--------------------------------------------------------------------------
% Define field 6: X3
%
[X3str] = cnvt2_8chs(X3);
fprintf(fid, '%c', num2str8(X3str));

%--------------------------------------------------------------------------
% Define field 7: CD
%
[CDstr] = cnvt2_8chs(CD);
fprintf(fid, '%c', CDstr);

%--------------------------------------------------------------------------
% Define field 8: PS
%
[PSstr] = cnvt2_8chs(PS);
fprintf(fid, '%c', PSstr);

%--------------------------------------------------------------------------
% Define field 9: SEID
%
[SEIDstr] = cnvt2_8chs(SEID);
fprintf(fid, '%c', SEIDstr);

%--------------------------------------------------------------------------
% New line
%
fprintf(fid, '\n');


