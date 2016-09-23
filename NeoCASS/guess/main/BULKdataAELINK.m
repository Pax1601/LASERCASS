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
% Defines the relationship between control surfaces deflection
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%  AELINK    ID        LABLD     LABLI     C
% 
%   ID    trim set identification number
%   LABLD dependent aerodynamic control surface
%   LABLI independent aerodynamic control surface (master)
%   C     relationship coefficient (-1: same deflection, 1: opposite direction)
% 
% Called by:    .m
% 
% Calls:        cnvt2_8chs.m
% 
% CREATED 2008-10-10
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataAELINK(fid, ID, LABLD, LABLI, C)


if isequal(LABLD, LABLI)
    error('Error in duplicating control surface name when estabilishing relationship in TRIM solution');
end
% 
% if (~isequal(C, -1) && ~isequal(C, 1))
%     error('Error in defining the relationship between control surfaces in TRIM solution');
% end


%--------------------------------------------------------------------------
% Line 1, field 1: AELINK
fprintf(fid, 'AELINK  ');

%--------------------------------------------------------------------------
% Line 1, field 2: ID
[IDstr] = cnvt2_8chs(ID);
fprintf(fid, '%c', IDstr);    

%--------------------------------------------------------------------------
% Line 1, field 3: LABLD
[LABLDstr] = cnvt2_8chs(LABLD);
fprintf(fid, '%c', LABLDstr);    

%--------------------------------------------------------------------------
% Line 1, field 4: LABLI
[LABLIstr] = cnvt2_8chs(LABLI);
fprintf(fid, '%c', LABLIstr);    

%--------------------------------------------------------------------------
% Line 1, field 5: C
if isequal( abs(C), floor(abs(C)) )
    C =[num2str(C) '.0'];
    C(length(C)+1:8)=' ';
    fprintf(fid, '%c', C);
else
    [Cstr] = cnvt2_8chs(C);
    fprintf(fid, '%c', Cstr);
end

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
