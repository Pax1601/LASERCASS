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

function [] = BULKdataCONM2(fid, EID, G  , CID, M  , X1 , X2 , X3, I11, I21, I22, I31, I32, I33)
%--------------------------------------------------------------------------------------------------
% BULKdataCONM2.m writes fields necessary to define concentrated mass
% element connection, rigid body form
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%     EID        G        CID        M        X1        X2        X3
%     I11       I21       I22       I31       I32       I33
% 
% 
% EID, element identification number (Integer > 0).
% 
% G, grid point identification number (Integer > 0).
% 
% CID, coordinate system identification number (Integer >= -1; Default = 0).
% 
% M, mass value (Real).
% 
% X1,X2,X3, offset distances from the grid point to the center of gravity
% of the mass in the coordinate system defined in field 4 (Real).
% 
% Iij, mass moment of inertia measured at the mass center of gravity in the
% coordinate system defined by field 4 (for I11,I22 and I33: Real >=0; for
% I21,I31 and I32: Real)
% 
% 
% Called by:    writeCONM22file.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------



%--------------------------------------------------------------------------
% Define field 1: CONM2
%
fprintf(fid, 'CONM2   ');

%--------------------------------------------------------------------------
% Define field 2: EID
%
[EIDstr] = cnvt2_8chs(EID);
fprintf(fid, '%c', EIDstr);

%--------------------------------------------------------------------------
% Define field 3: G
%
[Gstr] = cnvt2_8chs(G);
fprintf(fid, '%c', Gstr);

%--------------------------------------------------------------------------
% Define field 4: CID
%
[CIDstr] = cnvt2_8chs(CID);
fprintf(fid, '%c', CIDstr);

%--------------------------------------------------------------------------
% Define field 5: M
%
[Mstr] = cnvt2_8chs(M);
fprintf(fid, '%c', Mstr);

%--------------------------------------------------------------------------
% Define field 6: X1
%
[X1str] = cnvt2_8chs(X1);
fprintf(fid, '%c', X1str);

%--------------------------------------------------------------------------
% Define field 7: X2
%
[X2str] = cnvt2_8chs(X2);
fprintf(fid, '%c', X2str);

%--------------------------------------------------------------------------
% Define field 8: X3
%
[X3str] = cnvt2_8chs(X3);
fprintf(fid, '%c', X3str);

%--------------------------------------------------------------------------
% Second line of CONM2 card
%
fprintf(fid, '\n');

%--------------------------------------------------------------------------
% Define field 1, new line: empty
%
fprintf(fid, '        ');

%--------------------------------------------------------------------------
% Define field 2, new line: I11
%
[I11str] = cnvt2_8chs(I11);
fprintf(fid, '%c', num2str8(I11str));

%--------------------------------------------------------------------------
% Define field 3, new line: I21
%
[I21str] = cnvt2_8chs(I21);
fprintf(fid, '%c', num2str8(I21str));

%--------------------------------------------------------------------------
% Define field 4, new line: I22
%
[I22str] = cnvt2_8chs(I22);
fprintf(fid, '%c', num2str8(I22str));

%--------------------------------------------------------------------------
% Define field 5, new line: I31
%
[I31str] = cnvt2_8chs(I31);
fprintf(fid, '%c', num2str8(I31str));

%--------------------------------------------------------------------------
% Define field 6, new line: I32
%
[I32str] = cnvt2_8chs(I32);
fprintf(fid, '%c', num2str8(I32str));

%--------------------------------------------------------------------------
% Define field 7, new line: I33
%
[I33str] = cnvt2_8chs(I33);
fprintf(fid, '%c', num2str8(I33str));

%--------------------------------------------------------------------------
% New line
%
fprintf(fid, '\n');


