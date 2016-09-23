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

function [] = BULKdataCORD2R(fid, CID, RID, A, B, C)
%--------------------------------------------------------------------------------------------------
% BULKdataCOORD2R.m writes fields necessary to define reference system
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%     CID       RID        CID       A1        A2        A3        B1        B2        B3
%               C1         C2        C3
% 
% 
% CID, element identification number (Integer > 0).
% 
% RID, reference system used to define A.B,C.
% 
% A, B, C grid points coordinates.
% A is the origin, B defines Z axis, C lies on XZ plane. 
% 
% Calls:        cnvt2_8chs_real.m
%--------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Define field 1: CONM2
%
fprintf(fid, 'CORD2R  ');
%--------------------------------------------------------------------------
% Define field 2: CID
%
[EIDstr] = cnvt2_8chs(CID);
fprintf(fid, '%c', EIDstr);
%--------------------------------------------------------------------------
% Define field 3: RID
%
[Gstr] = cnvt2_8chs(RID);
fprintf(fid, '%c', Gstr);
%--------------------------------------------------------------------------
% Define field 4-5-6: A
%
[CIDstr] = cnvt2_8chs_real(A(1));
fprintf(fid, '%c', CIDstr);
[CIDstr] = cnvt2_8chs_real(A(2));
fprintf(fid, '%c', CIDstr);
if (length(A)==3)
  [CIDstr] = cnvt2_8chs_real(A(3));
else
  [CIDstr] = cnvt2_8chs_real(0.0);
end
fprintf(fid, '%c', num2str8(CIDstr));
%--------------------------------------------------------------------------
% Define field 7-8-9: B
%
[CIDstr] = cnvt2_8chs_real(B(1));
fprintf(fid, '%c', CIDstr);
[CIDstr] = cnvt2_8chs_real(B(2));
fprintf(fid, '%c', CIDstr);
if (length(A)==3)
  [CIDstr] = cnvt2_8chs_real(B(3));
else
  [CIDstr] = cnvt2_8chs_real(0.0);
end
fprintf(fid, '%c', num2str8(CIDstr));
fprintf(fid, '\n        ');
%--------------------------------------------------------------------------
% Define field 2-3-4: C
%
[CIDstr] = cnvt2_8chs_real(C(1));
fprintf(fid, '%c', CIDstr);
[CIDstr] = cnvt2_8chs_real(C(2));
fprintf(fid, '%c', CIDstr);
if (length(A)==3)
  [CIDstr] = cnvt2_8chs_real(C(3));
else
  [CIDstr] = cnvt2_8chs_real(0.0);
end
fprintf(fid, '%c', num2str8(CIDstr));
%--------------------------------------------------------------------------
% New line
%
fprintf(fid, '\n');


