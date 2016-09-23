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

function [] = BULKdataSPLINE1(fid, ID, IDCAERO1, FPAN, LPAN, IDSET, TOLL)
%--------------------------------------------------------------------------------------------------
% BULKdataTRIM.m defines parameters for TRIM
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%   SPLINE1     ID     IDCAERO1    FPAN      LPAN      IDSET                           
% 
% 
% Called by:    .m
% 
% Calls:        cnvt2_8chs.m
% 
%--------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------
% Define field 1: SPLINE1
%--------------------------------------------------------------------------------------------------
fprintf(fid, 'SPLINE1 ');
%--------------------------------------------------------------------------------------------------
% Define field 2: ID
%--------------------------------------------------------------------------------------------------
[IDstr] = cnvt2_8chs(ID);
fprintf(fid, '%c', IDstr);
%--------------------------------------------------------------------------------------------------
% Define field 3: IDCAERO1
%--------------------------------------------------------------------------------------------------
[IDCAERO1str] = cnvt2_8chs(IDCAERO1);
fprintf(fid, '%c', IDCAERO1str);
%--------------------------------------------------------------------------------------------------
% Define field 4: FPAN
%--------------------------------------------------------------------------------------------------
[FPANstr] = cnvt2_8chs(FPAN);
fprintf(fid, '%c', FPANstr);
%--------------------------------------------------------------------------------------------------
% Define field 5: LPAN
%--------------------------------------------------------------------------------------------------
[LPANstr] = cnvt2_8chs(LPAN);
fprintf(fid, '%c', LPANstr);
%--------------------------------------------------------------------------------------------------
% Define field 6: IDSET
%--------------------------------------------------------------------------------------------------
[IDSETstr] = cnvt2_8chs(IDSET);
fprintf(fid, '%c', IDSETstr);
%--------------------------------------------------------------------------------------------------
fprintf(fid, '                '); % skip field 7, 8
%--------------------------------------------------------------------------------------------------
% Define field 9: TOLL
%--------------------------------------------------------------------------------------------------
[TOLLstr] = cnvt2_8chs(TOLL);
fprintf(fid, '%c', TOLLstr); 
%--------------------------------------------------------------------------------------------------
fprintf(fid, '\n');
