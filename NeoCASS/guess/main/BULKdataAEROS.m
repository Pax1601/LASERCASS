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

function [] = BULKdataAEROS(fid, CP, REFC, REFB, REFS, SYMXZ, SYMXY)
%--------------------------------------------------------------------------------------------------
% BULKdataAEROS.m defines basic parameters for static aeroelasticity
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%    AEROS       -        CP        REFC      REFB      REFS     SYMXZ     SYMXY
% 
% 
% RCSID, reference coordinate system identification for rigid body motions
% (integer >= 0); default is the basic coordinate system.
% 
% REFC, reference chord length (real > 0).
% 
% REFB, reference span (real > 0); should be full span, even on half-span
% model.
% 
% REFS, reference wing area (real > 0); should be half area on half-span
% model.
% 
% SYMXZ, symmetry key for the aero coordinate x-z plane; +1 for symmetry, 0
% for no symmetry, -1 for antisymmetry (default = 0).
% 
% SYMXY, symmetry key for the aero coordinate x-y plane can be used to
% simulate ground effects; +1 for antisymmetry, 0 for no symmetry, -1 for
% symmetry
% 
% 
% Called by:    exportAEROS.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------



%--------------------------------------------------------------------------------------------------
% Define field 1: AEROS
%--------------------------------------------------------------------------------------------------

fprintf(fid, 'AEROS   ');


%--------------------------------------------------------------------------------------------------
% Define field 2: empty field
%--------------------------------------------------------------------------------------------------

fprintf(fid, '        ');


%--------------------------------------------------------------------------------------------------
% Define field 3: CP
%--------------------------------------------------------------------------------------------------

[CPstr] = cnvt2_8chs(CP);
fprintf(fid, '%c', CPstr);


%--------------------------------------------------------------------------------------------------
% Define field 4: REFC
%--------------------------------------------------------------------------------------------------

[REFCstr] = cnvt2_8chs(REFC);
fprintf(fid, '%c', REFCstr);


%--------------------------------------------------------------------------------------------------
% Define field 5: REFB
%--------------------------------------------------------------------------------------------------

[REFBstr] = cnvt2_8chs(REFB);
fprintf(fid, '%c', REFBstr);


%--------------------------------------------------------------------------------------------------
% Define field 6: REFS
%--------------------------------------------------------------------------------------------------

[REFSstr] = cnvt2_8chs(REFS);
fprintf(fid, '%c', REFSstr);


%--------------------------------------------------------------------------------------------------
% Define field 7: SYMXZ
%--------------------------------------------------------------------------------------------------

[SYMXZstr] = cnvt2_8chs(SYMXZ);
fprintf(fid, '%c', SYMXZstr);


%--------------------------------------------------------------------------------------------------
% Define field 8: SYMXY
%--------------------------------------------------------------------------------------------------

[SYMXYstr] = cnvt2_8chs(SYMXY);
fprintf(fid, '%c', SYMXYstr);


%--------------------------------------------------------------------------------------------------
% New line
%--------------------------------------------------------------------------------------------------

fprintf(fid, '\n');


