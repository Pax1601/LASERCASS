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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080224      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function export_st_aerosol_param(fp, cref, bref, sref, simmxz, simmxy)
%
%   DESCRIPTION: Appends AEROS card to SMARTCAD file
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    SMARTCAD file pointer
%                cref           real       Reference chord
%                bref           real       Reference span
%                sref           real       Reference surface
%                simmxz         integer    Symm./Antis. along vertical plane
%                simmxy         integer    Symm./Antis. along horizontal plane
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function export_st_aerosol_param(fp, cref, bref, sref, simmxz, simmxy, height)
%
FIELD = 8;
%
str = num2str(cref, '%7g'); cref_str = [str, blanks(FIELD-length(str))];
str = num2str(bref, '%7g'); bref_str = [str, blanks(FIELD-length(str))];
str = num2str(sref, '%7g'); sref_str = [str, blanks(FIELD-length(str))];
str = num2str(simmxz, '%d'); simmxz_str = [str, blanks(FIELD-length(str))];
str = num2str(simmxy, '%d'); simmxy_str = [str, blanks(FIELD-length(str))];
str = num2str(simmxy, '%7g'); height_str = [str, blanks(FIELD-length(str))];

%
line = ['\nAEROS   ', blanks(2*FIELD), cref_str, bref_str, sref_str, simmxz_str, simmxy_str, height_str]; 
%
fprintf(fp, '\n$ Vortex lattice solver parameters');
fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fp, line);
