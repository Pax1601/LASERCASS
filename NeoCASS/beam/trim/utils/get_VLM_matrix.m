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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************

function [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(geo, lattice, state, GAM)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;
PG = 1.0;
%
if ( (state.pgcorr) && (state.Mach < 1.0))
  PG = sqrt(1-state.Mach^2);
end
dwcond = 0;

[np vor_length dim] = size(lattice.VORTEX);

if vor_length ~= 8
    error('Wrong vortex struct dimension.');
end
if (nargin==3)
% assembly GAMMA_P and GAMMA_I
  [dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, 1, symmxz, symmxy, PG);
  [dwcond, GAMMA_P2, GAMMA_I] = assembly_vlm_mat(lattice, 2, symmxz, symmxy, PG);
else
% GAMMA_P available, evaluate GAMMA_I only
  [dwcond, GAMMA_P2, GAMMA_I] = assembly_vlm_mat(lattice, 2, symmxz, symmxy, PG);
  GAMMA_P= GAM;
end

end