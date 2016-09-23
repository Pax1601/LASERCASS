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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
% sort interpolation results for VLM nodes and vortecies
function [COLLOC_D, NODE_D, VORTEX_D] = deform_vlm_mesh(NODE, UPD_NODEPOS, UPD_NODER, Ic, In, Iv)

np = size(Ic, 1);

COLLOC_D = zeros(np, 3);
NODE_D = zeros(np, 5, 3);
VORTEX_D = zeros(np, 8, 3);

% node displacements
NODED = UPD_NODEPOS - NODE.Coord;

% collocation points coordinates
COLLOC_D(:,1) = Ic * NODED(:,1);
COLLOC_D(:,2) = Ic * NODED(:,2);
COLLOC_D(:,3) = Ic * NODED(:,3);

% panel nodes coordinates
ndispl = zeros(np * 4, 3);
ndispl(:,1) = In * NODED(:,1);
ndispl(:,2) = In * NODED(:,2);
ndispl(:,3) = In * NODED(:,3);

NODE_D = compact_interp_data(0, 4, 1, ndispl(:,1), NODE_D);
NODE_D = compact_interp_data(0, 4, 2, ndispl(:,2), NODE_D);
NODE_D = compact_interp_data(0, 4, 3, ndispl(:,3), NODE_D);

% panel vortices coordinates
vdispl = zeros(np * 6, 3);
vdispl(:,1) = Iv * NODED(:,1);
vdispl(:,2) = Iv * NODED(:,2);
vdispl(:,3) = Iv * NODED(:,3);

VORTEX_D = compact_interp_data(1, 6, 1, vdispl(:,1), VORTEX_D);
VORTEX_D = compact_interp_data(1, 6, 2, vdispl(:,2), VORTEX_D);
VORTEX_D = compact_interp_data(1, 6, 3, vdispl(:,3), VORTEX_D);

end
