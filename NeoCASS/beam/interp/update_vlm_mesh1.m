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
% Given structural master node position and rotation, updates VLM mesh

function lattice_defo = update_vlm_mesh1(NODE, DISPL, AERO) 
%
lattice_defo = AERO.lattice_vlm;
%
ngrid = size(NODE.Coord, 1);
np = length(lattice_defo.COLLOC);
COLLOC_C = zeros(np, 3);
N = zeros(np, 3);
NODE_C = zeros(np, 5, 3);
VORTEX_C = zeros(np, 8, 3);
%
[COLLOC_D, NODE_D, VORTEX_D] = deform_vlm_mesh1(NODE, DISPL, AERO.Interp.Ic, AERO.Interp.In, AERO.Interp.Iv);
% new colloc position
COLLOC_C = AERO.lattice_vlm.COLLOC + COLLOC_D;
% new node position
NODE_C = AERO.lattice_vlm.XYZ + NODE_D;
% new vortex position
VORTEX_C = AERO.lattice_vlm.VORTEX + VORTEX_D;
% determine new flat plate normal
N = zeros(size(COLLOC_C));
for nID = 1 : length(AERO.ID)
  nPT = find(AERO.lattice_vlm.DOF(nID,:,1));
  for nn = nPT
    N(AERO.lattice_vlm.DOF(nID,nn,1):AERO.lattice_vlm.DOF(nID,nn,2),:) = get_defo_normal( ...
      COLLOC_C(AERO.lattice_vlm.DOF(nID,nn,1):AERO.lattice_vlm.DOF(nID,nn,2),:),...
      VORTEX_C(AERO.lattice_vlm.DOF(nID,nn,1):AERO.lattice_vlm.DOF(nID,nn,2),:,:),...
      NODE_C(AERO.lattice_vlm.DOF(nID,nn,1):AERO.lattice_vlm.DOF(nID,nn,2),:,:),...
      AERO.lattice_vlm.DN(AERO.lattice_vlm.DOF(nID,nn,1):AERO.lattice_vlm.DOF(nID,nn,2),:),AERO.geo.b(nID));
  end
end
% set defo lattice struct
lattice_defo.COLLOC = COLLOC_C; 
lattice_defo.VORTEX = VORTEX_C;
lattice_defo.N = N;
lattice_defo.XYZ = NODE_C;
% determine new wake shape
lattice_defo = defo_wakesetup(lattice_defo, AERO.state, AERO.ref);
end
%***********************************************************************************************************************
