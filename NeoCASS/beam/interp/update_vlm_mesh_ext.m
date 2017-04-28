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
%
% 
%
%***********************************************************************************************************************
% Given structural master node position and rotation, updates VLM mesh

function lattice_defo = update_vlm_mesh_ext(NODE, UPD_NODEPOS, UPD_NODER, lattice_vlm, Interp, state, ref, ID)

lattice_defo = lattice_vlm;

try
	Interp.Ic;
catch
	error('Collocation points interface matrix not available.');
end

try
	Interp.In;
catch
	error('Panel nodes interface matrix not available.');
end

try
	Interp.Iv;
catch
	error('Vortex points interface matrix not available.');
end

try
	Interp.Imv;
catch
	error('Midpoint vortex points interface matrix not available.');
end

ngrid = size(NODE.Coord, 1);
np = size(Interp.Ic, 1);

COLLOC_C = zeros(np, 3);
N = zeros(np, 3);
NODE_C = zeros(np, 5, 3);
VORTEX_C = zeros(np, 8, 3);

% update slave aerobeam nodes
AERO_POS = update_aerobeam_node(ngrid, NODE, UPD_NODEPOS, UPD_NODER);

% update coord database with slave nodes position
for n=1:ngrid

    if isfield(NODE.Aero,'Index')
	ne = length(NODE.Aero.Index(n).data);

	if ne

		UPD_NODEPOS(NODE.Aero.Index(n).data, 1:3) = AERO_POS(n).data';

    end
    end
end

[COLLOC_D, NODE_D, VORTEX_D] = deform_vlm_mesh(NODE, UPD_NODEPOS, UPD_NODER, Interp.Ic, Interp.In, Interp.Iv);

% new colloc position
COLLOC_C = lattice_vlm.COLLOC + COLLOC_D;
% new node position
NODE_C = lattice_vlm.XYZ + NODE_D;
% new vortex position
VORTEX_C = lattice_vlm.VORTEX + VORTEX_D;
% determine new flat plate normal

N = zeros(size(COLLOC_C));
for nID = 1 : length(ID)
    nPT = find(lattice_vlm.DOF(nID,:,1));
    for nn = nPT
        N(lattice_vlm.DOF(nID,nn,1):lattice_vlm.DOF(nID,nn,2),:) = get_defo_normal( ...
            COLLOC_C(lattice_vlm.DOF(nID,nn,1):lattice_vlm.DOF(nID,nn,2),:),...
            VORTEX_C(lattice_vlm.DOF(nID,nn,1):lattice_vlm.DOF(nID,nn,2),:,:),...
            NODE_C(lattice_vlm.DOF(nID,nn,1):lattice_vlm.DOF(nID,nn,2),:,:),...
            lattice_vlm.DN(lattice_vlm.DOF(nID,nn,1):lattice_vlm.DOF(nID,nn,2),:),geo.b(nID));
    end
end
% VORTEX_C = lattice_vlm.VORTEX;
% set defo lattice struct
lattice_defo.COLLOC = COLLOC_C; 
lattice_defo.VORTEX = VORTEX_C;
lattice_defo.N = N;
lattice_defo.XYZ = NODE_C;

% determine new wake shape
lattice_defo = defo_wakesetup(lattice_defo, state, ref);

end
%***********************************************************************************************************************
