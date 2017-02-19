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
%   Author: Luca Cavagna
%***********************************************************************************************************************

function [ddispl, dwn, ndispl] = set_boundary_condition1(fid, geo, lattice, dlm, NODE, mode, AERO, SCALE, PLOT_RES)

  fprintf(fid, '\n - Assemblying boundary displacements and downwash...');
  offset = 100;
  scale = 500;
  colloc = lattice.COLLOC;
  midpoint = lattice.MID_DPOINT;
  aero_node = lattice.XYZ;
  cref = dlm.aero.cref;
  V = dlm.aero.V;
  normal = lattice.N;
  k_list = dlm.aero.k;
%
  NMODES = size(mode, 3);
  np = size(colloc, 1);
  nnp = 4*np;
  V_dir = repmat(V, np, 1);
  nk = length(k_list);
%
  ddispl = zeros(np, 3, NMODES);
  cdispl = zeros(np, 3, NMODES);
  dwn = zeros(np, NMODES, nk);
  ndispl = zeros(nnp, 3, NMODES);
%
  if (SCALE)
    colloc    = colloc .* cref;
    midpoint  = midpoint .* cref;
    aero_node = aero_node .* cref;
  end
% interface 
  Ic = assembly_colloc_interp_mat1(geo.nwing, NODE, AERO, colloc);
  In = assembly_node_interp_mat1(geo.nwing, NODE, AERO, aero_node);
  Im = assembly_colloc_interp_mat1(geo.nwing, NODE, AERO, midpoint);
%
  DIST = colloc - midpoint;
	DIST = dot(DIST, V_dir, 2); 
%
  for (m = 1: NMODES)
    MODEDOF = ndispl2dof(NODE.DOF, mode(:,:,m));
    data = Ic * MODEDOF;  
    cdispl(:,1,m) = data(1:np);
    cdispl(:,2,m) = data(np+1:2*np);
    cdispl(:,3,m) = data(2*np+1:3*np);
    data = In * MODEDOF;  
    ndispl(:,1,m) = data(1:nnp);
    ndispl(:,2,m) = data(nnp+1:2*nnp);
    ndispl(:,3,m) = data(2*nnp+1:end);
    data = Im * MODEDOF;  
    ddispl(:,1,m) = data(1:np);
    ddispl(:,2,m) = data(np+1:2*np);
    ddispl(:,3,m) = data(2*np+1:3*np);
    if (PLOT_RES)
			figure(offset); close; figure(offset); hold on; grid;
      plot3(NODE.Coord(:,1) , NODE.Coord(:,2), NODE.Coord(:,3), '.c');
			plot3(NODE.Coord(:,1) + scale.*mode(:,1,m),  NODE.Coord(:,2) + scale.*mode(:,2,m),  NODE.Coord(:,3) + scale.*mode(:,3,m),'.k');
	    plot3(midpoint(:,1)   + scale.*ddispl(:,1,m),midpoint(:,2)   + scale.*ddispl(:,2,m),midpoint(:,3)   + scale.*ddispl(:,3,m),'r.');
			plot3(colloc(:,1)     + scale.*cdispl(:,1,m),colloc(:,2)     + scale.*cdispl(:,2,m),colloc(:,3)     + scale.*cdispl(:,3,m),'yo');
      axis equal; 
      offset = offset + 1;
		end
%   get COLLOCATION POINT normal displacements
    CNDISPL = dot(cdispl(:,:,m), normal, 2);
%   get DOUBLET POINT normal displacements
    DNDISPL = dot(ddispl(:,:,m), normal, 2);
%   get normal variation
		DN = (CNDISPL - DNDISPL) ./ DIST;
		for (k = 1:nk) 
			dwn(:, m, k) = 1i .* CNDISPL .* (k_list(k) / cref) + DN;
		end
%
	end % mode loop
%  

  ddispl = ddispl(1:lattice.np,:,:);
  dwn = dwn(1:lattice.np,:,:);
%  ndispl = ndispl(1:lattice.np,:,:);

  fprintf(fid, '\n   done.');
end

