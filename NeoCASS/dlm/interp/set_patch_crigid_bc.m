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
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function [C_DISPL, dwn, N_DISPL] = set_patch_crigid_bc(cref, V, midpoint, colloc, aero_node, normal, REF_POINT, CINDEX, HINGE_AXIS, ...
                                                       MSCALE, k_list, SCALE, PLOT_RES)
	% get total number of active modes
	NMODES = 1; 
%
  REF_LEN = 1.0;
	if (SCALE)
    REF_LEN = cref;
  end
%
	colloc    = colloc .* REF_LEN   - repmat(REF_POINT, size(colloc,1),1);
	midpoint  = midpoint .* REF_LEN - repmat(REF_POINT, size(midpoint,1),1);
	aero_node = aero_node .* REF_LEN;
	% get midpoint displacements
	np = size(colloc, 1);
	nk = length(k_list);
	dwn = zeros(np, NMODES, nk);

	% set velocity cosines (usually 1,0,0)
	V_dir = repmat(V, np, 1);
 	% get distance vector
	DIST = colloc - midpoint;
	DIST = dot(DIST, V_dir, 2); 

	if (PLOT_RES)
		offset = 100;
		for (m = offset:(offset + NMODES))
			figure(m); close;
		end
	end
	
  D_DISPL = zeros(size(midpoint,1),3,NMODES);
  NP = size(colloc,1);
  C_DISPL = zeros(NP,3,NMODES);
  N_DISPL = zeros(NP*4,3,NMODES);

	aero_data = zeros(NP*4,3);
  loc_offset = 0;
%
  R = crossm(HINGE_AXIS.*MSCALE);
%
  for nl = 1: NP 
		 aero_data(loc_offset+1:loc_offset+4, 1) = aero_node(nl, 1:4, 1) - REF_POINT(1);
		 aero_data(loc_offset+1:loc_offset+4, 2) = aero_node(nl, 1:4, 2) - REF_POINT(2);
		 aero_data(loc_offset+1:loc_offset+4, 3) = aero_node(nl, 1:4, 3) - REF_POINT(3);
     loc_offset = loc_offset + 4;
  end
%
%-------------------------------------------------------------------------------
%   Node displacements
  NCP = length(CINDEX);
  for nl = 1: NCP
    offset = (CINDEX(nl)-1) * 4;
    for ncp = 1:4 
      N_DISPL(offset+ncp,1:3) = (R * aero_data(offset+ncp,:)')';
    end 
  end
%
  for nl = 1: NCP 
%   Collocation displacements
    C_DISPL(CINDEX(nl),1:3) = (R * colloc(CINDEX(nl),:)')';
%   Doublet displacements
    D_DISPL(CINDEX(nl),1:3) = (R * midpoint(CINDEX(nl),:)')';
  end
%
	if (PLOT_RES)
		offset = 99;
		figure(1 + offset); close; figure(1 + offset);hold on; grid;
	  % midpoint displacements
		plot3(midpoint(:,1),midpoint(:,2),midpoint(:,3),'ks');
		plot3(midpoint(:,1) + D_DISPL(:,1),midpoint(:,2) + D_DISPL(:,2),midpoint(:,3) + D_DISPL(:,3),'k.');
		% collocation point displacements
		plot3(colloc(:,1),colloc(:,2),colloc(:,3),'rs');
		plot3(colloc(:,1) + C_DISPL(:,1),colloc(:,2) + C_DISPL(:,2),colloc(:,3) + C_DISPL(:,3),'r.');
    axis equal;
	end
% get COLLOCATION POINT normal displacements
  CNDISPL = dot(C_DISPL(CINDEX,1:3), normal(CINDEX,1:3), 2);
% get DOUBLET POINT normal displacements
  DNDISPL = dot(D_DISPL(CINDEX,1:3), normal(CINDEX,1:3), 2);
% get normal variation
	DN = (CNDISPL - DNDISPL) ./ DIST(CINDEX);
%   Assembly downwash		
	for (k = 1:nk)
		dwn(CINDEX, 1, k) = i .* CNDISPL .* (k_list(k) / cref) + DN;
	end
%	
end
