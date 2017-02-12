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

function [D_DISPL, dwn, N_DISPL, DN] = set_patch_bc(pind, cref, V, midpoint, colloc, aero_node, normal, str_data, mode, k_list, AERO, ...
                                                SCALE, PLOT_RES)
	% get total number of active modes
	NMODES = size(mode, 3);
	if (SCALE)
		colloc    = colloc .* cref;
		midpoint  = midpoint .* cref;
		aero_node = aero_node .* cref;
	end
  [Id, Ic, In] = set_patch_interf_matrix(pind, midpoint, colloc, aero_node, str_data, AERO);
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
	for (m = 1: NMODES)
		xdef  = Id * mode(:, 1, m);
		ydef  = Id * mode(:, 2, m);
		zdef  = Id * mode(:, 3, m);
		D_DISPL(:,:,m) = [xdef, ydef, zdef];
		xdef  = Ic * mode(:, 1, m);
		ydef  = Ic * mode(:, 2, m);
		zdef  = Ic * mode(:, 3, m);
		C_DISPL(:,:,m) = [xdef, ydef, zdef];
		xdef  = In * mode(:, 1, m);
		ydef  = In * mode(:, 2, m);
		zdef  = In * mode(:, 3, m);
		N_DISPL(:,:,m) = [xdef, ydef, zdef];
		if (PLOT_RES)
			offset = 99;
			figure(m + offset); hold on; grid;
			% structural displacements
			%plot3(str_data(:,1), str_data(:,2), str_data(:,3),'.k');
			plot3(str_data(:,1) + mode(:,1,m), str_data(:,2) + mode(:,2,m), str_data(:,3) + mode(:,3,m),'.k');
	    % midpoint displacements
			% plot3(midpoint(:,1),midpoint(:,2),midpoint(:,3),'r.');
			plot3(midpoint(:,1) + D_DISPL(:,1,m),midpoint(:,2) + D_DISPL(:,2,m),midpoint(:,3) + D_DISPL(:,3,m),'r.');
			% collocation point displacements
			% plot3(colloc(:,1),colloc(:,2),colloc(:,3),'y.');
			plot3(colloc(:,1) + C_DISPL(:,1,m),colloc(:,2) + C_DISPL(:,2,m),colloc(:,3) + C_DISPL(:,3,m),'yo');
		end
%   get COLLOCATION POINT normal displacements
    CNDISPL = dot(C_DISPL(:,:,m), normal, 2);
%   get DOUBLET POINT normal displacements
    DNDISPL = dot(D_DISPL(:,:,m), normal, 2);
%   get normal variation
		DN = (CNDISPL - DNDISPL) ./ DIST;
		for (k = 1:nk) 
			dwn(:, m, k) = i .* CNDISPL .* (k_list(k) / cref) + DN;
		end
	end
end
