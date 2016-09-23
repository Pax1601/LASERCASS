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
% create interpolation matrix using MLS scheme for VLM mesh
function I = assembly_node_interp_mat(ncaero, NODE, AERO)

I = [];
ri = [];
ci = [];
vi = [];
loc_offset = 0;

ngrid = size(NODE.Coord, 1); % total nodes
tot_np = size(AERO.lattice.XYZ, 1);
ntot = 0;
offset = 0;
for n=1:ncaero
	np = 1 + AERO.lattice.DOF(n, AERO.geo.nelem(n), 2) - AERO.lattice.DOF(n, 1, 1); % total stored panels on patch
	SI = AERO.IS(n).data; % interpolation set index
  if (sum(SI)>0)
    SET_AV = unique(SI);  % find interpolation set used

	  for i = 1:length(SET_AV) % loop on set

	    ninterp = SET_AV(i);
		  nset = AERO.Interp.Set(ninterp);
      PANEL = find(SI == ninterp);
      NP = length(PANEL);		
		  str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords

		  aero_data = zeros(NP*4,3);
      loc_offset = 0;
      offset_arr = zeros(NP*4,1);

      for nl = 1: NP 
			   aero_data(loc_offset+1:loc_offset+4, 1) = AERO.lattice.XYZ(PANEL(nl)+offset, 1:4, 1);
			   aero_data(loc_offset+1:loc_offset+4, 2) = AERO.lattice.XYZ(PANEL(nl)+offset, 1:4, 2);
			   aero_data(loc_offset+1:loc_offset+4, 3) = AERO.lattice.XYZ(PANEL(nl)+offset, 1:4, 3);
        offset_arr(loc_offset+1:loc_offset+4, 1) = [(PANEL(nl)-1)*4+1:PANEL(nl)*4]; 
        loc_offset = loc_offset + 4;
      end
		  % params
		  poly   = AERO.Interp.Param(ninterp, 1);
		  weight = AERO.Interp.Param(ninterp, 2);
		  points = AERO.Interp.Param(ninterp, 3); 
		  rmax   = AERO.Interp.Param(ninterp, 4);
		  tcond  = AERO.Interp.Param(ninterp, 5);

		  switch AERO.Interp.Type(ninterp)

		  case 3

			  In = mls_interface(str_data, aero_data, poly, weight, points, rmax, tcond);

		  case 2

			  In = rbf_interface(str_data, aero_data, weight, rmax, tcond);

		  end

		  [r, c, v] = find(In);
      ri = [ri; offset_arr(r) + ntot]; 
      ci = [ci; AERO.Set.Node(nset).data(c)'];
      vi = [vi; v];

	  end % set loop	
  end
  ntot = ntot + np*4;
  offset = offset + np;

end	% thread loop	
		
I = sparse(ri, ci, vi, tot_np * 4, ngrid);

end
