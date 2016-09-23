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
% create interpolation matrix using MLS/RBF scheme for VLM mesh (collocation points)
function I = assembly_colloc_interp_mat(ncaero, NODE, AERO)

I = [];
ri = [];
ci = [];
vi = [];

ngrid = size(NODE.Coord, 1); % total nodes
tot_np = size(AERO.lattice.COLLOC, 1);
ntot = 0;
for n=1:ncaero
	np = 1 + AERO.lattice.DOF(n, AERO.geo.nelem(n), 2) - AERO.lattice.DOF(n, 1, 1); % total stored panels on patch
	SI = AERO.IS(n).data; % interpolation set index
  if (sum(SI)>0)
    SET_AV = unique(SI);  % find interpolation set used
  	for i = 1:length(SET_AV) % loop on set

	    ninterp = SET_AV(i);
		  nset = AERO.Interp.Set(ninterp);
      PANEL = find(SI == ninterp);

		  str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords

		  % params
		  poly   = AERO.Interp.Param(ninterp, 1);
		  weight = AERO.Interp.Param(ninterp, 2);
		  points = AERO.Interp.Param(ninterp, 3); 
		  rmax   = AERO.Interp.Param(ninterp, 4);
		  tcond  = AERO.Interp.Param(ninterp, 5);

		  switch AERO.Interp.Type(ninterp)

		    case 3

			    Ic = mls_interface(str_data, AERO.lattice.COLLOC(PANEL+ntot, 1:3), poly, weight, points, rmax, tcond);

		    case 2

			    Ic = rbf_interface(str_data, AERO.lattice.COLLOC(PANEL+ntot, 1:3), weight, rmax, tcond);

		  end

		  [r, c, v] = find(Ic);

      ri = [ri; (PANEL(r)+ntot)];
      ci = [ci; AERO.Set.Node(nset).data(c)'];
      vi = [vi; v];

	  end % set loop	
  end
  ntot = ntot + np;

end	% thread loop	
		
I = sparse(ri, ci, vi, tot_np, ngrid);

end
