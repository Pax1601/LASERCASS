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

% Assembly generalized aero forces and moments on master nodes using VLM aerodynamics

function [Fa, results] = gf_aero_nodal(INFO, DOF, NODE, NODEPOS, NODER, AERO)

% generalized aero forces vector on structural master nodes
Fa = zeros(INFO.ndof, 1);
results = [];
% check 

try
	AERO.Interp.Ic;
catch
	error('Collocation points interface matrix not available.');
end

try
	AERO.Interp.In;
catch
	error('Panel nodes interface matrix not available.');
end

try
	AERO.Interp.Iv;
catch
	error('Vortex points interface matrix not available.');
end

try
	AERO.Interp.Imv;
catch
	error('Midpoint vortex points interface matrix not available.');
end

ncaero = INFO.ncaero;
ngrid = INFO.ngrid;

if ncaero

 	% update deformed mesh
	lattice_defo = update_vlm_mesh(NODE, NODEPOS, NODER, AERO);
	% set rigid body boundary conditions
	RHS = rigid_body_vlm_rhs(AERO.state, AERO.geo, lattice_defo);
    np = length(RHS);
    % get panel forces
  [dwcond, Fxa, Fya, Fza] = get_panel_forces(RHS, AERO.geo, lattice_defo, AERO.state);

	% store rigid body forces
	results.dwcond = dwcond;
	results.FORCES = zeros(1,3);
	results.MOMENTS = zeros(2,3);
	results.F = zeros(np, 3);
	results.M = zeros(np, 3);
	results.FN = zeros(np, 1);
  results.lattice_defo = lattice_defo;  
	results.F = [Fxa, Fya, Fza];
  results.FN = sum(lattice_defo.N .* results.F, 2);
	[results.FORCES, results.MOMENTS, results.M] = get_rigid_body_forces(AERO.geo, lattice_defo, results.F); 

	% get structural nodal forces
	Fxs = zeros(ngrid, 1);	     Fys = zeros(ngrid, 1);       Fzs = zeros(ngrid, 1);
  Imat = (AERO.Interp.Imv)';
	Fxs = Imat * Fxa;            Fys = Imat * Fya;            Fzs = Imat * Fza;
  
	% master node forces
	Fxm = zeros(ngrid, 1); 	     Fym = zeros(ngrid, 1); 	    Fzm = zeros(ngrid, 1);
	Mxm = zeros(ngrid, 1); 	     Mym = zeros(ngrid, 1); 	    Mzm = zeros(ngrid, 1);

	for n = 1:ngrid
		if (NODE.Index(n)) % if master node
			Fxm(n) = Fxs(n);
			Fym(n) = Fys(n);
			Fzm(n) = Fzs(n);
    end
  end      
% get slave forces
	for n = 1:ngrid
	
		if (~isempty(NODE.Aero.Coord(n).data)) % master
		
			% forces on slaves
			fxs = Fxs(NODE.Aero.Index(n).data);
			fys = Fys(NODE.Aero.Index(n).data);
			fzs = Fzs(NODE.Aero.Index(n).data);
			
			Fxm(n) = sum(fxs) + Fxm(n);
			Fym(n) = sum(fys) + Fym(n);
			Fzm(n) = sum(fzs) + Fzm(n);
			
			M = sum( cross(NODE.Aero.Coord(n).data', [fxs, fys, fzs], 2), 1);

			Mxm(n) = Mxm(n) + M(1); 
			Mym(n) = Mym(n) + M(2); 
			Mzm(n) = Mzm(n) + M(3);
		end 
	
	end % end node loop

	F = zeros(ngrid, 3);
	M = zeros(ngrid, 3);
	
	F = [Fxm, Fym, Fzm];
	M = [Mxm, Mym, Mzm];

	% store master node forces and moments in the correct DOF position

	for n = 1:ngrid
	
		if (NODE.Index(n)) % if master node
		
			index = find(DOF(n, 1:3)); % get free dofs
			pos = DOF(n, index);
			Fa(pos, 1) = F(n, index);  % assembly
			
			index = find(DOF(n, 4:6)); % get free dofs
			if ~isempty(index)
			
				indexoff = index + 3;
				pos = DOF(n, indexoff);
				Fa(pos, 1) = M(n, index);  % assembly
			
			end
		
		end
	end

else

	error('No aerodynamic mesh available for aeroelastic calculation.');

end

end
