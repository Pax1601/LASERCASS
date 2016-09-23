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
function lattice_defo = rotate_control_surf(ref, state, geo, lattice, ROT_VALUE, HINGE, varargin)

LIN_ROT = false;
if (nargin>7)
  LIN_ROT = true;
end

nc = length(ROT_VALUE); % control surfaces
lattice_defo = lattice;
h1 = zeros(1,3);
h2 = zeros(1,3);

for n=1:nc

	if ROT_VALUE(n)
	
		h1(1, 1:3) = HINGE(n, 1, 1:3);
		h2(1, 1:3) = HINGE(n, 2, 1:3);
		patch = lattice.Control.Patch(n);
		partition = lattice.Control.Part(n);
		DOF = [lattice.DOF(patch, partition, 1) : lattice.DOF(patch, partition, 2)]; % patch dof
		lattice_defo = set_control_rot(patch, partition, ROT_VALUE(n), lattice_defo, geo, h1, h2, DOF, ...
                                   lattice.Control.DOF(n).data, LIN_ROT); 

	end
end

% update wake position
lattice_defo = defo_wakesetup(lattice_defo, state, ref);

end
%***********************************************************************************************************************
function lattice = set_control_rot(wing, division, deflection, latt, geo, a1, b1, patch_dof, control_dof, LIN_ROT)

hrot = @nlrot_rvec;
tempV1 = [];
tempV2 = []; 
if (LIN_ROT)
  hrot = @lrot_rvec;
end
lattice = latt;

[np nvor dim] = size(lattice.VORTEX);
               
switch nvor

case 8
	
	tempV1 = lattice.VORTEX(:,1,:);
	tempV2 = lattice.VORTEX(:,8,:);
	VORTEX = lattice.VORTEX(:,2:7,:);

case 6

	VORTEX = lattice.VORTEX;

otherwise

	error('Wrong vortex database.');

end
span = geo.b(wing, division);
nx = geo.nx(wing, division);
ny = geo.ny(wing, division);
fnx = geo.fnx(wing, division);
fnx2 = length(control_dof)/ny;
nx = nx+fnx-fnx2;
fnx = fnx2;
h = zeros(1,3);
hinge = zeros(1,3);

% set hinge line
h = b1 - a1;		  	
hinge = h ./ norm(h); % normalizing hingeline

patch_row = zeros(ny, nx);
control_row = zeros(ny, fnx);

for n=1:ny

	offset = (n-1) * (nx+fnx);
	patch_row(n, :)   = patch_dof([1 + offset : nx + offset]); 	
	offset = (n-1) * fnx;
	control_row(n, :) = control_dof([1 + offset : fnx + offset]); 	

end

COORD = zeros(1,3);
for n=1:ny
	
	index = control_row(n, fnx);
	COORD(1:3) = VORTEX(index, 1, 1:3);
	COORD(1:3) = hrot(hinge, COORD- a1, deflection) + a1;

	% fixed surf
	for k=1:nx
		
		VORTEX(patch_row(n, k), 1, 1:3) = COORD;
	
	end
	% control surf
	for k=1:fnx
	
		VORTEX(control_row(n, k), 1, 1:3) = COORD;
		VORTEX(control_row(n, k), 2, 1:3) = COORD;
	
	end	
	
	COORD(1:3) = VORTEX(index, 6, 1:3);
	COORD = hrot(hinge, COORD - a1, deflection) + a1;

	% fixed surf
	for k=1:nx

		VORTEX(patch_row(n,k), 6, 1:3) = COORD;

	end
	% control surf
	for k=1:fnx

		VORTEX(control_row(n, k), 6, 1:3) = COORD;
		VORTEX(control_row(n, k), 5, 1:3) = COORD;

	end	

	% rotate all the left points
	for k=1:fnx
	
		COORD(1:3) = VORTEX(control_row(n, k), 3, 1:3);
		COORD = hrot(hinge, COORD - a1, deflection) + a1;
		VORTEX(control_row(n, k), 3, 1:3) = COORD;

		COORD(1:3) = VORTEX(control_row(n, k), 4, 1:3);
		COORD = hrot(hinge, COORD - a1, deflection) + a1;
		VORTEX(control_row(n, k), 4, 1:3) = COORD;
	
	end

end

% rotate collocation points

for n = 1:ny

	for k=1:fnx
	
		COORD(1:3) = lattice.COLLOC(control_row(n, k), 1:3);
		lattice.COLLOC(control_row(n, k), 1:3) = hrot(hinge, COORD - a1, deflection) + a1;

	end

end

% rotate nodes

for n = 1:ny

	for k=1:fnx

		for m = 1:4
		
			COORD(1:3) = lattice.XYZ(control_row(n, k), m, 1:3);
			lattice.XYZ(control_row(n, k), m, 1:3) = hrot(hinge, COORD - a1, deflection) + a1;
			
		end
		
		lattice.XYZ(control_row(n, k), 5, 1:3) = lattice.XYZ(control_row(n, k), 1, 1:3);
	
	end
	
end
	 
lattice.VORTEX=[tempV1 VORTEX tempV2];

NDEFO = get_defo_normal(lattice.COLLOC(control_dof, 1:3), lattice.VORTEX(control_dof, :, 1:3), lattice.DN(control_dof, 1:3),span);

lattice.N(control_dof, 1:3) = NDEFO;	 	
		
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function prot = nlrot_rvec(hinge, p, alpha)

  prot(1:3) = expm(crossm(hinge.*alpha))*(p)';

end

function prot = lrot_rvec(hinge, p, alpha)

  prot(1:3) = (eye(3) + crossm(hinge.*alpha))*(p)';

end

end
