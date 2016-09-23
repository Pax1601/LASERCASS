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
% Load a NASTRAN file and create a beam struct
% Assembly rhs vector with nodal forces and moments specified by the user
function F = gf_lin_nodal(LOADS, INFO, FORCE, MOMENT, DOF)

F = zeros(INFO.ndof, 1);

Fn = zeros(3,1);
Mn = zeros(3,1);

% assembly nodal forces
for n = 1:INFO.nf

	if FORCE.ID(n) == LOADS
	
		nodei = FORCE.Node(n); % node index
		Fn = FORCE.Mag(n) .* FORCE.Orient(n,:)'; % force vector
		% assembly force in the correct position in the global array
		
		index = find(DOF(nodei, 1:3)); % get free dofs
		pos = DOF(nodei, index);
		
		F(pos, 1) = F(pos, 1) + Fn(index);  % assembly
	
	end
end

% assembly nodal moments
for n = 1:INFO.nm

	if MOMENT.ID(n) == LOADS

		nodei = MOMENT.Node(n); % node index
		Mn = MOMENT.Mag(n) .* MOMENT.Orient(n,:)'; % force vector
		% assembly force in the correct position in the global array
		index = find(DOF(nodei, 4:6)); % get free dofs
		if ~isempty(index)

			indexoff = index +3;
			pos = DOF(nodei, indexoff);

			F(pos, 1) = F(pos, 1) + Mn(index);  % assembly
		
		end
	end
end
