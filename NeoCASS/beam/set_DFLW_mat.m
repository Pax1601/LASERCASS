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
% Assembly follower force derivatives

function [i, j, v] = set_DFLW_mat(LOADS, INFO, FORCE, NODER, DOF, SOL, SCALE)

i = [];
j = [];
v = [];

Fn = zeros(3,1);
Fn0 = zeros(3,1);
Mn = zeros(3,1);
g = zeros(3,1);
G = zeros(3,3);
JF = zeros(3,3);
JM = zeros(3,3);

% assembly nodal forces
for n = 1:INFO.nflw

	if FORCE.ID(n) == LOADS
	
		nodei = FORCE.Node(n); % node index
		
		g = SOL(nodei, 4:6);
		G = Gmat(g);
		Fn0 = (SCALE * FORCE.Mag(n)) .* FORCE.Orient(n,:)'; % force vector
		Fn = NODER(:,:, nodei) * Fn0; % force vector
		Mn = NODER(:,:, nodei) * (crossm(FORCE.Offset(n,:)) * Fn0);
		% jacobian matrix
		JF = crossm(Fn) * G;
		JM = crossm(Mn) * G;
		
		% store data in sparse format
		index_r = find(DOF(nodei, 1:3)); % get free dofs
		index_c = find(DOF(nodei, 4:6)); % get free dofs
		
		if ~isempty(index_c) % check if g are constrained
		
			row_dof  = DOF(nodei, index_r); % row dof
			colm_dof = DOF(nodei, index_c+3); % colm dof
			
			if ~isempty(index_r) % assembly forces jacobian
			
				jf = JF(index_r, index_c);
				
				[in, jn, vin] = find(jf);
				
				i = [i; double(row_dof(in)')];
				j = [j; double(colm_dof(jn)')];
				v = [v; vin];
			
			end
			% assembly moments jacobian
		
			jm = JM(index_c, index_c);

			[in, jn, vin] = find(jm);

			i = [i; double(colm_dof(in)')];
			j = [j; double(colm_dof(jn)')];
			v = [v; vin];
		
		end
		
		
	end
end

end
