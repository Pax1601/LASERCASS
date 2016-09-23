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

% get element forces in global reference frame
function F = get_actual_nodal_forces(ngrid, nbar, BAR, BARR, DOF, COORD, NODER, FORCES)

	F = zeros(ngrid, 6);

	for n=1:nbar

		n1 = BAR.Conn(n, 1);
		n2 = BAR.Conn(n, 2);
		n3 = BAR.Conn(n, 3);
		% global offset	
		f1 = NODER(:,:,n1) * BAR.Offset(n, 1:3)';
		f2 = NODER(:,:,n2) * BAR.Offset(n, 4:6)';
		f3 = NODER(:,:,n3) * BAR.Offset(n, 7:9)';
		% global nodes
		c1 = f1 + COORD(n1, :)'; 
		c2 = f2 + COORD(n2, :)'; 
		c3 = f3 + COORD(n3, :)';

		COLLOC = interp_colloc_pos(c1, c2, c3);

		NODE = [c1, c2, c3]';

		A = set_A_mat(COLLOC, NODE);
		R = set_R_mat(BARR(:,:,4,n) , BARR(:,:,5,n));
	
		Fel = A * R * [FORCES(1,:,n) , FORCES(2,:,n)]';
		
		F(n1, 1:6) = F(n1, 1:6) + Fel(1:6)';
		F(n2, 1:6) = F(n2, 1:6) + Fel(7:12)';
		F(n3, 1:6) = F(n3, 1:6) + Fel(13:18)';
	
	end

end
