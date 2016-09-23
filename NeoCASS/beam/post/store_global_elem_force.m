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

function BARF = store_global_elem_force(nbar, BARR, FORCES)

	BARF = allocate_barf(nbar);

	for nelem = 1:nbar

		BARF(1,1:3,nelem) = (BARR(:,:,4, nelem) * FORCES(1,1:3, nelem)')'; 
		BARF(2,1:3,nelem) = (BARR(:,:,5, nelem) * FORCES(2,1:3, nelem)')'; 

		BARF(1,4:6,nelem) = (BARR(:,:,4, nelem) * FORCES(1,4:6, nelem)')'; 
		BARF(2,4:6,nelem) = (BARR(:,:,5, nelem) * FORCES(2,4:6, nelem)')'; 

	end

end
