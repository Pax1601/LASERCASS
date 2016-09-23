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

function UPDR = update_bar_rot(nbar, DELTAR, BAR_CONN, BARR, SOL)

	UPDR = zeros(3, 3, 5, nbar);

	% COLLOC 1
	eta = -1/sqrt(3);
	NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
	NID1 = NI(1) .* eye(3); NID2 = NI(2) .* eye(3); NID3 = NI(3) .* eye(3);

	% COLLOC 2
	eta = +1/sqrt(3);
	NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
	NIID1 = NII(1) .* eye(3); NIID2 = NII(2) .* eye(3); NIID3 = NII(3) .* eye(3);

	for n=1:nbar

		n1 = BAR_CONN(n, 1);
		n2 = BAR_CONN(n, 2);
		n3 = BAR_CONN(n, 3);
		
		% update nodal sol
		
		UPDR(:,:, 1, n) = DELTAR(:,:,n1) *  BARR(:,:, 1, n);
		UPDR(:,:, 2, n) = DELTAR(:,:,n2) *  BARR(:,:, 2, n);
		UPDR(:,:, 3, n) = DELTAR(:,:,n3) *  BARR(:,:, 3, n);
		
		% update colloc sol
		% interpolate Gibbs-Rodriguez parameters
		gI  = NID1  * SOL(n1, 4:6)' + NID2  * SOL(n2, 4:6)' + NID3  * SOL(n3, 4:6)';
		gII = NIID1 * SOL(n1, 4:6)' + NIID2 * SOL(n2, 4:6)' + NIID3 * SOL(n3, 4:6)';
		
		UPDR(:,:, 4, n) = Rmat(gI)  *  BARR(:,:, 4, n);
		UPDR(:,:, 5, n) = Rmat(gII) *  BARR(:,:, 5, n);

	end

end
