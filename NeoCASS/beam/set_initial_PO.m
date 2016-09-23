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

% Determine initial tangent vector in bar material reference frame
function PO = set_initial_PO(nbar, Bar, Node)

	PO = zeros(2, 3, nbar);

	% COLLOC 1
	eta = -1/sqrt(3);
	NIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC1
	% COLLOC2
	eta = +1/sqrt(3);
	NIIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC2 

	for n=1:nbar

		n1 = Bar.Conn(n, 1);
		n2 = Bar.Conn(n, 2);
		n3 = Bar.Conn(n, 3);
		% offset global coords
		f1 = Node.R(:,:, n1) * Bar.Offset(n, 1:3)';
		f2 = Node.R(:,:, n2) * Bar.Offset(n, 4:6)';
		f3 = Node.R(:,:, n3) * Bar.Offset(n, 7:9)';
		% node global coords
		c1 = f1 + Node.Coord(n1,:)';
		c2 = f2 + Node.Coord(n2,:)';
		c3 = f3 + Node.Coord(n3,:)';

		N = zeros(12,18);
		JI = zeros(1,3);
		JII = zeros(1,3);
		PpI = zeros(3,1);
		PpII = zeros(3,1);

		x = [c1(1) c2(1) c3(1)];
		y = [c1(2) c2(2) c3(2)];
		z = [c1(3) c2(3) c3(3)];

		% COLLOC 1
		JI(1) = dot(NIp, x);
		JI(2) = dot(NIp, y);
		JI(3) = dot(NIp, z);
		NIp = NIp ./ norm(JI);

		% COLLOC 2
		JII(1) = dot(NIIp, x);
		JII(2) = dot(NIIp, y);
		JII(3) = dot(NIIp, z);
		NIIp = NIIp ./ norm(JII);

		PO(1, 1, n) = dot(NIp, x);
		PO(1, 2, n) = dot(NIp, y);
		PO(1, 3, n) = dot(NIp, z);

		PO(2, 1, n) = dot(NIIp, x);
		PO(2, 2, n) = dot(NIIp, y);
		PO(2, 3, n) = dot(NIIp, z);

		% determine local initial PO
		
		PO(1, 1:3, n) = ((Bar.R(:,:,4,n))' * PO(1, 1:3, n)')';
		PO(2, 1:3, n) = ((Bar.R(:,:,5,n))' * PO(2, 1:3, n)')';
		
	end

end
