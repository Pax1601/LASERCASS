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
% Assembly tangent matrix

function [i, j, v] = set_DT_mat_nl(nbar, BAR, BARR, DOF, COORD0, COORD, NODER0, NODER, SOL)

i = [];
j = [];
v = [];

	for n=1:nbar

		n1 = BAR.Conn(n, 1);
		n2 = BAR.Conn(n, 2);
		n3 = BAR.Conn(n, 3);
		% involved dofs
		dof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)];
		index = find(dof); % look for constraints
		% global offset	
		f1 = NODER0(:,:,n1) * BAR.Offset(n, 1:3)';
		f2 = NODER0(:,:,n2) * BAR.Offset(n, 4:6)';
		f3 = NODER0(:,:,n3) * BAR.Offset(n, 7:9)';
		% global nodes
		c01 = f1 + COORD0(n1, :)'; 
		c02 = f2 + COORD0(n2, :)'; 
		c03 = f3 + COORD0(n3, :)';

		% global offset	
		f1 = NODER(:,:,n1) * BAR.Offset(n, 1:3)';
		f2 = NODER(:,:,n2) * BAR.Offset(n, 4:6)';
		f3 = NODER(:,:,n3) * BAR.Offset(n, 7:9)';
		% global nodes
		c1 = f1 + COORD(n1, :)'; 
		c2 = f2 + COORD(n2, :)'; 
		c3 = f3 + COORD(n3, :)';

		COLLOC = interp_colloc_pos(c1, c2, c3);

		NODE = [COORD(n1, :); COORD(n2, :); COORD(n3, :)];

		A = set_A_mat(COLLOC, NODE);

		D = st_D_global(BAR.D(:,:,1,n), BARR(:,:,4,n), BAR.D(:,:,2,n), BARR(:,:,5,n));
		N = set_N_mat(c01, c02, c03, c1, c2, c3, f1, f2, f3);

		JI = zeros(1,3);
		JII = zeros(1,3);
		PpI = zeros(3,1);
		PpII = zeros(3,1);

		x = [c01(1) c02(1) c03(1)];
		y = [c01(2) c02(2) c03(2)];
		z = [c01(3) c02(3) c03(3)];

		% COLLOC 1
		eta = -1/sqrt(3);
		NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
		NIp =2.* [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC1
		JI(1) = dot(NIp, x);
		JI(2) = dot(NIp, y);
		JI(3) = dot(NIp, z);
		NIp = NIp ./ norm(JI);
		% COLLOC 2
		eta = +1/sqrt(3);
		NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
		NIIp = 2.*[(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC2 
		JII(1) = dot(NIIp, x);
		JII(2) = dot(NIIp, y);
		JII(3) = dot(NIIp, z);
		NIIp = NIIp ./ norm(JII);

		x = [c1(1) c2(1) c3(1)];
		y = [c1(2) c2(2) c3(2)];
		z = [c1(3) c2(3) c3(3)];

		PpI(1) = dot(NIp, x);
		PpI(2) = dot(NIp, y);
		PpI(3) = dot(NIp, z);
		PpII(1) = dot(NIIp, x);
		PpII(2) = dot(NIIp, y);
		PpII(3) = dot(NIIp, z);

		NID1 = NI(1) .* eye(3); NID2 = NI(2) .* eye(3); NID3 = NI(3) .* eye(3); 
		NIpD1 = NIp(1) .* eye(3); NIpD2 = NIp(2) .* eye(3); NIpD3 = NIp(3) .* eye(3); 
		NIID1 = NII(1) .* eye(3); NIID2 = NII(2) .* eye(3); NIID3 = NII(3) .* eye(3);
		NIIpD1 = NIIp(1) .* eye(3); NIIpD2 = NIIp(2) .* eye(3); NIIpD3 = NIIp(3) .* eye(3); 

		r1 = SOL(n1,4:6)';
		r2 = SOL(n2,4:6)';
		r3 = SOL(n3,4:6)';

		gI = NID1 * r1 + NID2 * r2 + NID3 * r3;
		gII = NIID1 * r1 + NIID2 * r2 + NIID3 * r3;

		G1 = Gmat(gI);
		G2 = Gmat(gII);

		gIp = NIpD1 * r1 + NIpD2 * r2 + NIpD3 * r3;
		gIIp = NIIpD1 * r1 + NIIpD2 * r2 + NIIpD3 * r3;

		N(4:6, 4:6) = G1 * N(4:6, 4:6) + ( crossm(G1*gIp) * G1 * NID1 + DGmat(gI, gIp) * NID1);
		N(4:6, 10:12) =G1 *  N(4:6, 10:12) + ( crossm(G1*gIp) * G1 * NID2 + DGmat(gI, gIp) * NID2 );
		N(4:6, 16:18) =G1 *  N(4:6, 16:18) + ( crossm(G1*gIp) * G1 * NID3 + DGmat(gI, gIp) * NID3 );

		N(10:12, 4:6) =G2 *  N(10:12, 4:6) + ( crossm(G2*gIIp) * G2 * NID1 + DGmat(gII, gIIp) * NIID1);
		N(10:12, 10:12) =G2 * N(10:12, 10:12) + ( crossm(G2*gIIp) * G2 * NID2 + DGmat(gII, gIIp) * NIID2 );
		N(10:12, 16:18) =G2 * N(10:12, 16:18) + ( crossm(G2*gIIp) * G2 * NID3 + DGmat(gII, gIIp) * NIID3 );

		Kel = (A * D) * N;

		% delete constrained dofs
		Kff = Kel(index, index);
		nr = length(index);
		iel = [];
		jel = [];
		vel = [];
		% sparse mat row index		
		iel = double(repmat(dof(index)', nr, 1));
		% sparse mat col index and value
		for k = 1:nr

			jel( ((k-1) * nr +1) : nr * k, 1) = double(dof(index(k))');

			vel = [vel ; Kff(:,k)];

		end
		
		i = [i; iel]; j = [j; jel]; v = [v; vel];  

	end

end
