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
% Assembly rotation derivatives

function [i, j, v] = set_DR_mat(nbar, BAR, BARR, COORD, NODER, DOF, FORCES, SOL)

	i = [];
	j = [];
	v = [];

	% COLLOC 1
	eta = -1/sqrt(3);
	NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
	% COLLOC 2
	eta = +1/sqrt(3);
	NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];

	NID1 =  eye(3) * NI(1);
	NID2 =  eye(3) * NI(2);
	NID3 =  eye(3) * NI(3);

	NIID1 = eye(3) * NII(1);
	NIID2 = eye(3) * NII(2);
	NIID3 = eye(3) * NII(3);

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

		NODE = [COORD(n1, :); COORD(n2, :); COORD(n3, :)];

		A = set_A_mat(COLLOC, NODE);

		FCI  = zeros(3, 3);
		MCI  = zeros(3, 3);
		FCII = zeros(3, 3);
		MCII = zeros(3, 3);

		GI = zeros(3,3);
		GII = zeros(3,3);
		
		g1 = SOL(n1,4:6)';
		g2 = SOL(n2,4:6)';
		g3 = SOL(n3,4:6)';
		
		gI = NID1 * g1 + NID2 * g2 + NID3 * g3;
		gII = NIID1 * g1 + NIID2 * g2 + NIID3 * g3;

		GI = Gmat(gI);
		GII = Gmat(gII);

		% GLOBAL FORCES
		FCI = crossm(FORCES(1, 1:3, n));
		MCI = crossm(FORCES(1, 4:6, n));

		FCII = crossm(FORCES(2, 1:3, n));
		MCII = crossm(FORCES(2, 4:6, n));

		G = zeros(12,9);
		
		HI = FCI * GI;
		HII = FCII * GII;
		
		G(1:3, 1:3) = HI * NID1; G(1:3, 4:6) = HI * NID2; G(1:3, 7:9) = HI * NID3; % FI contributions
		G(7:9, 1:3) = HII * NIID1; G(7:9, 4:6) = HII * NIID2; G(7:9, 7:9) = HII * NIID3; % FII contributions

		HI = MCI * GI;
		HII = MCII * GII;

		G(4:6, 1:3) = HI * NID1;   G(4:6, 4:6) = HI * NID2;   G(4:6, 7:9) = HI * NID3; % MI contributions
		G(10:12, 1:3) = HII * NIID1; G(10:12, 4:6) = HII * NIID2; G(10:12, 7:9) = HII * NIID3; % MII contributions

		DR = zeros(18, 9); 		
		
		DR = -A * G;

		DR = [zeros(18,3), DR(:,1:3), zeros(18,3), DR(:,4:6), zeros(18,3), DR(:,7:9)];
		
		% involved dofs
		% assembly only rotations
		rdof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)];
		cdof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)];

		rindex = find(rdof); % look for constraints
		cindex = find(cdof); % look for constraints

		rdof = rdof(rindex);
		cdof = cdof(cindex);
	
		DRff = DR(rindex, cindex);
	
		[in, jn, vel] = find(DRff);
	
		iel = double(rdof(in)');
		jel = double(cdof(jn)');

		

	%	iel = [];
	%	jel = [];
	%	vel = [];
%		nr = length(rindex);
%		nc = length(cindex);

		% sparse mat row index		
%		iel = double(repmat(rdof(rindex)', nc, 1));

%		for k = 1:nc

%			jel( ((k-1) * nr +1) : nr * k, 1) = double(cdof(cindex(k)));

%			vel = [vel ; DRff(:,k)];

%		end  


	
		i = [i; iel]; j = [j; jel]; v = [v; vel];  
	end

end
