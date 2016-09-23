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
% BAR struct
% DOF struct
% Actual node angular pos
% Actual node coord

% Assembly one term of the jacobian related to derivatives of the arm matrix 
function [i, j, v] = set_DA_mat(nbar, BAR, BARR, DOF, NODER, FORCES, SOL)

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

	DA1 = zeros(3,18);
	DA2 = zeros(3,18);
	DA3 = zeros(3,18);
	DA = zeros(9, 18);

	n1 = BAR.Conn(n, 1);
	n2 = BAR.Conn(n, 2);
	n3 = BAR.Conn(n, 3);
	% global offset	
	f1 = NODER(:,:,n1) * BAR.Offset(n, 1:3)';
	f2 = NODER(:,:,n2) * BAR.Offset(n, 4:6)';
	f3 = NODER(:,:,n3) * BAR.Offset(n, 7:9)';

	R = set_R_mat(BARR(:,:,4,n) , BARR(:,:,5,n));
	FORCESG = zeros(12,1);
	FORCESG = R * [FORCES(1,1:6,n) , FORCES(2,1:6,n)]';

	% internal forces cross product
	F1C = crossm(FORCESG(1:3));
	F2C = crossm(FORCESG(7:9));
	% offset cross product
	f1c = crossm(f1);
	f2c = crossm(f2);
	f3c = crossm(f3);
	
	r1 = SOL(n1,4:6)';
	r2 = SOL(n2,4:6)';
	r3 = SOL(n3,4:6)';
	G1 = Gmat(r1);
	G2 = Gmat(r2);
	G3 = Gmat(r3);
	
	% node 1		
	DA1(1:3,1:3) =    -F1C * (NID1 - eye(3));
	DA1(1:3,4:6) =     F1C * NID1 * f1c * G1;
	DA1(1:3,7:9) =    -F1C * NID2;
	DA1(1:3,10:12) =   F1C * NID2 * f2c * G2;
	DA1(1:3,13:15) =  -F1C * NID3;
	DA1(1:3,16:18) =   F1C * NID3 * f3c * G3;
	% node 3
	DA3(1:3,1:3) =     F2C * NIID1;
	DA3(1:3,4:6) =    -F2C * NIID1 * f1c * G1;
	DA3(1:3,7:9) =     F2C * NIID2;
	DA3(1:3,10:12) =  -F2C * NIID2 * f2c * G2;
	DA3(1:3,13:15) =   F2C * (NIID3 - eye(3));
	DA3(1:3,16:18) =  -F2C * NIID3 * f3c * G3;
	% node 2 first contrib
	%DA2(1:3,1:3) =     F1C * NID1;
	%DA2(1:3,4:6) =    -F1C * NID1 * crossm(f1);
	%DA2(1:3,7:9) =     F1C * (NID2 - eye(3));
	%DA2(1:3,10:12) =  -F1C * NID2 * crossm(f2);
	%DA2(1:3,13:15) =   F1C * NID3;
	%DA2(1:3,16:18) =  -F1C * NID3 * crossm(f3);
	% node 2 second contrib
	%DA2(1:3,1:3) =    -F2C * NIID1;
	%DA2(1:3,4:6) =     F2C * NIID1 * crossm(f1);
	%DA2(1:3,7:9) =    -F2C * (NIID2 - eye(3));
	%DA2(1:3,10:12) =   F2C * NIID2 * crossm(f2);
	%DA2(1:3,13:15) =  -F2C * NIID3;
	%DA2(1:3,16:18) =   F2C * NIID3 * crossm(f3);

	% node 2
	DA2(1:3,1:3) =     F1C * NID1 - F2C * NIID1;
	DA2(1:3,4:6) =    -F1C * NID1 * f1c * G1 + F2C * NIID1 * f1c * G1;
	DA2(1:3,7:9) =     F1C * (NID2 - eye(3)) - F2C * (NIID2 - eye(3));
	DA2(1:3,10:12) =  -F1C * NID2 * f2c * G2 + F2C * NIID2 * f2c * G2;
	DA2(1:3,13:15) =   F1C * NID3 - F2C * NIID3;
	DA2(1:3,16:18) =  -F1C * NID3 * f3c * G3 + F2C * NIID3 * f3c * G3;

	DA = [zeros(3,18); DA1; zeros(3,18); DA2; zeros(3,18); DA3];

	% involved dofs
	% assembly only rotations
	rdof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)]; % row 18 index
	cdof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)]; % column 18 index

	rindex = find(rdof); % look for constraints
	cindex = find(cdof); % look for constraints

	rdof = rdof(rindex);
	cdof = cdof(cindex);
	
	DAff = DA(rindex, cindex);
	
	[in, jn, vel] = find(DAff);
	
	iel = double(rdof(in)');
	jel = double(cdof(jn)');
	

%	rdof(iel)
	
	%iel = double(rdof(iel));
	%jel = double(cdof(jel));
	

%	iel = [];
%	jel = [];
%	vel = [];
%	nr = length(rindex);
%	nc = length(cindex);


%iel= repmat(rindex', 18, 1)s
%jel=[
%for k=1:18

%	vel = [vel; DA(:,k)];

%end



	% sparse mat row index		
%	iel = double(repmat(rdof(rindex)', nc, 1));

%	for k = 1:nc

%		jel( ((k-1) * nr +1) : nr * k, 1) = double(cdof(cindex(k)));

%		vel = [vel ; DAff(:,k)];

%	end  

	i = [i; iel]; j = [j; jel]; v = [v; vel];  

end

end
