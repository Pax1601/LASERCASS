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

function [i, j, v] = set_DT_pre_mat(nbar, BAR, BARR, DOF, COORD0, COORD, NODER0, NODER, PRESTRESS)

i = [];
j = [];
v = [];

	for n=nbar

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
		Kp = set_Kp_mat(f1, f2, f3, PRESTRESS(:, :, n))
		Kel = (A * D) * N + Kp;

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
