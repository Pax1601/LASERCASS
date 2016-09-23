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
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************

% to be called when mesh is deformed
function [dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, type, symmxz, symmxy, PG)

dwcond = 0;
GAMMA_P = [];
GAMMA_I = [];
HEIGHT = 2 * double(symmxy);

[np vor_length dim] = size(lattice.VORTEX);
										   
switch vor_length 

	case 8
	% do nothing
	
	otherwise

	error('\n\t Wrong dimensions for vortex database.');

end

switch (type)

  case 1

    [GAMMA_P, GAMMA_I] = induced_dnwash(lattice.COLLOC, lattice.VORTEX, lattice.N, PG);

    if (symmxz) % symmetry along xz vertical plane
      COLLOC = lattice.COLLOC;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL = lattice.N;
      NORMAL(:,2) = -NORMAL(:,2);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end
    if (symmxy) % symmetry along xy horizontal plane (ground effect): symmxz specifies the height
      COLLOC = lattice.COLLOC;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      NORMAL = lattice.N;
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end
    if (symmxy && symmxz) % symmetry along xz and xy plane
      COLLOC = lattice.COLLOC;
      NORMAL = lattice.N;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL(:,2) = -NORMAL(:,2);
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end

  case 2

    b1 = vor_length / 2;
    MCOLLOC = zeros(np, 3, 1);
    MCOLLOC(:,1:3,1) = (lattice.VORTEX(:,b1,:) + lattice.VORTEX(:,b1+1,:))./2;
    [GAMMA_P, GAMMA_I] = induced_dnwash(MCOLLOC, lattice.VORTEX, lattice.N, PG);
    if (symmxz) % symmetry along xz vertical plane
      COLLOC = MCOLLOC;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL = lattice.N;
      NORMAL(:,2) = -NORMAL(:,2);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end
    if (symmxy) % symmetry along xy horizontal plane (ground effect): symmxz specifies the height
      COLLOC = MCOLLOC;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      NORMAL = lattice.N;
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end
    if (symmxy && symmxz) % symmetry along xz plane
      COLLOC = MCOLLOC;
      NORMAL = lattice.N;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL(:,2) = -NORMAL(:,2);
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      GAMMA_I = GAMMA_I + GAMMA_IS;
    end

  otherwise

    error('Unknown option type in assembly_vlm_mat.m function.');

end

dwcond = cond(GAMMA_P);

end
%***********************************************************************************************************************
function [A, DWB] = induced_dnwash(COLLOC, VORTEX, NS, PG)

one_by_four_pi=1/(4*pi);
near = 1e-7;     	   
nc = size(COLLOC,1);
[np vor_length dim] = size(VORTEX);

i = [1 2 3]; j = [2 3 1]; k = [3 1 2]; %index for cross product computation;

warning off
% apply Prandlt Glauert
COLLOC(:,1)   = COLLOC(:,1)./PG;
VORTEX(:,:,1) = VORTEX(:,:,1)./PG;
%
RA = zeros(7, 3);
RB = zeros(7, 3);
RC = zeros(7, 3);

A = zeros(nc, np);
DWB = zeros(nc, np, 3); 
		
for s = 1:nc

  RC = repmat(COLLOC(s,:), 7, 1);

	for t = 1:np

		RA(:,:) = VORTEX(t, 1:end-1, :);
		RB(:,:) = VORTEX(t, 2:end, :);

		R0 = RB - RA;    
		R1 = RC - RA;
		R2 = RC - RB; 

		Lem = R1(:, j) .* R2(:, k) - R1(:, k) .* R2(:, j);
		Lemma_length_square = sum(Lem.^2, 2);		

		LR1 = sqrt(sum(R1.^2, 2)); % r1 value            
		LR2 = sqrt(sum(R2.^2, 2)); % r2 value
		% Moran notation
		Fac1ab(:, 1) = Lem(:, 1) ./ Lemma_length_square;
		Fac1ab(:, 2) = Lem(:, 2) ./ Lemma_length_square; 
		Fac1ab(:, 3) = Lem(:, 3) ./ Lemma_length_square; 

		Fac2ab(:, 1) = R1(:, 1) ./ LR1 - R2(:, 1) ./ LR2;
		Fac2ab(:, 2) = R1(:, 2) ./ LR1 - R2(:, 2) ./ LR2;
		Fac2ab(:, 3) = R1(:, 3) ./ LR1 - R2(:, 3) ./ LR2;

		Fac2ab = sum(R0 .* Fac2ab, 2);

		dw(:,1) = Fac1ab(:, 1) .* Fac2ab;
		dw(:,2) = Fac1ab(:, 2) .* Fac2ab;
		dw(:,3) = Fac1ab(:, 3) .* Fac2ab;
%   Apply Prandtl Glauert
    dw(:,1) = dw(:,1) ./ PG;
		dw(Lemma_length_square < near, :) = 0; % avoid vortex core singularity
		dw = dw * one_by_four_pi;

		dwa = -sum(dw, 1);
		DWB(s, t, :) = dwa;
		
		A(s, t) = dwa * NS(s, :)'; % project downwash vector onto surface

	end
end   

warning on

end
