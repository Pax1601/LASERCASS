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
function [GAMMA_P, GAMMA_I2] = assembly_vlm2body_mat(i, lattice, body, symmxz, symmxy, PG)

dwcond = 0;
%
GAMMA_P = [];
GAMMA_I = [];
GAMMA_P2 = [];
GAMMA_I2 = [];

HEIGHT = 2 * double(symmxy);
%
Rmat = body.Rmat(:,:,i);
[np vor_length dim] = size(lattice.VORTEX);
ns = length(body.geo.x{i});
%VORTEX = lattice.VORTEX;
%for k=1:size(VORTEX,1)
%  for j=1:size(VORTEX,2)
%    VORTEX(k,j,1:3) = Rmat'*(squeeze(VORTEX(k,j,:)) -body.geo.ref_point(i,:)'); 
%  end
%end
%if (btype==1)
%  BCOLLOC = [repmat(body.geo.x{i},2,1), zeros(2*ns,1), [body.geo.R{i}; -body.geo.R{i}]];
%  BNORM = [repmat(-body.geo.Rx{i},2,1), zeros(2*ns,2)];
%  BNORM(1:ns,3) = 1.0; 
%  BNORM(ns+1:end,3) = -1.0; 
%  BNORM = BNORM ./ repmat(sqrt(dot(BNORM,BNORM,2)),1,3);
ns = 3*ns;
%else
%  BCOLLOC = [body.geo.x{i}, zeros(ns,1), body.geo.R{i}];
%  BNORM = [-body.geo.Rx{i}, zeros(ns,2)];
%  BNORM(:,3) = 1.0;
%  BNORM = BNORM ./ repmat(sqrt(dot(BNORM,BNORM,2)),1,3);
%end
%
BCOLLOC = body.lattice.COLLOC{i}(1:ns,:);
BNORM = body.lattice.N{i}(1:ns,:);

BCOLLOC2 = body.lattice.Elem.Midpoint_loc{i};
BNORM2 = body.lattice.Elem.Norm{i};
%
% set normal pointing into body
  BNORM = -BNORM;									   
%
  switch vor_length 

	  case 8
	  % do nothing
	
	  otherwise

	  error('\n\t Wrong dimensions for vortex database.');

  end

    [GAMMA_P, GAMMA_I] = induced_dnwash(BCOLLOC, lattice.VORTEX, BNORM, PG);
    [GAMMA_P2, GAMMA_I2] = induced_dnwash(BCOLLOC2, lattice.VORTEX, BNORM2, PG);
%    [GAMMA_P, GAMMA_I] = induced_dnwash(BCOLLOC, VORTEX, BNORM, PG);

    if (symmxz) % symmetry along xz vertical plane
      COLLOC = BCOLLOC;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL = BNORM;
      NORMAL(:,2) = -NORMAL(:,2);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      COLLOC = BCOLLOC2;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL = BNORM2;
      NORMAL(:,2) = -NORMAL(:,2);
      [GAMMA_PS2, GAMMA_IS2] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_I2 = GAMMA_I2 + GAMMA_IS2;
    end
    if (symmxy) % symmetry along xy horizontal plane (ground effect): symmxz specifies the height
      COLLOC = BCOLLOC;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      NORMAL = BNORM;
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      COLLOC = BCOLLOC2;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      NORMAL = BNORM2;
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS2, GAMMA_IS2] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_I2 = GAMMA_I2 + GAMMA_IS2;
    end
    if (symmxy && symmxz) % symmetry along xz and xy plane
      COLLOC = BCOLLOC;
      NORMAL = BNORM;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL(:,2) = -NORMAL(:,2);
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS, GAMMA_IS] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_P = GAMMA_P + GAMMA_PS;
      COLLOC = BCOLLOC2;
      NORMAL = BNORM2;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL(:,2) = -NORMAL(:,2);
      NORMAL(:,3) = -NORMAL(:,3);
      [GAMMA_PS2, GAMMA_IS2] = induced_dnwash(COLLOC, lattice.VORTEX, NORMAL, PG);
      GAMMA_I2 = GAMMA_I2 + GAMMA_IS2;
    end
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
