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
%
% function [VELX, VELT, VELR] = body_vel(nbody, BAERO, lattice, VLMVEL, state, PG, SIGMA, MUY, MUZ, GAMMA, PARAM
%
% Input:
% nbody: body index
% BAERO: body struct
% lattice: VLM lattice
% VLMVEL: influence matrix for induced velocity from VLM (can be empty if weak coupling adopted)
% state: aero state
% PG: Prandtl Glauert number
% SIGMA: source amplitude
% MUY: doublet amplitude along y
% MUZ: doublet amplitude along z
% GAMMA: vortex amplitude
% PARAM: PARAM struct (.bou is used to specifiy body-vlm coupling, i.e.
% 1 for weak coupling, 2 for strong coupling)
%
% Output:
%
% VELX: Axial velocity on panels
% VELT: Tangential velocity on panels
% VELR: Radial velocity on panels
function [VELX, VELT, VELR] = body_vel(nbody, BAERO, lattice, VLMVEL, state, PG, SIGMA, MUY, MUZ, GAMMA, PARAM)
Rmat = BAERO.Rmat(:,:,nbody);
alpha = D2R(state.alpha);
beta = D2R(state.betha);
wind = state.AS.*([cos(alpha)*cos(beta) sin(beta) sin(alpha)*cos(beta)]);
wind = (Rmat'*wind')';
alpha = atan(wind(3)/wind(1));
beta = asin(wind(2)/state.AS);
np = length(BAERO.lattice.Elem.Theta{nbody}); % radial points
X = BAERO.lattice.Elem.Midpoint_loc{nbody}(1:np:end,1);
ns = length(X); % longitudinal section
VELX = zeros(ns, np);
VELT = zeros(ns, np);
VELR = zeros(ns, np);
%
nd = length(SIGMA);
PG2 = PG^2;
PG3 = PG*PG2;
PG4 = PG2*PG2;
%
if(~isempty(lattice) && PARAM.BCOU>1)
%  VLM = vlm2body_vel(lattice, BAERO, BAERO.lattice.Elem.Midpoint_loc{nbody}, ...
%                   state.SIMXZ, state.SIMXY, PG, GAMMA);
  ncolloc = size(BAERO.lattice.Elem.Midpoint_loc{nbody},1);
  ngamma = length(GAMMA);
  VLM = zeros(ncolloc ,3);
  for j=1:ncolloc
    for t=1:ngamma
%     determine induced velocity from VLM (subtract)
      VLM(j,:) =  VLM(j,:) - squeeze(VLMVEL{nbody}(j, t,:) .* GAMMA(t))';
    end
  end
  for j=1:ns % section loop
    for k=1:np % radial loop
      THETA = BAERO.lattice.Elem.Theta{nbody}(k);
      ST = sin(THETA);
      CT = cos(THETA);
      index = (j-1) * np +k;
      VLMR = (Rmat')*VLM(index,:)';
      VELX(j,k) = VLMR(1);
      VELT(j,k) = VLMR(2) * CT + VLMR(3) * ST;
      VELR(j,k) = VLMR(2) * ST + VLMR(3) * CT;
    end
  end
end

% source + doublet
%  THETA0 = acot(sin(alpha) * cot(-beta));
%  MUZ = -MU.* cos(THETA0);
%  MUY =  MU .* sin(THETA0);
  for j=1:ns % section loop
    for k=1:np % radial loop

      THETA = BAERO.lattice.Elem.Theta{nbody}(k);
      ST = sin(THETA);
      CT = cos(THETA);
%
      VELX(j,k) = VELX(j,k) + wind(1);
      VELT(j,k) = VELT(j,k) + wind(2) * CT + wind(3) * ST;
      VELR(j,k) = VELR(j,k) + wind(2) * ST + wind(3) * CT;
%
      for i=1:nd % doublet loop
        R     = BAERO.lattice.Elem.R{nbody}(j);
        X0 = BAERO.geo.x{nbody}(i);
        DELTAX = X(j)-X0;
        D3 = (DELTAX^2 + PG2*R^2)^(3/2);
        D5 = (DELTAX^2 + PG2*R^2)^(5/2);
%       doublet
        VELX(j,k) = VELX(j,k) + 3*(MUZ(i)*CT + MUY(i)*ST)*PG2*R*DELTAX/D5; 
        VELT(j,k) = VELT(j,k) +   (MUZ(i)*ST - MUY(i)*CT)*PG2/D3;
        VELR(j,k) = VELR(j,k) -   (MUZ(i)*CT + MUY(i)*ST)*PG2/D3 + ...
                                3*(MUZ(i)*CT + MUY(i)*ST)*PG4*R^2/D5;
%       source
        VELX(j,k) = VELX(j,k) + DELTAX/D3; 
        VELR(j,k) = VELR(j,k) + PG2 * R/D3;
%
%        VELX(j,k) = VELX(j,k) + 3*MUY(i)*PG2*R*ST*DELTAX/D5; 
%        VELT(j,k) = VELT(j,k) - MUY(i)*PG2*CT/D3;
%        VELR(j,k) = VELR(j,k) - MUY(i)*PG2*ST/D3 + 3*MUY(i)*PG4*R^2*ST/D5;
%
      end
    end
  end

end
%-------------------------------------------------------------------------------
% induced velocity from VLM to BODY
function [GAMMA_V] = vlm2body_vel(lattice, body, BCOLLOC, symmxz, symmxy, PG, GAMMA)
%
GAMMA_V = [];
HEIGHT = 2 * double(symmxy);
%
%

    GAMMA_V = induced_vel(BCOLLOC, lattice.VORTEX, GAMMA, PG);

    if (symmxz) % symmetry along xz vertical plane
      COLLOC = BCOLLOC;
      COLLOC(:,2) = -COLLOC(:,2);
      GAMMA_VS = induced_vel(COLLOC, lattice.VORTEX, GAMMA, PG);
      GAMMA_V = GAMMA_V + GAMMA_VS;
    end
    if (symmxy) % symmetry along xy horizontal plane (ground effect): symmxz specifies the height
      COLLOC = BCOLLOC;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      GAMMA_VS = induced_vel(COLLOC, lattice.VORTEX, GAMMA, PG);
      GAMMA_V = GAMMA_V + GAMMA_VS;
    end
    if (symmxy && symmxz) % symmetry along xz and xy plane
      COLLOC = BCOLLOC;
      COLLOC(:,3) = -COLLOC(:,3) - HEIGHT;
      COLLOC(:,2) = -COLLOC(:,2);
      NORMAL(:,2) = -NORMAL(:,2);
      NORMAL(:,3) = -NORMAL(:,3);
      GAMMA_VS = induced_vel(COLLOC, lattice.VORTEX, GAMMA, PG);
      GAMMA_V = GAMMA_V + GAMMA_VS;
    end

end
%***********************************************************************************************************************
% induced velocity from VLM to BODY
function DWB = induced_vel(COLLOC, VORTEX, GAMMA, PG)

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
DWB = zeros(nc, 3); 
		
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
		DWB(s, :) = DWB(s, :) + dwa .* GAMMA(t);
		
	end
end   

warning on

end
