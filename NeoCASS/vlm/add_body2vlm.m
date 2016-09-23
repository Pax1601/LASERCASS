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
%**************************************************************************
%
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% Combine VLM and BODY influence matrices
% lattice: VLM lattice
% BAERO: body struct
% GAMMA_MAT: VLM influence matrix or empty if no VLM available
% Sij, DijY, DijZ: body source and doublets influence matrices 
%                 on body collocation points
% SijV, DijYV, DijZV: body source and doublets influence matrices on VLM
% GHB: influence matrices from VLM to body collocation points 
%      (empty if weak coupling used)
% state: flight state
% PARAM: PARAM struct (.bou is used to specifiy body-vlm coupling, i.e.
% 1 for weak coupling, 2 for strong coupling)
%
% Output: 
% GAMMA_MAT: system matrix
% NELEM: number of elements for each body
%
function [GAMMA_MAT, NELEM] = add_body2vlm(lattice, BAERO, GAMMA_MAT, ...
                                             Sij, DijY, DijZ, ...
                                             SijV, DijYV, DijZV, GHB, state, PARAM)
%
nb = size(Sij,2);
nsprev = 0;
NELEM = zeros(nb,1);
np = 0;
if ~isempty(lattice)
  [np vor_length dim] = size(lattice.VORTEX);
  if vor_length ~= 8
    error('Wrong vortex struct dimension.');
  end
end
%
for i=1:nb
  Rmat = BAERO.Rmat(:,:,i);
  alpha = D2R(state.alpha);
  beta = D2R(state.betha);
  wind = state.AS.*([cos(alpha)*cos(beta) sin(beta) sin(alpha)*cos(beta)]);
  wind = (Rmat'*wind')';
  alpha = atan(wind(3)/wind(1));
  beta = asin(wind(2)/state.AS);
  GAMMA_HB = [];
  GAMMA_BH = [];
  ns = size(Sij{i},1);
  if ~isempty(lattice)
    GAMMA_HB = GHB{i};
    GAMMA_BH = [SijV{i}, DijYV{i}, DijZV{i}];
  end
  GAMMA_BB = [Sij{i}, DijY{i}, DijZ{i}];
  BODYINT   = [-GAMMA_BH; zeros(nsprev*(i-1),ns)];
  BODYINT2  = [GAMMA_HB, zeros(ns*(i-1),nsprev)];
  GAMMA_MAT = [GAMMA_MAT, BODYINT; BODYINT2, -GAMMA_BB]; % expand matrix with body contribution
  nsprev = ns;
  NELEM(i) = ns;
end
