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
% Solve for VLM and BODY aerodynamics
% geo: geo struct
% lattice: VLM lattice
% body: body struct
% state: flight state
% PARAM: PARAM struct (.bou is used to specifiy body-vlm coupling, i.e.
% 1 for weak coupling, 2 for strong coupling)
% GAMMA_P, GAMMA_I: vlm matrices
% GAMMA_VEL: vlm induced velocity on body
%
% Sij, DijY, DijZ: body source and doublets influence matrices 
%                 on body collocation points
% SijV, DijYV, DijZV: body source and doublets influence matrices on VLM
% GAMMA_HB: influence matrices from VLM to body collocation points 
%           (empty if weak coupling used)
% DOFINT: panels excluded from coupling and aero forces
%
% Output: 
% results: VLM and body forces, resultants
% GAMMA_MAT: system matrix
% NELEM: number of elements for each body
%

function [results, GAMMA_MAT, NELEM] = solve_vlm_body(geo, lattice, body, state, PARAM, ...
                                                 GAMMA_P, GAMMA_I, GAMMA_VEL, ...
                                                 Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, DOFINT)
%
results.FORCES = zeros(1,3);
results.MOMENTS = zeros(2,3);
%
GAMMA_MAT = []; NELEM = [];
COLLOC = []; NORM = [];
if ~isempty(lattice)
  GAMMA_MAT = GAMMA_P;
  COLLOC = [COLLOC; lattice.COLLOC];
  NORM = [NORM; lattice.N];
end
nbody = size(Sij,2);
if (nbody)
  [GAMMA_MAT, NELEM] = add_body2vlm(lattice, body, GAMMA_MAT, ...
                       Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, state, PARAM);
  for i=1:nbody
    COLLOC = [COLLOC; body.lattice.COLLOC{i}(1:NELEM(i),:)];
    NORM   = [NORM; body.lattice.N{i}(1:NELEM(i),:)];
  end
end
%   Boundary conditions
DWN = rigid_body_vlm_rhs(state, geo, COLLOC, NORM);

SOL = GAMMA_MAT \ DWN;
%   VLM forces
if ~isempty(lattice)
  results = recover_vlm_forces(SOL, GAMMA_I, geo, lattice, state, DOFINT);
end
%   body forces
if (nbody)
  results = recover_body_forces(results, SOL, NELEM, geo, lattice, GAMMA_VEL,...
                                                    body, state, PARAM);
end