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
% Determine panel forces and resultants along VLM mesh
%
% SOL: solution vector with gamma values (and body source and doublets)
% geo: geo struct
% lattice: lattice struct
% state: state flight
% DOFINT: carrythrough panels to exclude from resultants if interfering with bodies
%
% results: panel forces, normal forces and resultants
function results = recover_vlm_forces(SOL, GAMMA_I, geo, lattice, state, DOFINT)

Fx = []; Fy = []; Fz = [];

if ~isempty(lattice)
  [np vor_length dim] = size(lattice.VORTEX);
  gamma = SOL(1:np);
  RHO = state.rho;
%
  VX = zeros(np, 3); VY = zeros(np, 3); VZ = zeros(np, 3); VD = zeros(np, 3);
%
  VX = GAMMA_I(:,:,1) * gamma;
  VY = GAMMA_I(:,:,2) * gamma; 
  VZ = GAMMA_I(:,:,3) * gamma;
  b1 = vor_length / 2;
  VD(:,:) = (lattice.VORTEX(:, b1+1, :) - lattice.VORTEX(:, b1, :));
  wind = state.AS.*([cos(state.alpha)*cos(state.betha) sin(state.betha) sin(state.alpha)*cos(state.betha)]);
  VFLOW = repmat(wind, np, 1) - [VX VY VZ];
  try
    geo.CG;
  catch 
    geo.CG = geo.ref_point;
  end
  OMEGA = [state.P state.Q state.R];
  if norm(OMEGA)
    VBODY = cross((lattice.COLLOC - repmat(geo.CG, np, 1)),...
            repmat(OMEGA, np, 1), 2);
    VFLOW = VFLOW + VBODY;
  end
  F = zeros(np, 3);
  F = RHO .* cross(VFLOW, VD, 2);
  Fx = F(:,1).*gamma; Fy = F(:,2).*gamma; Fz = F(:,3).*gamma;

	results.F = zeros(np, 3);
  results.M = zeros(np, 3);
  results.FN = zeros(np,1);
  %
  results.F = [Fx, Fy, Fz];
  % exclude CT forces
  results.F(DOFINT,:) = 0.0;
  results.FN = sum(lattice.N .* results.F, 2);
  %
  [results.FORCES, results.MOMENTS, results.M] = get_rigid_body_forces(geo, lattice, results.F); 
  % get hinge moments
  results.HINGE = [];
  if geo.nc 
    results.HINGE = get_hinge_forces(geo, lattice, results.F); 
  end 

end