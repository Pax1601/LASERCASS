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
% Determine body forces and sum resultants to VLM solution
%
% results0: VLM results struct (can be empty if no VLM)
% SOL: solution vector with gamma values (and body source and doublets)
% NELEM: for each body, number of singularity points
% geo: geo struct
% lattice: lattice struct
% VLMVEL: influence matric of induced veocity from VLM to 
%          body panels collocation points
% body: bosy struct
% state: flight state
% PARAM: PARAM struct (.bou is used to specifiy body-vlm coupling, i.e.
% 1 for weak coupling, 2 for strong coupling)
%
function results = recover_body_forces(results0, SOL, NELEM, geo, lattice, VLMVEL, body, state, PARAM)

np = 0; gamma = [];
nb = length(NELEM);
PG = 1.0;
Fxb = {}; Fyb = {}; Fzb = {};
if ( (state.pgcorr) && (state.Mach < 1.0))
  PG = sqrt(1-state.Mach^2);
end
%
if (isempty(results0))
  results.FORCES = zeros(1,3);
	results.MOMENTS = zeros(2,3);
else
  results = results0;
end
%
if ~isempty(lattice)
  [np vor_length dim] = size(lattice.VORTEX);
  gamma = SOL(1:np);
end
offset = np;
for i=1:nb
  ns = NELEM(i)/3;
  SIGMA = SOL(offset+1:offset+ns);
  MUY    = SOL(offset+1+ns:offset+2*ns);
  MUZ    = SOL(offset+1+2*ns:offset+3*ns);
  offset = offset + NELEM(i);
% recover velocities
  [VELX, VELT, VELR] = body_vel(i, body, lattice, VLMVEL, state, PG, SIGMA, MUY, MUZ, gamma, PARAM);
% recover forces
  [BX, BY, BZ] = body_force(i, body, VELX, VELT, VELR, state);
  Fxb{i} = BX';  
  Fyb{i} = BY';  
  Fzb{i} = BZ';  
  results.Fb{i} = [Fxb{i}, Fyb{i}, Fzb{i}];
  results.FNb{i} = sum(body.lattice.Elem.Norm{i} .* results.Fb{i}, 2);
  results.Mb{i} = results.Fb{i};
	[SFORCES, SMOMENTS, results.Mb{i}] = get_rigid_body_forces2(geo, i, body, results.Fb{i}); 
  results.FORCES  = results.FORCES + SFORCES;
  results.MOMENTS = results.MOMENTS + SMOMENTS;
end
