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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function [dwcond, Fx, Fy, Fz] = get_panel_forces(RHS, geo, lattice, state)
%
%   DESCRIPTION: Plot beam model struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                RHS            real array VLM rhs             
%                geo            struct     VLM geo struct             
%                lattice        struct     VLM lattice struct             
%                state          struct     VLM state struct             
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                dwcond         real       circulation matrix cond number             
%                FX             real       Force along x axis             
%                FY             real       Force along y axis             
%                FZ             real       Force along z axis             
%
%    REFERENCES:
%
%*******************************************************************************

function [dwcond, Fx, Fy, Fz, Fxb, Fyb, Fzb] = get_panel_forces(geo, lattice, body, state, PARAM)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;
RHO = state.rho;
PG = 1.0;
Fx = []; Fy = []; Fz = [];
Fxb = {}; Fyb = {}; Fzb = {};

if ( (state.pgcorr) && (state.Mach < 1.0))
  PG = sqrt(1-state.Mach^2);
end
dwcond = 0;
% VLM panels
np = 0;
% aero bodies
nb = length(body.ID);
% initialize data
GAMMA_MAT = [];
COLLOC = [];
NORM = [];
gamma = [];
%-------------------------------------------------------------------------------
% VLM
%
if ~isempty(lattice)
  [np vor_length dim] = size(lattice.VORTEX);
  if vor_length ~= 8
    error('Wrong vortex struct dimension.');
  end
  Fx = zeros(np,1); Fy = zeros(np,1); Fz = zeros(np,1);
  [dwcond, GAMMA_HH, GAMMA_VHH] = assembly_vlm_mat(lattice, 1, symmxz, symmxy, PG);
  GAMMA_MAT = GAMMA_HH;
  COLLOC = [COLLOC; lattice.COLLOC];
  NORM = [NORM; lattice.N];
end
%-------------------------------------------------------------------------------
% AERO bodies
%
nsprev = 0;
body_info = zeros(nb,2); VLMVEL = {};
for i=1:nb
  [GAMMA_BB, btype]   = assembly_body_mat(i, body, state, PG);  % assembly body self influence
  ns = size(GAMMA_BB,1);
  GAMMA_BH = []; GAMMA_HB = []; GAMMA_VHB = [];
  COLLOC = [COLLOC; body.lattice.COLLOC{i}(1:ns,:)];
  NORM   = [NORM; body.lattice.N{i}(1:ns,:)];
  if ~isempty(lattice)
    GAMMA_HB = zeros(ns, np); GAMMA_BH = zeros(np, ns);
    if (PARAM.BCOU>0) % assembly body influence on vlm
      [GAMMA_BH, IGNORED] = assembly_body2vlm_mat(i, lattice, body, state, btype, PG); % assembly body influence on vlm
    end
    if (PARAM.BCOU==2) % assembly vlm influence on body
      [GAMMA_HB, VLMVEL{i}] = assembly_vlm2body_mat(i, lattice, body, symmxz, symmxy, btype, PG);
    end
  end
  BODYINT   = [-GAMMA_BH; zeros(nsprev*(i-1),ns)];
  BODYINT2  = [GAMMA_HB, zeros(ns*(i-1),nsprev)];
  GAMMA_MAT = [GAMMA_MAT, BODYINT; BODYINT2, -GAMMA_BB]; % expand matrix with body contribution
  nsprev = ns;
  body_info(i,1) = ns;
  body_info(i,2) = btype;
end
%-------------------------------------------------------------------------------
% Boundary conditions
DWN = rigid_body_vlm_rhs(state, geo, COLLOC, NORM);
% solution
SOL = GAMMA_MAT \ DWN;
%-------------------------------------------------------------------------------
% recover VLM solution
if ~isempty(lattice)
  gamma = SOL(1:np);
  [dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, 2, symmxz, symmxy, PG);
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
end
%-------------------------------------------------------------------------------
% recover BODY solution
offset = np;
for i=1:nb
  if (body_info(i,2)==1)
    ns = body_info(i,1)/2;
    SIGMA = SOL(offset+1:offset+ns);
    MU    = SOL(offset+1+ns:offset+2*ns);
  else
    SIGMA = SOL(offset+1:offset+body_info(i,1));
    MU = [];
  end
  offset = offset + body_info(i,1);
% recover velocities
  [VELX, VELT, VELR] = body_vel(i, body, lattice, VLMVEL, state, PG, MU, SIGMA, gamma, PARAM);
% recover forces
  [BX, BY, BZ] = body_force(i, body, VELX, VELT, VELR, state);
  Fxb{i} = BX';  
  Fyb{i} = BY';  
  Fzb{i} = BZ';  
end
