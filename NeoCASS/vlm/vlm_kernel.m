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
% function [dwcond, FORCE] = vlm_kernel(geo, lattice, state)
%
%   DESCRIPTION: Plot beam model struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                geo            struct     VLM geo struct             
%                lattice        struct     VLM lattice struct             
%                state          struct     VLM state struct             
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                dwcond         real       circulation matrix cond number             
%                FORCE          real       Panel forces
%                GAMMA_P        matrix     VLM influce matrix
%
%    REFERENCES:
%
%*******************************************************************************

function [dwcond, FORCE, GAMMA_P] = vlm_kernel(geo, lattice, state)
%
symmxz = state.SIMXZ;
symmxy = state.SIMXY;
RHO = state.rho;
PG = 1.0;
%
if ( (state.pgcorr) && (state.Mach < 1.0))
  PG = sqrt(1-state.Mach^2);
end
%
dwcond = 0;
%
[np vor_length dim] = size(lattice.VORTEX);
%
Fx = zeros(np,1); Fy = zeros(np,1); Fz = zeros(np,1);
%
if vor_length ~= 8
  error('Wrong vortex struct dimension.');
end
%
[dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, 1, symmxz, symmxy, PG);
invK = inv(GAMMA_P);
b1 = vor_length / 2;
r=[]; c=[]; v=[];
for k=1:np
  r = [r, (k-1)*3+1:k*3];
  c = [c, k.*ones(1,3)];
  v = [v, squeeze([lattice.VORTEX(k, b1+1, :) - lattice.VORTEX(k, b1, :)])'];
end
r = double(r);
c = double(c);
v = double(v);
GAMMA = sparse(r,c,v,3*np,np);
%
r=[]; c=[]; v=[];
for k=1:np
  r = [r, (k-1)*3+2, (k-1)*3+3];
  c = [c, (k-1)*3+3, (k-1)*3+2];
  v= [v, [-1.0 1.0]];
end
VINFCROSS = sparse(r,c,v,3*np,3*np);
%
FORCE = (RHO*state.AS) .* (VINFCROSS * GAMMA * invK);