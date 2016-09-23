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
% function [DijY, DijZ, Sij] = get_body_matrix(nbody, BAERO, state, PG)
%
%   DESCRIPTION: Assemblly influence matrix for aero body
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                nbody          int        body index             
%                BAERO          struct     body struct             
%                state          struct     VLM state struct
%                PG             real       Prandtl Glauert             
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                DijY,DijZ,Sij  matrix     influence matrix for Y doublets, 
%                                          Z doublets and sources
%    REFERENCES:
%
%*******************************************************************************

function [Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, GAMMA_VHB] = get_body_matrix(nbody, lattice, BAERO, state, PARAM)
%
DijY = [];
DijZ = [];
Sij = [];
SijV = [];
DijYV = [];
DijZV = [];
GAMMA_HB = [];
GAMMA_VHB = [];
%
symmxz = state.SIMXZ;
symmxy = state.SIMXY;
RHO = state.rho;
PG = 1.0;
if ( (state.pgcorr) && (state.Mach < 1.0))
  PG = sqrt(1-state.Mach^2);
end
%
np = 0;
if ~isempty(lattice)
  [np vor_length dim] = size(lattice.VORTEX);
  if vor_length ~= 8
    error('Wrong vortex struct dimension.');
  end
end
%
ns = length(BAERO.geo.x{nbody});
%
ORIGIN = [BAERO.geo.x{nbody}, zeros(ns,2)];
ST = sin(BAERO.COLLOCT); CT = cos(BAERO.COLLOCT);
COLLOC = zeros(3*ns ,3); BNORM = ones(3*ns,3);
COLLOC(:,1) = repmat(BAERO.geo.x{nbody},3,1);
COLLOC(:,2) = [BAERO.geo.R{nbody}.*ST(1); BAERO.geo.R{nbody}.*ST(2); BAERO.geo.R{nbody}.*ST(3)];
COLLOC(:,3) = [BAERO.geo.R{nbody}.*CT(1); BAERO.geo.R{nbody}.*CT(2); BAERO.geo.R{nbody}.*CT(3)];
%
NORM(:,1) = repmat(-BAERO.geo.Rx{nbody},3,1);
NORM(:,2) = [repmat(ST(1),ns,1); repmat(ST(2),ns,1); repmat(ST(3),ns,1)];
NORM(:,3) = [repmat(CT(1),ns,1); repmat(CT(2),ns,1); repmat(CT(3),ns,1)];
NORM = NORM./repmat(sqrt(dot(NORM,NORM,2)),1,3);
%
% close body points for bc
%NORM(1:ns:end,1) = -1.0;
%NORM(1:ns:end,2) = 0.0;
%NORM(1:ns:end,3) = 0.0;
%NORM(ns:ns:end,1) = 1.0;
%NORM(ns:ns:end,2) = 0.0;
%NORM(ns:ns:end,3) = 0.0;
%
%COLLOC = [repmat(BAERO.geo.x{nbody},2,1), zeros(2*ns,1), [BAERO.geo.R{nbody}; -BAERO.geo.R{nbody}]];
%NORM = [repmat(-BAERO.geo.Rx{nbody},2,1), zeros(2*ns,2)];
%NORM(1:ns,3) = 1.0; 
%NORM(ns+1:end,3) = -1.0; 
%NORM = NORM ./ repmat(sqrt(dot(NORM,NORM,2)),1,3);
[DijY, DijZ] = doublet_el_yz(ORIGIN, COLLOC, NORM, PG);
Sij = source_el(ORIGIN, COLLOC, NORM, PG);
%
if ~isempty(lattice)
  SijV = zeros(np,3*ns); 
  DijYV = zeros(np,3*ns); 
  DijZV = zeros(np,3*ns); 
  GAMMA_HB = zeros(3*ns, np);
  if (PARAM.BCOU>0) % assembly body influence on vlm
    [SijV, DijYV, DijZV, IGN] = assembly_body2vlm_mat_yz(nbody, lattice, BAERO, state, PG);
  end
  if (PARAM.BCOU==2) % assembly vlm influence on body
    [GAMMA_HB, GAMMA_VHB] = assembly_vlm2body_mat(nbody, lattice, BAERO, symmxz, symmxy, PG);
  end
end
%
end





