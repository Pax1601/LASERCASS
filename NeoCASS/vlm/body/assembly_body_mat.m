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
% function [GAMMA_B, type] = assembly_body_mat(nbody, BAERO, state, PG)
%
%   DESCRIPTION: Assemblly influence matrix for aero body
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                nbody          int        body index             
%                BAERO          struct     body struct             
%                lattice        struct     VLM lattice struct             
%                state          struct     VLM state struct
%                PG             real       Prandtl Glauert             
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                GAMMA_B        matric     influence matrix
%    REFERENCES:
%
%*******************************************************************************

function [GAMMA_B, type] = assembly_body_mat(nbody, BAERO, state, PG)
%
Dij = [];
Sij = [];
%
Rmat = BAERO.Rmat(:,:,nbody);
alpha = D2R(state.alpha);
beta = D2R(state.betha);
wind = state.AS.*([cos(alpha)*cos(beta) sin(beta) sin(alpha)*cos(beta)]);
wind = (Rmat'*wind')';
alpha = atan(wind(3)/wind(1));
beta = asin(wind(2)/state.AS);
ns = length(BAERO.geo.x{nbody});
if (abs(alpha)>0 ||abs(beta)>0) % litfing case: add doublets
  ORIGIN = [BAERO.geo.x{nbody}, zeros(ns,2)];
  COLLOC = [repmat(BAERO.geo.x{nbody},2,1), zeros(2*ns,1), [BAERO.geo.R{nbody}; -BAERO.geo.R{nbody}]];
  NORM = [repmat(-BAERO.geo.Rx{nbody},2,1), zeros(2*ns,2)];
  NORM(1:ns,3) = 1.0; 
  NORM(ns+1:end,3) = -1.0; 
  NORM = NORM ./ repmat(sqrt(dot(NORM,NORM,2)),1,3);
  THETA0 = acot(sin(alpha) * cot(-beta));
  Dij = doublet_el(ORIGIN, COLLOC, NORM, THETA0, PG);
  Sij = source_el(ORIGIN, COLLOC, NORM, PG);
  GAMMA_B = [Sij,Dij];
  type = 1;
else  % thickness case: add sources only
  ORIGIN = [BAERO.geo.x{nbody}, zeros(ns,2)];
  COLLOC = [BAERO.geo.x{nbody}, zeros(ns,1), BAERO.geo.R{nbody}];
  NORM = [-BAERO.geo.Rx{nbody}, zeros(ns,2)];
  NORM(:,3) = 1.0;
  NORM = NORM ./ repmat(sqrt(dot(NORM,NORM,2)),1,3);
  Sij = source_el(ORIGIN, COLLOC, NORM, PG);
  GAMMA_B = [Sij];
  type = 0;
end
%
end





