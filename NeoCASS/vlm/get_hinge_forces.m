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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
% determine hinge moments along all surfaces
function [Mtot] = get_hinge_forces(geo, lattice, Fp) 
%
Mtot = [];
Mtot = zeros(geo.nc,1);
%
colm = [4 5];
a = size(lattice.COLLOC, 1);
x1 = zeros(a,2); x2 = zeros(a,2); x3 = zeros(a,2);
v1 = zeros(a,1); v2 = zeros(a,1); v3 = zeros(a,1);
x1 = lattice.VORTEX(:, colm, 1); x2 = lattice.VORTEX(:, colm, 2); x3 = lattice.VORTEX(:, colm, 3);
v1 = mean(x1,2); v2 = mean(x2,2); v3 = mean(x3,2);
MIDV = zeros(a, 3);
MIDV = [v1, v2, v3];
%
  for i=1:geo.nc
%
    dof = lattice.Control.DOF(i).data;    
    np = length(lattice.Control.DOF(i).data);
    F = Fp(dof,:);
    P0 = squeeze(lattice.Control.Hinge(i,1,:))';
    P1 = squeeze(lattice.Control.Hinge(i,2,:))';
    ARM = MIDV(dof,:) - repmat(P0, np, 1);
    Mp = zeros(1, 3);
    Mp = sum(cross(ARM, Fp(dof,:), 2),1); % panels moment
    % project along hinge
    v = P1 - P0;
    v = v ./norm(v);
    Mtot(i) = dot(v, Mp);
%
  end
%
end
