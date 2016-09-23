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

function [Ftot, Mtot, Mp] = get_rigid_body_forces2(geo, nbody, BAERO, Fp) 

Ftot = zeros(1,3);  
Mtot = zeros(2,3);% first row to get value in the reference point, second to get value in the CG
colm = [4 5];
COLLOC = BAERO.lattice.Elem.Midpoint_loc{nbody};
a = size(COLLOC, 1);

ARM = COLLOC - repmat(geo.ref_point, a, 1); % get arm matrix for moment calculations around aircraft CG (aero forces are supposed to be in c/4 vortex midpoint)
Mp = zeros(a, 3);
Mp = cross(ARM, Fp, 2); % panels moment

Ftot = sum(Fp, 1);
Mtot(1, :) = sum(Mp, 1);

ARM = COLLOC - repmat(geo.CG, a, 1); % get arm matrix for moment calculations around aircraft CG (aero forces are supposed to be in c/4 vortex midpoint)

Mp_cg = zeros(a, 3);
Mp_cg = cross(ARM, Fp, 2); % panels moment

Mtot(2, :) = sum(Mp_cg, 1);

end
