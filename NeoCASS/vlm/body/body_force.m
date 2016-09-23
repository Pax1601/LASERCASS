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
%
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
%
% function [Fx, Fy, Fz] = body_force(nbody, body, VELX, VELT, VELR, state)
%
% Inputs:
% nbody: body index
% body: body struct
% VELX, VELT, VELR: body axial, tangential and radial velocities
% state: state flight point
%
% Outout:
% Fx, Fy, Fy body panel forces
%
function [Fx, Fy, Fz] = body_force(nbody, body, VELX, VELT, VELR, state)
%
ns = size(VELX,1);
np = size(VELX,2);
%
AS = state.AS;
AS2 = AS*AS;
QINF = 0.5*state.rho*AS2;
%
AREA = body.lattice.Elem.Area{nbody};
NORM = body.lattice.Elem.Norm{nbody};
%
for j=1:ns
  for k=1:np
    index = (j-1)*np + k; 
    CP(index) = 1 - (VELX(j,k)^2+VELT(j,k)^2+VELR(j,k)^2) / AS2;
    Fx(index) = QINF * CP(index) * AREA(j) * NORM(index,1);
    Fy(index) = QINF * CP(index) * AREA(j) * NORM(index,2);
    Fz(index) = QINF * CP(index) * AREA(j) * NORM(index,3);
  end
end