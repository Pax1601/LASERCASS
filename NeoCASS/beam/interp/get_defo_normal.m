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
function NDEFO = get_defo_normal(COLLOC, VORTEX, DELTAN, span)

np = size(COLLOC,1);
REF_P = zeros(3,3);
NDEFO = zeros(np, 3);
% determine new normal by the cross product of panel diagonals
r1 = zeros(1,3);
r2 = zeros(1,3);
Z  = zeros(1,3);
Y  = zeros(1,3);

for n = 1:np

	r1(1:3) = VORTEX(n, 4, 1:3);
    r2(1:3) = VORTEX(n, 5, 1:3);
    
    r1 = COLLOC(n, 1:3) - r1;
    r2 = COLLOC(n, 1:3) - r2;
    %
    Z = cross(r1,r2);
    if span<0
      Z = -Z;
    end
    Z = Z ./ norm(Z) ;
    Y = cross(Z ,[1, 0 , 0]);
    Y = Y ./ norm(Y);
    X = cross(Y, Z);
    X = X ./ norm(X);
    REF_P = [X(1) Y(1) Z(1); X(2) Y(2) Z(2); X(3) Y(3) Z(3)]; % local panel reference frame
% 	NDEFO(n, 1:3) = Z + (REF_P * DELTAN(n,:)')';
    NDEFO(n, 1:3) = Z + DELTAN(n,:);
end

end