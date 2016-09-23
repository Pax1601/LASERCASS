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
% Determine inertia principal axis and principal moments
function [Mpa, V] = wb_principal_axis(MCG)

J = MCG(4:6,4:6);
[V, Jp] = eig(J);

Jp = diag(Jp);
[m , i] = max(abs(V),[],1); % check is principal axis are alligned along reference frame axis
if sum(i) ~= 6 % 1 + 2 +3
	
	error('Unable to determine inertia principal axes.');

end

V = V(:,i);
Jp = Jp(i);

n1 = norm(V(:,1)); n2 = norm(V(:,2)); n3 = norm(V(:,3));

V(:,1) = V(:,1) ./ n1; V(:,2) = V(:,2) ./ n2; V(:,3) = V(:,3) ./ n3;

% check for correct axis orientation
% x- axis

%if V(1,1) < 0 

%	V(:,1) = -V(:,1);
%	V(2:3, 1) = -V(2:3, 1);

%end

% y- axis

%if V(2,2) < 0 

%	V(:,2) = -V(:,2);
%	col = [1 3];
%	V([1,3], 2) = -V([1,3], 2);

%end

V(:,3) = cross(V(:,1), V(:,2));
V(:,3) = V(:,3) ./ norm(V(:,3));
Mpa = zeros(6,6);
Mpa(1:3,1:3) = MCG(1:3,1:3);
Mpa(4:6,4:6) = diag(Jp);
