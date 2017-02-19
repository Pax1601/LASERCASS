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

function [Kp] = set_Kp_mat(f1, f2, f3, PRESTRESS)

Kp = zeros(18,18);


% COLLOC 1
eta = -1/sqrt(3);
NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];

% COLLOC 2
eta = +1/sqrt(3);
NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];



% PRESTRESS CONTRIBUTION
tI0 = PRESTRESS(1, 1:3);
tII0 = PRESTRESS(2, 1:3);

Kp(4:6,1:3) = crossm(tI0) * (eye(3) * (NI(1) - 1));
Kp(4:6,4:6) = -crossm(tI0 .* NI(1)) * crossm(diag([f1, f2, f3]));

Kp(4:6,7:9) = crossm(tI0 .* NI(2));
Kp(4:6,10:12) = -crossm(tI0 .* NI(2)) * crossm(diag([f1, f2, f3]));

Kp(4:6,13:15) = crossm(tI0 .* NI(3));
Kp(4:6,16:18) = -crossm(tI0 .* NI(3)) * crossm(diag([f1, f2, f3]));


Kp(10:12,1:3) = -crossm(tI0 .* NI(1)) + crossm(tII0 .* NII(1));
Kp(10:12,4:6) = crossm(tI0 .* NI(1)) * crossm(diag([f1, f2, f3])) - crossm(tII0 .* NII(1)) * crossm(diag([f1, f2, f3]));

Kp(10:12,7:9) = -crossm(tI0) * (eye(3) * (NI(2) - 1)) + crossm(tII0) * (eye(3) * (NII(2) - 1));
Kp(10:12,10:12) = crossm(tI0 .* NI(2)) * crossm(diag([f1, f2, f3])) - crossm(tII0 .* NII(2)) * crossm(diag([f1, f2, f3]));

Kp(10:12,13:15) = -crossm(tI0 .* NI(3)) + crossm(tII0 .* NII(3));
Kp(10:12,16:18) = crossm(tI0 .* NI(3)) * crossm(diag([f1, f2, f3])) - crossm(tII0 .* NII(3)) * crossm(diag([f1, f2, f3]));


Kp(16:18,1:3) = crossm(tII0 .* NII(1));
Kp(16:18,4:6) = -crossm(tII0 .* NII(1)) * crossm(diag([f1, f2, f3]));

Kp(16:18,7:9) = crossm(tII0 .* NII(2));
Kp(16:18,10:12) = -crossm(tII0 .* NII(2)) * crossm(diag([f1, f2, f3]));

Kp(16:18,13:15) = crossm(tII0) * (eye(3) * (NII(3) - 1));
Kp(16:18,16:18) = -crossm(tII0 .* NII(3)) * crossm(diag([f1, f2, f3]));
