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

function N = set_N_mat(c01, c02, c03, c1, c2, c3, f1, f2, f3)

N = zeros(12,18);
JI = zeros(1,3);
JII = zeros(1,3);
PpI = zeros(3,1);
PpII = zeros(3,1);

x = [c01(1) c02(1) c03(1)];
y = [c01(2) c02(2) c03(2)];
z = [c01(3) c02(3) c03(3)];

% COLLOC 1
eta = -1/sqrt(3);
NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
NIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC1
JI(1) = dot(NIp, x);
JI(2) = dot(NIp, y);
JI(3) = dot(NIp, z);
NIp = NIp ./ norm(JI);

% COLLOC 2
eta = +1/sqrt(3);
NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
NIIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC2 
JII(1) = dot(NIIp, x);
JII(2) = dot(NIIp, y);
JII(3) = dot(NIIp, z);
NIIp = NIIp ./ norm(JII);
                                 
x = [c1(1) c2(1) c3(1)];
y = [c1(2) c2(2) c3(2)];
z = [c1(3) c2(3) c3(3)];

PpI(1) = dot(NIp, x);
PpI(2) = dot(NIp, y);
PpI(3) = dot(NIp, z);
PpII(1) = dot(NIIp, x);
PpII(2) = dot(NIIp, y);
PpII(3) = dot(NIIp, z);

% FIRST section
% translation
% node 1
N(1:3,1:3) = diag([NIp(1) NIp(1) NIp(1)]);
N(1:3,4:6) = -crossm(NIp(1) .* f1) + crossm(PpI .* NI(1));
% node 2 
N(1:3,7:9) = diag([NIp(2) NIp(2) NIp(2)]);
N(1:3,10:12) = -crossm(NIp(2) .* f2) + crossm(PpI .* NI(2));
% node 3
N(1:3,13:15) = diag([NIp(3) NIp(3) NIp(3)]);
N(1:3,16:18) = -crossm(NIp(3) .* f3) + crossm(PpI .* NI(3));
% rotation
N(4:6,4:6) =   diag([NIp(1) NIp(1) NIp(1)]);
N(4:6,10:12) = diag([NIp(2) NIp(2) NIp(2)]);
N(4:6,16:18) = diag([NIp(3) NIp(3) NIp(3)]);
% SECOND section
% translation
% node 1		
N(7:9,1:3) = diag([NIIp(1) NIIp(1) NIIp(1)]);
N(7:9,4:6) = -crossm(NIIp(1) .* f1) + crossm(PpII .* NII(1));
% node 2 
N(7:9,7:9) = diag([NIIp(2) NIIp(2) NIIp(2)]);
N(7:9,10:12) = -crossm(NIIp(2) .* f2) + crossm(PpII .* NII(2));
% node 3
N(7:9,13:15) = diag([NIIp(3) NIIp(3) NIIp(3)]);
N(7:9,16:18) = -crossm(NIIp(3) .* f3) + crossm(PpII .* NII(3));
% rotation
N(10:12,4:6) =   diag([NIIp(1) NIIp(1) NIIp(1)]);
N(10:12,10:12) = diag([NIIp(2) NIIp(2) NIIp(2)]);
N(10:12,16:18) = diag([NIIp(3) NIIp(3) NIIp(3)]);
