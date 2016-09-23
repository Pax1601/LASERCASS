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
% Transform section stifness matrix in the basic reference system
%
function R = set_R_matT(R1, R2, R3)

i1 = [1 1 1 2 2 2 3 3 3 4 4 4 5 5 5 6 6 6]; % R1, R1
j1 = [1 2 3 1 2 3 1 2 3 4 5 6 4 5 6 4 5 6];
%
i2 = i1 + 6; % R2, R2
j2 = j1 + 6;
i3 = i2 + 6;
j3 = j2 + 6;

%
v1 = [R1(1,1) R1(1,2) R1(1,3) R1(2,1) R1(2,2) R1(2,3) R1(3,1) R1(3,2) R1(3,3) R1(1,1) R1(1,2) R1(1,3) R1(2,1) R1(2,2) R1(2,3) R1(3,1) R1(3,2) R1(3,3)];
v2 = [R2(1,1) R2(1,2) R2(1,3) R2(2,1) R2(2,2) R2(2,3) R2(3,1) R2(3,2) R2(3,3) R2(1,1) R2(1,2) R2(1,3) R2(2,1) R2(2,2) R2(2,3) R2(3,1) R2(3,2) R2(3,3)];
v3 = [R3(1,1) R3(1,2) R3(1,3) R3(2,1) R3(2,2) R3(2,3) R3(3,1) R3(3,2) R3(3,3) R3(1,1) R3(1,2) R3(1,3) R3(2,1) R3(2,2) R3(2,3) R3(3,1) R3(3,2) R3(3,3)];
%
i = [i1, i2, i3];
j = [j1, j2, j3];
v = [v1, v2, v3];
R = sparse(i, j, v, 18, 18);
