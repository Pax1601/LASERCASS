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
% Load a NASTRAN file and create a beam struct
% Assembly rhs vector with nodal forces and moments specified by the user

function A = set_A_mat(COLLOC, NODE)
% full assembly
%A = -eye(18,12);

%A(7:9,1:3) = eye(3);
%A(10:12,4:6) = eye(3);
%A(13:15,7:9) = eye(3);
%A(16:18,10:12) = eye(3);

%A(4:6, 1:3) = crossm(COLLOC(1,:) - NODE(1,:));
%A(10:12, 1:3) = -crossm(COLLOC(1,:) - NODE(2,:));
%A(10:12, 7:9) = crossm(COLLOC(2,:) - NODE(2,:));
%A(16:18, 7:9) = -crossm(COLLOC(2,:) - NODE(3,:));

i = [  1  7  2  8  3  9  4 10  5 11  6 12  7 13  8 14  9 15 10 16 11 17 12 18];
j = [  1  1  2  2  3  3  4  4  5  5  6  6  7  7  8  8  9  9 10 10 11 11 12 12];
v = [ -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1 -1  1];
% COLLOC 1 - NODE 1
i1 = [4 4 5 5 6 6];
j1 = [2 3 1 3 1 2];
p = COLLOC(1,:) - NODE(1,:);
v1 = [p(3) -p(2) -p(3) p(1) p(2) -p(1)];

% COLLOC 1 - NODE 2
i2 = [10 10 11 11 12 12];
j2 = [ 2  3  1  3  1  2];
p = COLLOC(1,:) - NODE(2,:);
v2 = [-p(3) p(2) p(3) -p(1) -p(2) p(1)];

% COLLOC 2 - NODE 2
i3 = [10 10 11 11 12 12];
j3 = [ 8  9  7  9  7  8];
p = COLLOC(2,:) - NODE(2,:);
v3 = [p(3) -p(2) -p(3) p(1) p(2) -p(1)];

% COLLOC 2 - NODE 3
i4 = [16 16 17 17 18 18];
j4 = [ 8  9  7  9  7  8];
p = COLLOC(2,:) - NODE(3,:);
v4 = [-p(3) p(2) p(3) -p(1) -p(2) p(1)];
% assembly sparse matrix
i = [i , i1, i2 ,i3 ,i4];
j = [j, j1 ,j2, j3, j4];
v = [v, v1, v2 ,v3 ,v4];
A = sparse(i, j, v, 18, 12);
