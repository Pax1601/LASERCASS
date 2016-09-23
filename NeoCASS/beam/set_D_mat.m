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

function D = set_D_mat(D1, D2)

i1 = [1 1 1 1 1 1 2 2 2 2 2 2 3 3 3 3 3 3 4 4 4 4 4 4 5 5 5 5 5 5 6 6 6 6 6 6]; % D1
j1 = [1 2 3 4 5 6 1 2 3 4 5 6 1 2 3 4 5 6 1 2 3 4 5 6 1 2 3 4 5 6 1 2 3 4 5 6];
%
i2 = i1 + 6; % D2
j2 = j1 + 6;
%
v1 = [D1(1,1) D1(1,2) D1(1,3) D1(1,4) D1(1,5) D1(1,6) D1(2,1) D1(2,2) D1(2,3) D1(2,4) D1(2,5) D1(2,6) D1(3,1) D1(3,2) D1(3,3) D1(3,4) D1(3,5) D1(3,6) ...
	  D1(4,1) D1(4,2) D1(4,3) D1(4,4) D1(4,5) D1(4,6) D1(5,1) D1(5,2) D1(5,3) D1(5,4) D1(5,5) D1(5,6) D1(6,1) D1(6,2) D1(6,3) D1(6,4) D1(6,5) D1(6,6)];	
%
v2 = [D2(1,1) D2(1,2) D2(1,3) D2(1,4) D2(1,5) D2(1,6) D2(2,1) D2(2,2) D2(2,3) D2(2,4) D2(2,5) D2(2,6) D2(3,1) D2(3,2) D2(3,3) D2(3,4) D2(3,5) D2(3,6) ...
	  D2(4,1) D2(4,2) D2(4,3) D2(4,4) D2(4,5) D2(4,6) D2(5,1) D2(5,2) D2(5,3) D2(5,4) D2(5,5) D2(5,6) D2(6,1) D2(6,2) D2(6,3) D2(6,4) D2(6,5) D2(6,6)];	
%
i = [i1, i2];
j = [j1, j2];
v = [v1, v2];
%
D = sparse(i, j, v, 12, 12);
