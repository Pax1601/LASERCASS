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




a=1;
b=a;

NODE= [a b 0; 0 b 0; 0 0 0; a 0 0];

BETA = [1 2; 2 3; 3 4];
G = ones(3,1);
T = ones(3,1);
Tx = 0;
Ty = 0;
M = 1;

AREA = ones(4,1);

%[fluxes, tetap] = solve_mono(NODE, AREA, BETA, T, G, Tx, Ty, M)
[fluxes, tetap, CG, SC, Jx, Jy, R] = solve_mono(NODE, AREA, BETA, T, G, Tx, Ty, M);
