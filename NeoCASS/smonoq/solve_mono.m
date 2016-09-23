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


function [fluxes, tetap, CG, SC, Jx, Jy, R] = solve_mono(NODE, AREA, BETA, T, G, Tx, Ty, Mz)

SC = zeros(1,2);
NODE = NODE(:,1:2);

% determine section properties
[CG, Jx, Jy, Jxx, Jyy, Jxy, Sx, Sy, R] = sec_prop(NODE, AREA);
% transform loads into main principal axes
LOAD = R * [Tx; Ty];
% assembly system rhs
Q = load_assembly(NODE, Sx, Sy, Jx, Jy, LOAD(1), LOAD(2), Mz);
% assembly stiffness matrix
[K, L, OMEGA] = stiff_section(CG, NODE, BETA, T, G);
% solve system and determine warp
sol = K\Q;
% get differential section rotation
tetap = sol(end);
sol(2:end) = sol(1:end-1);
% enforce first stringer warp to zero
sol(1) = 0.0;
% determine panel fluxes
fluxes = fluxes_calc(BETA, T, G, L, OMEGA, sol, tetap);
%
Kgg = K(1:end-1, 1:end-1);
Ktg = K(end, 1:end-1);
% determine X shear center
Q = (1/Jx) * Sx(2:end);
g = Kgg\Q;
SC(1) = Ktg * g;
% determine Y shear center
Q = (1/Jy) * Sy(2:end);
g = Kgg\Q;
SC(2) = Ktg * g;
% transform shear center into section axes
SC = CG + (R' * SC')';

end
