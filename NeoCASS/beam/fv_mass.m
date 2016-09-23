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

syms xi xi_0 xi_I L real

N = [xi*(xi - 1)/2, (1 - xi^2), xi*(xi + 1)/2];
NN = [N(1)*eye(6), N(2)*eye(6), N(3)*eye(6)];

syms m S_x S_y S_z J_xx J_xy J_xz J_yy J_yz J_zz real

S = [S_x; S_y; S_z];
J = [J_xx, J_xy, J_xz; J_xy, J_yy, J_yz; J_xz, J_yz, J_zz];

M = [m*eye(3), cross(S)'; cross(S), J];

l = [1; 0; 0];
X1 = -L/2*l;
X2 = zeros(size(l));
X3 = L/2*l;

p = X1*N(1) + X2*N(2) + X3*N(3);

U = [eye(3), zeros(3); cross(p - subs(p, xi, xi_0))', eye(3)];

F = U*M*NN*L/2;

MM = [int(subs(F, xi_0, -1), xi, -1, -xi_I); ...
      int(subs(F, xi_0, 0), xi, -xi_I, xi_I); ...
      int(subs(F, xi_0, 1), xi, xi_I, 1)];

MMM=simplify(subs(MM,{J_xy,J_xz,J_yz,S_x,S_y,S_z,xi_I},{0,0,0,0,0,0,1/sqrt(3)}))

