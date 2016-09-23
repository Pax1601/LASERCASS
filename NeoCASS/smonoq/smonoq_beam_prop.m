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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L.Cavagna        Creation
%     080101      1.0     L.Riccobene      Creation
%
%*******************************************************************************
%
%   function [CG, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop
%                                                    (NODE, AREA, BETA, T, G)
%
%   DESCRIPTION: Plot beam model struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE        DESCRIPTION
%                NODE           real(array) stringers coorindates [Nx2]
%                AREA           real(array) stringers area
%        
%        OUTPUT: NAME           TYPE        DESCRIPTION
%                CG             real(array) section CG
%                JX,Jy          real        section inertias in principal axes
%                Sx, Sy         real(array) stringers static moments in pr. axis
%                R              real(array) Rotation matrix for pr. axes (2x2)
%    REFERENCES:
%
%*******************************************************************************

function [CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref)
%
SC = zeros(1,2);
NODE = NODE(:, 1:2);
GAx = 0.0; GAy = 0.0; GJ= 0.0;
% determine section properties
[CG, Jx, Jy, Jxx, Jyy, Jxy, Sx, Sy, R] = sec_prop(NODE, AREA);
% assembly stiffness matrix
[K, L, OMEGA] = stiff_section(CG, NODE, BETA, T, G);
%
Kgg = K(1:end-1, 1:end-1);
Ktg = K(end, 1:end-1);
% determine X shear center
Q = load_assembly(NODE, Sx, Sy, Jx, Jy, 0, 1, 0);
ndof = size(Q, 1);
g = Kgg\Q(1:end-1);
%sol = K\Q
%tetap = sol(end);
%fluxes = fluxes_calc(BETA, T, G, L, OMEGA, sol, tetap)
%
SC(1) = Ktg * g;
% determine Y shear center
Q = load_assembly(NODE, Sx, Sy, Jx, Jy, 1, 0, 0);
g = Kgg\Q(1:end-1);
SC(2) = Ktg * g;
% transform shear center into section axes
SC =  CG + (R' * SC')';
% apply unitary torque
Q = zeros(ndof,1); Q(ndof, 1) = 1.0;
sol = K\Q;
% get differential section rotation
tetap = sol(end);
sol(2:end) = sol(1:end-1);
% enforce first stringer warp to zero
sol(1) = 0.0;
% determine panel fluxes
fluxes = fluxes_calc(BETA, T, G, L, OMEGA, sol, tetap);
GJ = GJ_calc(fluxes, G, T, L);
%
c = R*(SC-CG)';
%
% determine shear stiffness along x axis
Tx = 1.0;
Ty = 0.0;
Mz = -c(2) * Tx;
Q = load_assembly(NODE, Sx, Sy, Jx, Jy, Tx, Ty, Mz);
sol = K\Q;
tetap = sol(end);
sol(2:end) = sol(1:end-1);
sol(1) = 0.0;
fluxes = fluxes_calc(BETA, T, G, L, OMEGA, sol, tetap);
GAx = GJ_calc(fluxes, G, T, L);
% determine shear stiffness along y axis
Tx = 0.0;
Ty = 1.0;
Mz = c(1) * Ty;
Q = load_assembly(NODE, Sx, Sy, Jx, Jy, Tx, Ty, Mz);
sol = K\Q;
tetap = sol(end);
sol(2:end) = sol(1:end-1);
sol(1) = 0.0;
fluxes = fluxes_calc(BETA, T, G, L, OMEGA, sol, tetap);
GAy = GJ_calc(fluxes, G, T, L);
A = sum(AREA);

Jt = GJ / Gref;
K1 = GAx / (Gref * A);
K2 = GAy / (Gref * A);

end
%*******************************************************************************
function GJ = GJ_calc(fluxes, G, T, L)
  % Calculate 1/GJ then invert
  i_GJ = fluxes'*(diag(L./(G.*T)))*fluxes;
  GJ = 1.0/i_GJ;
end
