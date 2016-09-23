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
%     251111      1.0     L. Cavagna       Creation
%                         
%*******************************************************************************
%
% function guess
%
%   DESCRIPTION: Run GUESS Module from structural sizing to stick model creation
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                X0             array 3x1  current solution
%                data           struct     internal parameters
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                CIN,CEQ        array      constraint value for < and =
%
%    REFERENCES:
%
%*******************************************************************************

function [CIN, CEQ] = cstr_strut_8(X, data)
%
CIN = [];
CEQ = [];
%
Kb = 0.5; % consider colum as fixed at both ends
%
ts = X(1);
% geo
R = data.geo.R;
r = R - ts;
L = data.geo.L;
% material
E = data.param.E;
smax = data.param.smax;
% loads
N  = data.load.N;
Tx = data.load.Tx;
Ty = data.load.Ty;
Mx = data.load.Mx;
Mt = data.load.Mz;
CINDEX = data.param.Cindex;
%
A = pi * (R^2 - r^2);
J = 0.25 * pi * (R^4 - r^4);
Jt = 2 * J;
%
tau = Mt * R / Jt + 2 * Tx / A + 2 * Ty / A;
sigma = Mx * R / J + abs(N) / A;
% 
instglob = -1;
P_cr = pi * E * J / (Kb * L)^2;
%
if data.load.N<0
  instglob = 1 - (P_cr / abs(N));  
end
%
sigma_vm = sqrt(sigma^2 + 3*tau^2);
SVM = 1 - smax/sigma_vm;
%
%-------------------------------------------------------------------------------
%
% Store constraints
%
CIN(1) = SVM;
CIN(2) = instglob;
%
if (~isempty(CINDEX))
  CIN = CIN(CINDEX);
end
