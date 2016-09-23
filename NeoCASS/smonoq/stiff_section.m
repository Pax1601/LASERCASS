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
%   function K = stiff_section(REF_POINT, NODE, BETA, T, G)
%
%   DESCRIPTION: Plot beam model struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function [KR, L, OMEGA] = stiff_section(REF_POINT, NODE, BETA, T, G)

np = size(BETA, 1);
L = zeros(np, 1);
OMEGA = zeros(np, 1);

ns = size(NODE, 1);
ndof = ns +1;

COORD = zeros(2,3);
i = [];
j = [];
v = [];
K = [];

for n = 1:np

  N1 = BETA(n, 1);
  N2 = BETA(n, 2);
  COORD = [NODE(N1, :); NODE(N2, :)];
  [Kp, L(n), OMEGA(n)] = stiff_panel(T(n), G(n), COORD, REF_POINT);  
  DOF = [N1, N2, ndof];
  i = [i, repmat(DOF(1), 1,3), repmat(DOF(2), 1,3), repmat(DOF(3), 1,3)];
  j = [j, repmat(DOF, 1, 3)];
  v = [v, Kp(1,:), Kp(2,:), Kp(3,:)];

end
% constrain first stringer by default
K = sparse(i, j, v, ndof, ndof);
KR = K(2:end, 2:end);

end

function [Kp, l, omega2_o] = stiff_panel(t, G, xy, xo)

% Panel length
l = norm(xy(2, :)-xy(1, :));

% Vectors w.r.t. xo pole
v = xy - repmat(xo, 2, 1);

% Area (2*omega) w.r.t. xo pole
omega2_o = -((v(1,1)*v(2,2) - v(1,2)*v(2,1))) /2.0;

Kp = ((G*t)/l)*[        1,       -1,  -2*omega2_o;
                       -1,        1,   2*omega2_o;
                        -2*omega2_o, 2*omega2_o, 4*omega2_o^2];

end
