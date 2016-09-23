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
%   function [CG, Jx, Jy, Sx, Sy, R] = sec_prop(NODE, AREA)
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

function [CG, Jx, Jy, Jxx, Jyy, Jxy, Sx, Sy, R] = sec_prop(NODE, AREA)

ns = size(NODE, 1);
Q = zeros(ns, 1);
Sx = zeros(ns, 1);
Sy = zeros(ns, 1);
R = eye(2);
CG = zeros(1,2);

xl = NODE(:,1);
yl = NODE(:,2);

Sx = AREA .* yl; Sy = AREA .* xl;
M_TOT = sum(AREA);

xcg = sum(Sy) / M_TOT;
ycg = sum(Sx) / M_TOT;

xl = NODE(:,1) - xcg;
yl = NODE(:,2) - ycg;
Jxx = sum(AREA.*yl.*yl); Jyy = sum(AREA.*xl.*xl); Jxy = sum(AREA.*xl.*yl); 

if (Jxy ~= 0)
  alpha = 0.5 * atan(2*Jxy / (Jyy - Jxx));
  R = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];
  COORD = [xl'; yl'];
  COORD = R * COORD;
  xl = COORD(1,:)';
  yl = COORD(2,:)';
end

Sx = AREA .* yl; Sy = AREA .* xl;
Jx = sum(Sx.*yl); Jy = sum(Sy.*xl); 

CG = [xcg, ycg];

end
