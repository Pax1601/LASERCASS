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
%
%*******************************************************************************
%
% Determine stifness properties for single torque pipe with internal frames
%
function PBAR = smonoq2_fuse_stiff(fid, PBAR)
%          ___________
%         |   Rec 2     |
%  t     |              |                ^ y axis
%       |                |               |
%      | Rec 3            | Rec 1        |
%       |                |               |
%        |              |                ------> z axis
%         |            |
%          ____________
%              Rec 4
  sm_index = find(PBAR.Type == 2); % single circular cell monocoque
  
  if (~isempty(sm_index))

	    fprintf(fid, '\n\t - Calculating circular bar framed-semimonocoque stiffness...');

      for n=1:length(sm_index)
%
        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        R = PBAR.Section(SI).data(4);     % wing box radius
        ts = PBAR.Section(SI).data(1);    % skin isotropic thickness
%
        Af = PBAR.Section(SI).data(2);    % frame area
        d = PBAR.Section(SI).data(3);     % frame spacing
        tf = Af / d;
%
        rhof = PBAR.Section(SI).data(11); % frame density (kg/m3)
%
        t = ts + tf;
%
        PBAR.Kshear(i, 1) = 0;  
        PBAR.Kshear(i, 2) = 0; 
%
        PBAR.A(i)   = (2*pi) * ts * R;
        PBAR.I(i,1) = pi*t*R^3;  
        PBAR.I(i,2) = PBAR.I(i,1);
        PBAR.I(i,3) = 0;
        PBAR.J(i)   = (2*pi) * ts * R^3; % Bredt formula
        PBAR.RhoNSV(i) =  rhof * (tf / ts);
%
      end
%  
  	  fprintf(fid, 'done.');

  end

end
