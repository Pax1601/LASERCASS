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
% Determine stifness properties for single cell semimonocoque wingbox
% CARD: PBARSM1 defines the simplest semimonocoque wing box
%       The wing box is assumed to be by-simmetric
function PBAR = smonoq1_stiff(fid, PBAR)
%
%            tskin
%       -----------------                ^ y axis
%  tweb |     Rec 2     |                |
%       |               |                |
% Rec 3 |               | Rec 1          |
%       |               |                ------> z axis
%       |               |
%       -----------------
%             Rec 4
  sm_index = find(PBAR.Type == 10); % single cell monocoque
  
  if (~isempty(sm_index))

	    fprintf(fid, '\n\t - Calculating GUESS bar semimonocoque stiffness...');
      for n=1:length(sm_index)

        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        PBAR.Kshear(i, 1) = 0; % unitary G given
        PBAR.Kshear(i, 2) = 0; % unitary G given
%
        Acorner = PBAR.Section(SI).data(1); % bending material
        tskin = PBAR.Section(SI).data(2); % panel thickness
        chord = PBAR.Section(SI).data(3); % wing box chord
        h = PBAR.Section(SI).data(4);     % wing box height
%
        PBAR.A(i)   = (tskin * 2 *(chord+h)) + 4*Acorner;

        PBAR.I(i,1) = 2 * (1/12 * chord * (tskin)^3 + (tskin)*chord*(h/2-tskin/2)^2) + 4*Acorner*(h/2)^2 + ...
                      2 * (1/12 * tskin * h^3);  
        PBAR.I(i,2) = 2 * (1/12 * ((tskin) * chord^3) + 4*Acorner*(chord/2)^2) + 2*(1/12*h*(tskin)^3 + tskin*h*(chord/2-tskin/2)^2);

        PBAR.I(i,3) = 0;
        PBAR.J(i)   = (2*tskin*(chord*h)^2) / (h+chord); % Bredt formula

      end
  
  	  fprintf(fid, 'done.');

  end

end
