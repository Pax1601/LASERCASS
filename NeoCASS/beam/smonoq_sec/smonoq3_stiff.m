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
% Determine stifness properties for single elliptic torque pipe
%
function PBAR = smonoq3_stiff(fid, PBAR, OPTION)
%          ___________
%         |     ^      |
%  t     |      !        |
%       |       ! b       |  
%      |        !      a   |
%       |       -------> |
%        |              |
%         |            |
%          ____________
%
  sm_index = find(PBAR.Type == 3); % single elliptic cell monocoque
  
  if (~isempty(sm_index))

	    fprintf(fid, '\n\t - Calculating elliptic bar semimonocoque stiffness...');

      for n=1:length(sm_index)

        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        b = PBAR.Section(SI).data(1);     % local y
        a = PBAR.Section(SI).data(2);     % local z
        t = PBAR.Section(SI).data(3);     % thickness

        PBAR.Kshear(i, 1) = 0; 
        PBAR.Kshear(i, 2) = 0; 
        p = pi * ( 3*(a+b) - sqrt((3*a+b)*(3*b+a)) ); % perimeter
        A = pi * a * b;
%       Mean radius
        Rm = A / p;
        Rmax = Rm + t/2;
        Rmin = Rm - t/2;
%
        PBAR.J(i)   = 4 * A^4 / (p/t); % Bredt formula
%
        PBAR.A(i)   = pi * (Rmax^2 - Rmin^2);
        PBAR.I(i,1) = (pi/4) * (Rmax^4 - Rmin^4)  
        PBAR.I(i,2) = PBAR.I(i,1);
        PBAR.I(i,3) = 0;

      end
  
  	  fprintf(fid, 'done.\n\t');

  end

end
