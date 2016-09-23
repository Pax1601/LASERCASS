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
% CARD: PBARWB1 defines the simplest semimonocoque wing box
%       The wing box is assumed to be by-simmetric and stringers are used-defined
function PBAR = smonoqwb1_stiff(fid, PBAR, OPTION)
%
%            tskin
%       -----------------                ^ y axis
%  tweb |     Rec 2     |                |
%       |               |                |
% Rec 3 |               | Rec 1          |
%       |               |                ------> z axis
%       | stringers     |
%       -----------------
%             Rec 4
  sm_index = find(PBAR.Type == 11); % single cell monocoque
  
  if (~isempty(sm_index))

    switch(OPTION)

    case 0
    % overwrite initial loaded data 
	    fprintf(fid, '\n\t - Setting bar wing-box stiffness...');

      for n=1:length(sm_index)

        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        %
        A     = PBAR.Section(SI).data(1); % section total area
        tw    = PBAR.Section(SI).data(2); % web shear thickness
        ts    = PBAR.Section(SI).data(3); % web shear thickness
        chord = PBAR.Section(SI).data(4); % wing box chord
        h     = PBAR.Section(SI).data(5); % wing box height
        stype = PBAR.Section(SI).data(6); % stringer type
        nstr  = PBAR.Section(SI).data(7); % upper side stringer number 
        ribsp = PBAR.Section(SI).data(8); % ribs pitch
        C1    = PBAR.Section(SI).data(9); % hflange/thickness 
        C2    = PBAR.Section(SI).data(10);% vflange/thickness
        %
        % Determine stringer thickness
        %
        Astr = (A - 2 * chord * ts - 2 * h * tw) / (2*nstr);
        if (Astr<0)
          error('Inconsistent data provided in PBARWB1 %d. Negative equivalent bending material determined.', PBAR.ID(sm_index(n)));
        end
        switch(stype)
          case {1,2,3}
            t = sqrt(Astr / (2*C1+C2));
          case {4,5}
            t = sqrt(Astr / (C1+C2));
        end

        PBAR.Section(SI).data(1) = t;

      end

  	  fprintf(fid, 'done.');

%
    case 1
%
	    fprintf(fid, '\n\t - Calculating GUESS bar semimonocoque stiffness...');
      for n=1:length(sm_index)

        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        PBAR.Kshear(i, 1) = 0; % unitary G given
        PBAR.Kshear(i, 2) = 0; % unitary G given
%
        t     = PBAR.Section(SI).data(1); % section total area
        tweb  = PBAR.Section(SI).data(2); % web shear thickness
        tskin = PBAR.Section(SI).data(3); % skin shear thickness
        chord = PBAR.Section(SI).data(4); % wing box chord
        h     = PBAR.Section(SI).data(5); % wing box height
        stype = PBAR.Section(SI).data(6); % stringer type
        nstr  = PBAR.Section(SI).data(7); % upper side stringer number 
        ribsp = PBAR.Section(SI).data(8); % ribs pitch
        C1    = PBAR.Section(SI).data(9); % hflange/thickness 
        C2    = PBAR.Section(SI).data(10);% vflange/thickness
%
        step = [0:chord/nstr-1:chord] - chord/2;
%
        Astr = 0;
%
        switch(stype)
          case 1 % I stiffener
            Aloc = (2*C1+C2)*t^2;
            Astr = 2*nstr*Aloc;
            hof = C1*t;
            hvf = C2*t;
            J1 = 2 * nstr * (Astr*(h/2-t-hvf/2)^2 + (1/12*t*(hvf)^3 + 2*(1/12*hof*t^3 + hof*t*(t+hvf/2)^2)));
            J2 = 2 * nstr * (1/12*hvf*t^3 + 1/6*t*hof^3) + Astr * sum(step.^2);        

          case 5 % T stiffener
            Aloc = (C1+C2)*t^2;
            Astr = 2*nstr*Aloc;
            hof = C1*t;
            hvf = C2*t;
            J1 = 2 * nstr * (Astr*(h/2-t-hvf/2)^2 + (1/12*t*(hvf)^3 + 1/12*hof*t^3 + hof*t*(t+hvf/2)^2));
            J2 = 2 * nstr * (1/12*hvf*t^3 + 1/12*t*hof^3) + Astr * sum(step.^2);        

          otherwise
            error('Stringer type not implemented yet.');
        end

        PBAR.A(i)   = 2*(tweb * h + tskin * chord) + Astr;
        PBAR.I(i,1) = 2 * (1/12 * (tweb * h^3) + 1/12 * chord * tskin^3 + tskin*chord*(h/2-tskin/2)^2) + J1;  
        PBAR.I(i,2) = 2 * (1/12 * (tskin * chord^3) + 1/12 * h * tweb^3 + tweb*h*(chord/2-tweb/2)^2) + J2;
        PBAR.I(i,3) = 0;
        PBAR.J(i)   = (2*(chord*h)^2) / (h/tweb + chord/tskin); % Bredt formula

        PBAR.Section(SI).data(11) = J1; 
        PBAR.Section(SI).data(12) = J2;

      end
  
  	  fprintf(fid, 'done.');

    otherwise

      error('Unknown option: error in smonoq1_stiff.m function.');

    end 

  end

end
