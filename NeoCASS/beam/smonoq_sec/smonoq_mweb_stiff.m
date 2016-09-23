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
% CARD: PBARGMW defines the multi web wing box
%       The wing box is assumed to be by-simmetric
function PBAR = smonoq_mweb_stiff(fid, PBAR)
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
  sm_index = find(PBAR.Type == 3); % single cell monocoque
  
  if (~isempty(sm_index))

	    fprintf(fid, '\n\t - Calculating GUESS multi-web bar stiffness...');
      for n=1:length(sm_index)

        i = sm_index(n); % pbar index
        SI = PBAR.SI(i);
        PBAR.Kshear(i, 1) = 0;
        PBAR.Kshear(i, 2) = 0;
        %
        % web equivalent thickness 
        tw = PBAR.Section(SI).data(1);
        ts = PBAR.Section(SI).data(2);
        %
        chord = PBAR.Section(SI).data(4); % wing box chord
        h = PBAR.Section(SI).data(5);     % wing box height
        dw = PBAR.Section(SI).data(3);
        cd = chord / dw;
        Nwmin = floor(cd) + 1;
        if (Nwmin <2)
          Nwmin=2;
          tw = ts;
          fprintf('\nWarning: number of webs for %d set to 2 %f %f!!!',SI,chord ,dw);
        end
        Nwmax = Nwmin + 1;
        % total area
        PBAR.A(i) = 2*ts*chord + cd*(tw*h); 
        % moments of inertia (supposing webs do not contribute)
        PBAR.I(i,1) = 2 * (1/12 * chord * ts^3 + ts*chord*(h/2)^2);  
        PBAR.I(i,2) = 2 * (1/12 * (ts * chord^3));
        PBAR.I(i,3) = 0;
        % Stiffness calculated using monocoque method
        % PBAR.J(i) = 4 .*(chord.*h).^2 ./ (2.*(chord+h)./ts); Bredt formula single cell
        % linear interpolation between nmin and nmax webs
        [Ks1min, Ks2min, Jtmin] = smonoq_setup_mweb(fid, h, chord, Nwmin-2, PBAR.I(i,1), ts, tw);
        [Ks1max, Ks2max, Jtmax] = smonoq_setup_mweb(fid, h, chord, Nwmax-2, PBAR.I(i,1), ts, tw);
%        PBAR.Kshear(i, 1) = (Ks1min - Ks1max)/(Nwmin - Nwmax)*dw + Ks1min;
%        PBAR.Kshear(i, 2) = (Ks2min - Ks2max)/(Nwmin - Nwmax)*dw + Ks2min;
        PBAR.J(i) =        (Jtmin - Jtmax)/(Nwmin - Nwmax)*dw + Jtmin; 

      end
  
  	  fprintf(fid, 'done.');

  end

end
%--------------------------------------------------------------------------------------------------
% Monocoque method for multi web               h   c
function [K1, K2, J] = smonoq_setup_mweb(fid, tbs, Zs, nwebs, Ixx, tC, tW)
% nwebs: number of internal webs
% number of panels
npans = (nwebs + 1)*2 + nwebs + 2;

% number of stringers
nstrs = (nwebs + 2)*2;

% Inizialize to correct dimension
NODE = zeros(nstrs, 2);
AREA = zeros(nstrs, 1);
BETA = zeros(npans, 2);
T = zeros(npans,1);
G = zeros(npans,1);
Gref = 0;

% area of each stringer
area = Ixx/(nstrs*(tbs/2)^2);

% X and Y coordinates for the upper nodes
X = [Zs/2 : -Zs/(nwebs+1) : -Zs/2]';
Y = tbs/2*ones(nwebs+2, 1);

% Setup NODE
NODE = [ X,  Y;...
        -X, -Y];
    
% Setup AREA
AREA = area*ones(nstrs, 1);

% Setup BETA
for i = 1:nstrs-1
    BETA(i,:) = [i, i+1];
end
BETA(nstrs,:) = [nstrs, 1]; % close the outer circle
for j = 1:nwebs
    BETA(nstrs+j,:) = [BETA(j+1, 1), BETA(nstrs-j, 1)]; % webs within the section
end

% Setup T
T(1:nstrs) = tC;
T(nstrs+1:end) = tW;

% Setup G and Gref
G = ones(npans,1);
Gref = 1;
%
% Monocoque method
[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);
%
% Save output
K1 = GAy/(A*Gref);
K2 = GAx/(A*Gref);
J  = GJ/Gref;

% Output error
if abs(CG(1)) > 1e-8 | abs(CG(2)) > 1e-8 | abs(SC(1)) > 1e-8 | abs(SC(2)) > 1e-8
    fprintf(fid, '\n      +-----------------------------------------------------------------');
    fprintf(fid, '\n      |Center of Gravity (CG) and Shear Centre not at centre of section:');
    fprintf(fid, '\n      |CGx = %g, CGy = %g; SCx = %g, SCy = %g', CG(1), CG(2), SC(1), SC(2));
    fprintf(fid, '\n      +-----------------------------------------------------------------');
end


end % end of smonoq_setup_wing.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------
