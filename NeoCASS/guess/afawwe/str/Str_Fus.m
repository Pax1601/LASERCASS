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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Structural analysis modulus applied to fuselage.
% 
% Called by:    AFaWWE.m
%
% Calls:        Str_Fus_BuckCrit.m, Str_Fus_ComprYield.m, Str_Fus.UltStress.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%*******************************************************************************
function str = Str_Fus(pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure: str.fus
%--------------------------------------------------------------------------------------------------
str.fus.tbar_F  = []; % equivalent isotropic thick for shell                         [m] , vector
str.fus.tbar_S  = []; % equivalent isotropic thick for frame                         [m] , vector
str.fus.tbar_SC = []; % shell thick due to compressive yield strength                [m] , vector
str.fus.tbar_ST = []; % shell thick due to ultimate tensile strength                 [m] , vector
str.fus.tbar_SG = []; % shell thick due to minimum gage                              [m] , vector
str.fus.tbar_SB = []; % shell thick due to buckling                                  [m] , vector
str.fus.tbar_B  = []; % total thickness of the fuselage structure                    [m] , vector
str.fus.tbar_Af = []; % Frame area as product of frame spacing and smeared thickness [m^2], vector
str.fus.d       = []; % frame distance                                               [m] , vector
str.fus.WI      = []; % ideal fuselage structural weight                             [kg], vector

%--------------------------------------------------------------------------------------------------
% 4 failure criteria to compute equivalent isotropic thickness for shell tS
% and frames tF along fuselage lenght, based on: 1. compressive yield
% strength; 2. ultimate tensile strength; 3. minimum gage; 4. buckling.
%--------------------------------------------------------------------------------------------------
% 1. compressive yield strength
tbar_SC = Str_Fus_ComprYield(pdcylin, aircraft, geo, loads, str);
str.fus.tbar_SC = tbar_SC;

% 2. ultimate tensile strength
tbar_ST = Str_Fus_UltStress(pdcylin, aircraft, geo, loads, str);
str.fus.tbar_ST = tbar_ST;

% 3. minimum gage
tbar_SG = Str_Fus_MinGage(pdcylin, aircraft, geo, loads, str);
str.fus.tbar_SG = tbar_SG;

% 4. determine optimum buckling smeared thickness for skin and frames
switch(pdcylin.fus.kcon)
    
    % FRAMED BEAM
    case {1,2,3,4,5}
        
        [tbar_SB, tbar_FB, d] = Str_Fus_BuckCrit_Framed(pdcylin, aircraft, geo, loads, str);
        
        % Determine new section frame spacing greater than optimum value to
        % reduce frame weight since skin is not buckling critical 
        [tbar_S, index] = max( [tbar_SC, tbar_ST, tbar_SG, tbar_SB], [], 2);
        nbc = intersect(find(index ~= 4), find(loads.fus.Nxc/max(loads.fus.Nxc)>1e-6));
        if ~isempty(nbc)
            for i=1:length(nbc)
                k = nbc(i);
                % change frame spacing
                d(k) = pdcylin.fus.es * geo.fus.epsilon * tbar_S(k)^2/loads.fus.Nxc(k);
                % change frame smeared tickness
                tbar_FB(k) = (2 * geo.fus.r(k)^2) * sqrt( pi * pdcylin.wing.cf * loads.fus.Nxc(k)/...
                    (pdcylin.fus.ef * pdcylin.fus.ckf * d(k)^3));
            end
        end
        str.fus.tbar_SB = tbar_SB;
        str.fus.tbar_F  = tbar_FB;
        str.fus.d       = d;
        str.fus.tbar_S  = tbar_S;
        str.fus.tbar_Af = str.fus.d .* str.fus.tbar_F;
        
    % UNFRAMED BEAM
    case {6,7}
        
        tbar_SB = Str_Fus_BuckCrit_Unframed(pdcylin, aircraft, geo, loads, str);
        
        % Determine maximum skin thickness
        str.fus.tbar_S  = max( [tbar_SC, tbar_ST, tbar_SG, tbar_SB], [], 2);
        str.fus.tbar_SB = tbar_SB;
        str.fus.tbar_F  = zeros(size(tbar_SB));
        str.fus.d       = str.fus.tbar_F;
        
    otherwise
        error('Unknown fuselage kcon value.');
        %
end
%
str.fus.tbar_B = str.fus.tbar_S + str.fus.tbar_F;

% IDEAL FUSELAGE STRUCTURAL WEIGHT
rm_av      = meancouple(geo.fus.r);
tbar_S_av  = meancouple(str.fus.tbar_S);
tbar_F_av  = meancouple(str.fus.tbar_F);

% Weight computed on mid-point of each element and using variable dx (the
% fuselage is not always a straight line)
str.fus.WI = 2*pi.*rm_av.*(pdcylin.fus.ds .*tbar_S_av + pdcylin.fus.df .*tbar_F_av).*diff(geo.fus.x_nodes_thick);
