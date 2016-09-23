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
%   Author: <andreadr@kth.se>
%
% Extract main mechanical properties for multi-cell wing. Bredt formula and
% semicoque method are implemented, activated by user-defined option, and
% computes the following parameters:
%   +----------------------------+
%   | m   A   I1  I2  J   K1  K2 |
%   +----------------------------+------------------+
%   | Y   Y   Y   Y   -   -   -  | current function |
%   | -   -   -   -   Y   N   N  | BREDT            |
%   | -   -   -   -   Y   Y   Y  | MONOCOQUE METHOD |
%   +----------------------------+------------------+
%
% Called by:    Prop_Sec.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080723      1.0     A. Da Ronch      Creation
%*******************************************************************************
function str = Prop_Sec_Canr(fid, pdcylin, geo, str, stick)

%--------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------
%
str.canard.m  = [];   % Mass distribution                       [kg], vector
str.canard.A  = [];   % Area distribution                       [m2], vector
str.canard.I1 = [];   % Second moment of inertia                [m4], vector
str.canard.I2 = [];   % Second moment of inertia                [m4], vector
str.canard.J  = [];   % Torsional constant                      [m4], vector
str.canard.K1 = [];   % Area factor for shear                   [??], vector
str.canard.K2 = [];   % Area factor for shear                   [??], vector


if pdcylin.canard.kcon <= 6
    
    % Number of webs
    nwebs = str.canard.nrW;
    % Check on carry-through existence
    nct = 0;
    if isequal(pdcylin.stick.model.fuse, 1)
        nct = pdcylin.stick.ncanard_carryth;
        nwebs = [ones(nct, 1)*nwebs(1); nwebs];
    end
    
    % Compute structural areas at BAR ends (BAR number + 1, only cantilever part)
    As_end = 2.*(geo.canard.Zs(nct+1:end)+ geo.canard.tbs(nct+1:end)).*str.canard.tC(nct+1:end)+...
        str.canard.nrW.*str.canard.tW(nct+1:end).*geo.canard.tbs(nct+1:end);
    
    % Compute structural areas at mid-point of each element (BAR
    % number)
    As = meancouple(As_end);
    
    % Compute box structural mass of each element (BAR number, only cantilever part)
    str.canard.mbox = pdcylin.canard.dsw.*diff(geo.canard.y_nodes_thick(nct+1:end)).*As;
    % Compute carry-through structural mass (if present)
    if isequal(pdcylin.stick.model.fuse, 1)
        str.canard.mc = stick.canr.Lbeam(1:nct).*( 2.*(geo.canard.Zs(1)+geo.canard.tbs(1)).*str.canard.tC(1) +...
            str.canard.nrW(1).*str.canard.tW(1).*geo.canard.tbs(1)).*pdcylin.canard.dsw;
        m = [str.canard.mc; str.canard.mbox];
        str.canard.m = m;
    else
        str.canard.m = str.canard.mbox;
    end
        
    % Second moment of inertias
    % **I1 = Ixx
    str.canard.I1 = 2*geo.canard.Zs.*str.canard.tC.*(geo.canard.tbs./2).^2 + 2*1/12.*str.canard.tC.*geo.canard.tbs.^3 +...
        nwebs*1/12.*str.canard.tW.*geo.canard.tbs.^3;
    
    % **I2 = Iyy
    str.canard.I2 = 2*1/12.*str.canard.tC.*geo.canard.Zs.^3 + 2*geo.canard.tbs.*str.canard.tC.*(geo.canard.Zs./2).^2;
    
    % Generate a distance vector for each web with respect section's
    % center, then cut 1st and last webs already accounted for and compute
    % inertia contribution by parallel axis theorem
    for i = 1:length(geo.canard.y)
        X = linspace(-geo.canard.Zs(i)/2, geo.canard.Zs(i)/2, nwebs(i));
        str.canard.I2(i) = str.canard.I2(i) + geo.canard.tbs(i)*str.canard.tW(i)*sum(X(2:end-1).^2);
    end
 
    % Torsional constant
    fprintf(fid, '\n\t\tCanard torsional constant, active option: ');
    switch pdcylin.ibredt.canard
        case 1
            
            % Bredt analitical formula
            fprintf(fid, 'Bredt formula.');
            %
            str.canard.J = 4.*(geo.canard.Zs.*geo.canard.tbs).^2 ./ (2.*(geo.canard.Zs+geo.canard.tbs)./str.canard.tC);
            str.canard.K1 = zeros(geo.canard.leny, 1);
            str.canard.K2 = zeros(geo.canard.leny, 1);
            
        otherwise
            
            if ~isequal(pdcylin.optimization_smonoq, 1)
                % monocoque method
                fprintf(fid, 'Monocoque method.');
                %
                str.canard.K1 = zeros(geo.canard.leny, 1);
                str.canard.K2 = zeros(geo.canard.leny, 1);
                str.canard.J  = zeros(geo.canard.leny, 1);
                for i = 1:geo.canard.leny
                    Ixx = str.canard.I1(i);
                    str = smonoq_setup_canard(fid, i, nwebs, Ixx, geo, str);
                end
            end
    end
    
else
    
    switch pdcylin.canard.kcon
            
        case {8}
            nct = 0;
            if isequal(pdcylin.stick.model.fuse, 1)
                nct = pdcylin.stick.ncanard_carryth;
            end
            [As_end, I1, I2, J, K1, K2] = Prop_Sec_8(nct, geo.canard.tbs, str.canard.skin);
            % hor. plane bend
            str.canard.I1 = I1;
            % vert. plane bend
            str.canard.I2 = I2;
            % torsion
            str.canard.J  = J;
            str.canard.K1 = K1;
            str.canard.K2 = K2;
            % Compute structural areas at mid-point of each element (BAR
            % number)
            As = meancouple(As_end);
            % Compute box structural mass of each element (BAR number, only cantilever part)
            str.canard.mbox = pdcylin.canard.dsw.*diff(geo.canard.y_nodes_thick(nct+1:end)).*As;
            % Compute carry-through structural mass (if present)
            if geo.canard.twc
                str.canard.mc = stick.canr.Lbeam(1:nct).*( As_end(1)*pdcylin.canard.dsw);
                m = [str.canard.mc; str.canard.mbox];
                str.canard.m = m;
            else
                str.canard.m = str.canard.mbox;
            end
%
        case {9}

           % Check on carry-through existence
            nct = 0;
            if isequal(pdcylin.stick.model.fuse, 1)
                nct = pdcylin.stick.ncanard_carryth;
            end
            [As_end, I1, I2, J, K1, K2] = Prop_Sec_9(nct, geo.canard.Zs, geo.canard.tbs, str.canard.skin, str.canard.web);
            % hor. plane bend
            str.canard.I1 = I1;
            % vert. plane bend
            str.canard.I2 = I2;
            % torsion
            str.canard.J  = J;
            str.canard.K1 = K1;
            str.canard.K2 = K2;
            % Compute structural areas at mid-point of each element (BAR
            % number)
            As = meancouple(As_end);
            % Compute box structural mass of each element (BAR number, only cantilever part)
            str.canard.mbox = pdcylin.canard.dsw.*diff(geo.canard.y_nodes_thick(nct+1:end)).*As;
            % Compute carry-through structural mass (if present)
            if geo.canard.twc
                str.canard.mc = stick.canr.Lbeam(1:nct).*( 2.*(geo.canard.Zs(1)).*str.canard.skin.tskin(1) +...
                    2*geo.canard.tbs(1).*str.canard.web.tw(1)+...
                    2*str.canard.skin.Astr(1).*str.canard.skin.Nstr(1)).*pdcylin.canard.dsw;
                m = [str.canard.mc; str.canard.mbox];
                str.canard.m = m;
            else
                str.canard.m = str.canard.mbox;
            end

    end
    
end

end % end of Prop_Sec_canard.m, DO NOT REMOVE

%--------------------------------------------------------------------------------------------------
function str = smonoq_setup_canard(fid, indsec, nwebs, Ixx, geo, str)
%
% Monocoque method setup for canard call
% INPUT : NAME      NOTES
%         indsec    index of section to consider
%         nwebs     number of webs within the structural box (nwebs >= 0)

% number of panels
npans = (nwebs - 1)*2 + nwebs;
% number of stringers
nstrs = nwebs*2;

% Inizialize to correct dimension
% NODE = zeros(nstrs, 2);
% AREA = zeros(nstrs, 1);
BETA = zeros(npans, 2);
T = zeros(npans,1);
% G = zeros(npans,1);
% Gref = 0;

% area of each stringer
area = Ixx/(nstrs*(geo.canard.tbs(indsec)/2)^2);

% X and Y coordinates for the upper nodes
X = (geo.canard.Zs(indsec)/2 : -geo.canard.Zs(indsec)/(nwebs-1) : -geo.canard.Zs(indsec)/2)';
Y = geo.canard.tbs(indsec)/2*ones(nwebs, 1);

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
for j = 1:nwebs-2
    BETA(nstrs+j,:) = [BETA(j+1, 1), BETA(nstrs-j, 1)]; % webs within the section
end

% Setup T
T(1:nstrs) = str.canard.tC(indsec);
T(nstrs+1:end) = str.canard.tW(indsec);

% Setup G and Gref
G = ones(npans,1);
Gref = 1;

%--------------------------------------------------------------------------
% Monocoque method
[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Save output
str.canard.K1(indsec, 1) = GAy/(A*Gref);
str.canard.K2(indsec, 1) = GAx/(A*Gref);
str.canard.J(indsec, 1)  = GJ/Gref;
%--------------------------------------------------------------------------

% Output error
if abs(CG(1)) > 1e-8 || abs(CG(2)) > 1e-8 || abs(SC(1)) > 1e-8 || abs(SC(2)) > 1e-8
    fprintf(fid, '\n      +-----------------------------------------------------------------');
    fprintf(fid, '\n      |Center of Gravity (CG) and Shear Centre not at centre of section:');
    fprintf(fid, '\n      |CGx = %g, CGy = %g; SCx = %g, SCy = %g', CG(1), CG(2), SC(1), SC(2));
    fprintf(fid, '\n      +-----------------------------------------------------------------');
end


end % end of smonoq_setup_canard.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------
