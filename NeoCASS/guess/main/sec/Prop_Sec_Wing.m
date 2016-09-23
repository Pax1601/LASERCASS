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
% Calls:
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080723      1.0     A. Da Ronch      Creation
%     091119      1.3.9   L. Travaglini    Modification
%
% Cleaned by Travaglini
%*******************************************************************************
function str = Prop_Sec_Wing(fid, pdcylin, geo, str, stick)

%--------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------
str.wing.m  = [];   % Mass distribution                       [kg], vector
str.wing.I1 = [];   % Second moment of inertia                [m4], vector
str.wing.I2 = [];   % Second moment of inertia                [m4], vector
str.wing.J  = [];   % Torsional constant                      [m4], vector
str.wing.K1 = [];   % Area factor for shear                   [??], vector
str.wing.K2 = [];   % Area factor for shear                   [??], vector

% Number of webs

if pdcylin.wing.kcon <=6
    
    nwebs = str.wing.nrW;
    % Check on carry-through existence
    nct = 0;
    if isequal(pdcylin.stick.model.fuse, 1)
        nct   = pdcylin.stick.nwing_carryth;
        nwebs = [ones(nct, 1)*nwebs(1); nwebs];
    end
    
    % Compute structural areas at BAR ends (BAR number + 1, only cantilever part)
    As_end = 2.*(geo.wing.Zs(nct+1:end)+ geo.wing.tbs(nct+1:end)).*str.wing.tC(nct+1:end)+...
        str.wing.nrW.*str.wing.tW(nct+1:end).*geo.wing.tbs(nct+1:end);
    
    % Compute structural areas at mid-point of each element (BAR
    % number)
    As = meancouple(As_end);
    
    % Compute box structural mass of each element (BAR number, only cantilever part)
    str.wing.mbox = pdcylin.wing.dsw.*diff(geo.wing.y_nodes_thick(nct+1:end)).*As;
    
    % Compute carry-through structural mass (if present)
    if isequal(pdcylin.stick.model.fuse, 1)
        str.wing.mc = stick.wing.Lbeam(1:nct).*( 2.*(geo.wing.Zs(1)+geo.wing.tbs(1)).*str.wing.tC(1) +...
            str.wing.nrW(1).*str.wing.tW(1).*geo.wing.tbs(1)).*pdcylin.wing.dsw;
        m = [str.wing.mc; str.wing.mbox];
        str.wing.m = m;
    else
        str.wing.m = str.wing.mbox;
    end
    
    % Second moment of inertias
    % **I1 = Ixx
    str.wing.I1 = 2*geo.wing.Zs.*str.wing.tC.*(geo.wing.tbs./2).^2 + 2*1/12.*str.wing.tC.*geo.wing.tbs.^3 + ...
        nwebs*1/12.*str.wing.tW.*geo.wing.tbs.^3;
    
    % **I2 = Iyy
    str.wing.I2 = 2*1/12.*str.wing.tC.*geo.wing.Zs.^3 + 2*geo.wing.tbs.*str.wing.tC.*(geo.wing.Zs./2).^2;
    
    %     X = zeros(length(geo.wing.y),(nwebs+1)+1) ;
    %     for i = 1:length(geo.wing.y)
    %         X(i,:) = (geo.wing.Zs(i)/2:-geo.wing.Zs(i)/(nwebs+1):-geo.wing.Zs(i)/2);
    %     end
    %
    %     % Cut 1st and last webs already accounted for
    %     Xcut = X;
    %     Xcut(:, 1)   = [];
    %     Xcut(:, end) = [];
    %
    %     % Y = geo.wing.tbs./2;
    %     for i = 1:length(geo.wing.y)
    %         str.wing.I2(i) = str.wing.I2(i) + geo.wing.tbs(i)*str.wing.tW(i)*sum(Xcut(i,:).^2);
    %     end
    
    % Generate a distance vector for each web with respect section's
    % center, then cut 1st and last webs already accounted for and compute
    % inertia contribution by parallel axis theorem
    for i = 1:length(geo.wing.y)
        X = linspace(-geo.wing.Zs(i)/2, geo.wing.Zs(i)/2, nwebs(i));
        str.wing.I2(i) = str.wing.I2(i) + geo.wing.tbs(i)*str.wing.tW(i)*sum(X(2:end-1).^2);
    end
    
    % Torsional constant
    fprintf(fid, '\n\t\tWing torsional constant, active option: ');
    switch pdcylin.ibredt.wing
        
        case 1
            
            % Bredt analitical formula
            fprintf(fid, 'Bredt formula.');
            str.wing.J  = 4.*(geo.wing.Zs.*geo.wing.tbs).^2 ./ (2.*(geo.wing.Zs+geo.wing.tbs)./str.wing.tC);
            str.wing.K1 = zeros(geo.wing.leny, 1);
            str.wing.K2 = zeros(geo.wing.leny, 1);
            
        otherwise
            
            % monocoque method
            fprintf(fid, 'Monocoque method.');
            %
            str.wing.K1 = zeros(geo.wing.leny, 1);
            str.wing.K2 = zeros(geo.wing.leny, 1);
            str.wing.J  = zeros(geo.wing.leny, 1);
            for i = 1:geo.wing.leny
                Ixx = str.wing.I1(i);
                str = smonoq_setup_wing(fid, i, nwebs, Ixx, geo, str);
            end
            
    end
    
else
    
    switch pdcylin.wing.kcon
        
        case {9}
            
            % Check on carry-through existence
            nct = 0;
            if isequal(pdcylin.stick.model.fuse, 1)
                nct = pdcylin.stick.nwing_carryth;
            end
            [As_end, I1, I2, J, K1, K2] = Prop_Sec_9(nct, geo.wing.Zs, geo.wing.tbs, str.wing.skin, str.wing.web);
            % hor. plane bend
            str.wing.I1 = I1;
            % vert. plane bend
            str.wing.I2 = I2;
            % torsion
            str.wing.J  = J;
            str.wing.K1 = K1;
            str.wing.K2 = K2;
            % Compute structural areas at mid-point of each element (BAR
            % number)
            As = meancouple(As_end);
            % Compute box structural mass of each element (BAR number, only cantilever part)
            str.wing.mbox = pdcylin.wing.dsw.*diff(geo.wing.y_nodes_thick(nct+1:end)).*As;
            % Compute carry-through structural mass (if present)
            if isequal(pdcylin.stick.model.fuse, 1)
                str.wing.mc = stick.wing.Lbeam(1:nct).*( 2.*(geo.wing.Zs(1)).*str.wing.skin.tskin(1) +...
                    2*geo.wing.tbs(1).*str.wing.web.tw(1)+...
                    2*str.wing.skin.Astr(1).*str.wing.skin.Nstr(1)).*pdcylin.wing.dsw;
                m = [str.wing.mc; str.wing.mbox];
                str.wing.m = m;
            else
                str.wing.m = str.wing.mbox;
            end

        case {10}
            
            % Check on carry-through existence
            nct = 0;
            if isequal(pdcylin.stick.model.fuse, 1)
                nct = pdcylin.stick.nwing_carryth;
            end
            pdcylin.wing = check_input_params(pdcylin.wing, geo.wing);
            [As_end, I1, I2, J, K1, K2] = Prop_Sec_10(nct, geo.wing.Zs, geo.wing.tbs, str.wing.web, str.wing.skin, pdcylin.wing.nspar);
            % hor. plane bend
            str.wing.I1 = I1;
            % vert. plane bend
            str.wing.I2 = I2;
            % torsion
            str.wing.J  = J;
            str.wing.K1 = K1;
            str.wing.K2 = K2;
            % Compute structural areas at mid-point of each element (BAR
            % number)
            As = meancouple(As_end);
            % Compute box structural mass of each element (BAR number, only cantilever part)
            str.wing.mbox = pdcylin.wing.dsw.*diff(geo.wing.y_nodes_thick(nct+1:end)).*As;
            % Compute carry-through structural mass (if present)
            if isequal(pdcylin.stick.model.fuse, 1)
                str.wing.mc = stick.wing.Lbeam(1:nct).*( 2.*(geo.wing.Zs(1)).*str.wing.skin.tskin(1) +...
                    2*geo.wing.tbs(1).*str.wing.web.tw(1)+...
                    2*str.wing.skin.Astr(1).*str.wing.skin.Nstr(1) + 2.*pdcylin.wing.nspar.*str.wing.web.Acap(1)).*pdcylin.wing.dsw;
                m = [str.wing.mc; str.wing.mbox];
                str.wing.m = m;
            else
                str.wing.m = str.wing.mbox;
            end
    end
end

%--------------------------------------------------------------------------------------------------

end % end of Prop_Sec_Wing.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
function str = smonoq_setup_wing(fid, indsec, nwebs, Ixx, geo, str)
%
% Monocoque method setup for wing call
% INPUT : NAME      NOTES
%         indsec    index of section to consider
%         nwebs     number of webs within the structural box (nwebs >= 0)

% number of panels
npans = (nwebs + 1)*2 + nwebs + 2;

% number of stringers
nstrs = (nwebs + 2)*2;

% Inizialize to correct dimension
% NODE = zeros(nstrs, 2);
% AREA = zeros(nstrs, 1);
BETA = zeros(npans, 2);
T = zeros(npans,1);
% G = zeros(npans,1);
% Gref = 0;

% area of each stringer
area = Ixx/(nstrs*(geo.wing.tbs(indsec)/2)^2);

% X and Y coordinates for the upper nodes
X = (geo.wing.Zs(indsec)/2 : -geo.wing.Zs(indsec)/(nwebs+1) : -geo.wing.Zs(indsec)/2)';
Y = geo.wing.tbs(indsec)/2*ones(nwebs+2, 1);

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
T(1:nstrs) = str.wing.tC(indsec);
T(nstrs+1:end) = str.wing.tW(indsec);

% Setup G and Gref
G = ones(npans,1);
Gref = 1;

%
% Monocoque method
[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);

%
% Save output
str.wing.K1(indsec, 1) = GAy/(A*Gref);
str.wing.K2(indsec, 1) = GAx/(A*Gref);
str.wing.J(indsec, 1)  = GJ/Gref;

% Output error
if abs(CG(1)) > 1e-8 || abs(CG(2)) > 1e-8 || abs(SC(1)) > 1e-8 || abs(SC(2)) > 1e-8
    fprintf(fid, '\n      +-----------------------------------------------------------------');
    fprintf(fid, '\n      |Center of Gravity (CG) and Shear Centre not at centre of section:');
    fprintf(fid, '\n      |CGx = %g, CGy = %g; SCx = %g, SCy = %g', CG(1), CG(2), SC(1), SC(2));
    fprintf(fid, '\n      +-----------------------------------------------------------------');
end


end % end of smonoq_setup_wing.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------
