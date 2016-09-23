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
function str = Prop_Sec_Vert(fid, pdcylin, geo, str, stick)

%--------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------
str.vtail.m  = [];   % Mass distribution                       [kg], vector
str.vtail.I1 = [];   % Second moment of inertia                [m4], vector
str.vtail.I2 = [];   % Second moment of inertia                [m4], vector
str.vtail.J  = [];   % Torsional constant                      [m4], vector
str.vtail.K1 = [];   % Area factor for shear                   [??], vector
str.vtail.K2 = [];   % Area factor for shear                   [??], vector

if pdcylin.vtail.kcon <= 6

  % Number of webs
  nwebs = str.vtail.nrW;

    % VT features no carry-through
    nct = 0;
    
    % Compute structural areas at BAR ends (BAR number + 1)
    As_end = 2.*(geo.vtail.Zs(nct+1:end) + geo.vtail.tbs(nct+1:end)).*str.vtail.tC(nct+1:end) +...
        str.vtail.nrW(nct+1:end).*str.vtail.tW(nct+1:end).*geo.vtail.tbs(nct+1:end);
    
    % Compute structural areas at mid-point of each element (BAR number)
    As = meancouple(As_end);
    
    % Compute box structural mass of each element (BAR number)
    str.vtail.mbox = diff(geo.vtail.y_nodes_thick).*As.*pdcylin.vtail.dsw;
    
    % Carry-through mass coeherently set to zero
    str.vtail.mc = 0;
    
    % Second moment of inertias (BAR number + 1 -> sections number)
    % **I1 = Ixx
    str.vtail.I1 = 2*geo.vtail.Zs.*str.vtail.tC.*(geo.vtail.tbs./2).^2 + 2*1/12.*str.vtail.tC.*geo.vtail.tbs.^3 + ...
        nwebs.*1/12.*str.vtail.tW.*geo.vtail.tbs.^3;

    % **I2 = Iyy
    str.vtail.I2 = 2*1/12.*str.vtail.tC.*geo.vtail.Zs.^3 + 2*geo.vtail.tbs.*str.vtail.tC.*(geo.vtail.Zs./2).^2;
    
    % Generate a distance vector for each web with respect section's
    % center, then cut 1st and last webs already accounted for and compute
    % inertia contribution by parallel axis theorem
    for i = 1:length(geo.vtail.y)
        X = linspace(-geo.vtail.Zs(i)/2, geo.vtail.Zs(i), nwebs(i));
        str.vtail.I2(i) = str.vtail.I2(i) + geo.vtail.tbs(i)*str.vtail.tW(i)*sum(X.^2);
    end
    
    % Torsional constant
    fprintf(fid, '\n\t\tVertical tail torsional constant, active option: ');
    switch pdcylin.ibredt.vtail
        
        case 1
            
            % Bredt analitical formula
            fprintf(fid, 'Bredt formula.');
            
            str.vtail.J  = 4.*(geo.vtail.Zs.*geo.vtail.tbs).^2./(2.*(geo.vtail.Zs+geo.vtail.tbs)./str.vtail.tC);
            str.vtail.K1 = zeros(geo.vtail.leny, 1);
            str.vtail.K2 = zeros(geo.vtail.leny, 1);
            
        otherwise
            
            if ~isequal(pdcylin.optimization_smonoq, 1)
                
                % Monocoque method
                fprintf(fid, 'Monocoque method.');
                
                str.vtail.K1 = zeros(geo.vtail.leny, 1);
                str.vtail.K2 = zeros(geo.vtail.leny, 1);
                str.vtail.J  = zeros(geo.vtail.leny, 1);
                for i = 1:geo.vtail.leny
                    Ixx = str.vtail.I1(i);
                    str = smonoq_setup_vtail(i, nwebs(i), Ixx, geo, str);
                end
            end
            
    end
    
else
    
    switch pdcylin.vtail.kcon
        
        case {9}

            % Check on carry-through existence
            nct = 0;
            [As_end, I1, I2, J, K1, K2] = Prop_Sec_9(nct, geo.vtail.Zs, geo.vtail.tbs, str.vtail.skin, str.vtail.web);
            % hor. plane bend
            str.vtail.I1 = I1;
            % vert. plane bend
            str.vtail.I2 = I2;
            % torsion
            str.vtail.J  = J;
            str.vtail.K1 = K1;
            str.vtail.K2 = K2;
            % Compute structural areas at mid-point of each element (BAR
            % number)
            As = meancouple(As_end);
            % Compute box structural mass of each element (BAR number, only cantilever part)
            str.vtail.mbox = pdcylin.vtail.dsw.*diff(geo.vtail.y_nodes_thick).*As;
            
    end
    
end

% Export half-model
if isequal(stick.model.symmXZ, 0)
    str.vtail.m  = str.vtail.m ./2;
    str.vtail.I1 = str.vtail.I1./2;
    str.vtail.I2 = str.vtail.I2./2;
    str.vtail.J  = str.vtail.J ./2;
    str.vtail.K1 = str.vtail.K1./2;
    str.vtail.K2 = str.vtail.K2./2;
end


end % end of Prop_Sec_Vert.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
function str = smonoq_setup_vtail(indsec, nwebs, Ixx, geo, str)
%
% Monocoque method setup for HT call
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
area = Ixx/(nstrs*(geo.vtail.tbs(indsec)/2)^2);

% X and Y coordinates for the upper nodes
X = (geo.vtail.Zs(indsec)/2 : -geo.vtail.Zs(indsec)/(nwebs-1) : -geo.vtail.Zs(indsec)/2)';
Y = geo.vtail.tbs(indsec)/2*ones(nwebs, 1);

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
T(1:nstrs) = str.vtail.tC(indsec);
T(nstrs+1:end) = str.vtail.tW(indsec);

% Setup G and Gref
G = ones(npans,1);
Gref = 1;

%--------------------------------------------------------------------------
% Monocoque method
[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Save output
str.vtail.K1(indsec, 1) = GAy/(A*Gref);
str.vtail.K2(indsec, 1) = GAx/(A*Gref);
str.vtail.J(indsec, 1)  = GJ/Gref;
%--------------------------------------------------------------------------

% Output error
if abs(CG(1)) > 1e-8 || abs(CG(2)) > 1e-8 || abs(SC(1)) > 1e-8 || abs(SC(2)) > 1e-8
    fprintf(fid, '\n      +-----------------------------------------------------------------');
    fprintf(fid, '\n      |Center of Gravity (CG) and Shear Centre not at centre of section:');
    fprintf(fid, '\n      |CGx = %g, CGy = %g; SCx = %g, SCy = %g', CG(1), CG(2), SC(1), SC(2));
    fprintf(fid, '\n      +-----------------------------------------------------------------');
end


end % end of smonoq_setup_vtail.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------
