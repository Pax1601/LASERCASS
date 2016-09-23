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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% MULTICELLA
function [] = multicella(indsec, nwebs, geo, str)

% % %--------------------------------------
% % % index for section
% % indsec = 1;
% % % number of webs
% % nwebs = 10 - 2;
% % %%%%nwebs = str.wing.nrW - 2;
% % %--------------------------------------

% number of panels
npans = (nwebs + 1)*2 + nwebs + 2;

% number of stringers
nstrs = (nwebs + 2)*2;

% inertia respect X
Ixx = 2*geo.wing.Zs(indsec)*str.wing.tC(indsec)*(geo.wing.tbs(indsec)/2)^2 +...
      2*1/12*str.wing.tC(indsec)*geo.wing.tbs(indsec)^3 +...
      nwebs*1/12*str.wing.tW(indsec)*geo.wing.tbs(indsec)^3;

% area of each stringer
area = Ixx/(nstrs*(geo.wing.tbs(indsec)/2)^2);

%-------------------------------------------
% BREDT
Jt1 = 4*(geo.wing.Zs(indsec)*geo.wing.tbs(indsec))^2 / (2*(geo.wing.Zs(indsec)+geo.wing.tbs(indsec))/str.wing.tC(indsec));

%-------------------------------------------
% MONOCOQUE
NODE = zeros(nstrs, 2);
AREA = zeros(nstrs, 1);
BETA = zeros(npans, 2);
T = zeros(npans,1);
G = zeros(npans,1);
Gref = 0;

% X and Y coordinates for the upper nodes
X = [geo.wing.Zs(indsec)/2:-geo.wing.Zs(indsec)/(nwebs+1):-geo.wing.Zs(indsec)/2]';
Y = geo.wing.tbs(indsec)/2*ones(nwebs+2, 1);

%
% Setup NODE
NODE = [ X,  Y;...
        -X, -Y];
%
% Setup AREA
AREA = area*ones(nstrs, 1);
%
% Setup BETA
for i = 1:nstrs-1
    BETA(i,:) = [i, i+1];
end
BETA(nstrs,:) = [nstrs, 1];
for j = 1:nwebs
    BETA(nstrs+j,:) = [BETA(j+1, 1), BETA(nstrs-j, 1)];
end
%
% Setup T
T(1:nstrs) = str.wing.tC(indsec);
T(nstrs+1:end) = str.wing.tW(indsec);
%
% Setup G and Gref
G = 1e+9*ones(npans,1);
Gref = 1e+9;

[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);
Jt2 = GJ/Gref;

% comparison
Jxx
Ixx
Jyy
A
Jt1
Jt2
CG
SC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


