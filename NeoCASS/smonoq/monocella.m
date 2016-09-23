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
%%%%%%%%%%%%%% MONOCELLA
% index for section
indsec = 10;

% inertia respect X
Ixx = 2*geo.wing.Zs(indsec)*str.wing.tC(indsec)*(geo.wing.tbs(indsec)/2)^2 + 2*1/12*str.wing.tC(indsec)*geo.wing.tbs(indsec)^3;
area = Ixx/geo.wing.tbs(indsec)^2;

% BREDT
Jt1 = 4*(geo.wing.Zs(indsec)*geo.wing.tbs(indsec))^2 / (2*(geo.wing.Zs(indsec)+geo.wing.tbs(indsec))/str.wing.tC(indsec));

% MONOCOQUE
NODE = zeros(4, 2);
AREA = zeros(4, 1);
BETA = zeros(4, 2);
T = zeros(4,1);
G = zeros(4,1);
Gref = 0;

NODE = [ geo.wing.Zs(indsec)/2,  geo.wing.tbs(indsec)/2;...
        -geo.wing.Zs(indsec)/2,  geo.wing.tbs(indsec)/2;...
        -geo.wing.Zs(indsec)/2, -geo.wing.tbs(indsec)/2;...
         geo.wing.Zs(indsec)/2, -geo.wing.tbs(indsec)/2];
AREA = [area;...
        area;...
        area;...
        area];
BETA = [1, 2;...
        2, 3;...
        3, 4;...
        4, 1];
T = [str.wing.tC(indsec);...
     str.wing.tC(indsec);...
     str.wing.tC(indsec);...
     str.wing.tC(indsec)];
G = [1;...
     1;...
     1;...
     1];
Gref = 1;

[CG, A, Jxx, Jyy, Jxy, SC, GAx, GAy, GJ] = smonoq_beam_prop(NODE, AREA, BETA, T, G, Gref);
Jt2 = GJ/Gref;

% comparison
Ixx
area
NODE
AREA
BETA
T
G
Gref
Jt1
Jt2
CG
SC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


