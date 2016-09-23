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

function [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz, varargin)

% -->c: chord at wing-fuselage link
% -->b: distance of kink1 from fuselage
% -->T: taper ration CK1/c
% -->TW: root_incidence & kink1_incidence (rad)
% -->SW: sweep angle at QC
% -->dihed: dihedral angle
% --> ox, oy, oz LE point at wing-fuselge link
% --> alpha: fore_spar & aft_spar (adimensional)
% --> REFx, REFy, REFz middle point (structural) at wing-fuselge link
% --> varargin: if present, contains the pdcylin struct, used to apply a
% defined deformation
lem(1) =  0.25*c;
lem(2) =  0.25*T*c;
lem(3) = -0.75*T*c;
lem(4) = -0.75*c;
%
DX = [(1-cos(TW(1,1)))*cos(SW) (1-cos(TW(2,1)))*cos(SW)...
      (1-cos(TW(2,1)))*cos(SW) (1-cos(TW(1,1)))*cos(SW)].*lem;
%
DY = -[sin(TW(1,1))*sin(dihed)*cos(SW) sin(TW(2,1))*sin(dihed)*cos(SW)...
       sin(TW(2,1))*sin(dihed)*cos(SW) sin(TW(1,1))*sin(dihed)*cos(SW)].*lem;
%
DZ = [sin(TW(1,1))*cos(dihed) sin(TW(2,1))*cos(dihed)...
      sin(TW(2,1))*cos(dihed) sin(TW(1,1))*cos(dihed)].*lem;

% Panel corners
wingx = [0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c] + ox + DX;
wingy = [0 b*cos(dihed) b*cos(dihed) 0] + oy + DY;
wingz = [0 b*sin(dihed) b*sin(dihed) 0] + oz + DZ;


% QC line
qcx = [wingx(1,1) + (wingx(1,4)-wingx(1,1))/4, wingx(1,2) + (wingx(1,3)-wingx(1,2))/4];
qcy = [wingy(1,1) + (wingy(1,4)-wingy(1,1))/4, wingy(1,2) + (wingy(1,3)-wingy(1,2))/4];
qcz = [wingz(1,1) + (wingz(1,4)-wingz(1,1))/4, wingz(1,2) + (wingz(1,3)-wingz(1,2))/4];

% Given line
c2x = [wingx(1,1) + (wingx(1,4)-wingx(1,1))*alpha(1,1), wingx(1,2) + (wingx(1,3)-wingx(1,2))*alpha(2,1)];
c2y = [wingy(1,1) + (wingy(1,4)-wingy(1,1))*alpha(1,1), wingy(1,2) + (wingy(1,3)-wingy(1,2))*alpha(2,1)];
c2z = [wingz(1,1) + (wingz(1,4)-wingz(1,1))*alpha(1,1), wingz(1,2) + (wingz(1,3)-wingz(1,2))*alpha(2,1)];

%**************************************************************************
% Calculate panel corners without incidence angle
DX = [(1-cos(0))*cos(SW) (1-cos(0))*cos(SW)...
      (1-cos(0))*cos(SW) (1-cos(0))*cos(SW)].*lem;
%
DY = -[sin(0)*sin(dihed)*cos(SW) sin(0)*sin(dihed)*cos(SW)...
       sin(0)*sin(dihed)*cos(SW) sin(0)*sin(dihed)*cos(SW)].*lem;
%
DZ = [sin(0)*cos(dihed) sin(0)*cos(dihed)...
      sin(0)*cos(dihed) sin(0)*cos(dihed)].*lem;
% Panel corners
panex = [0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c] + ox + DX;
paney = [0 b*cos(dihed) b*cos(dihed) 0] + oy + DY;
panez = [0 b*sin(dihed) b*sin(dihed) 0] + oz + DZ;
%**************************************************************************

% Move points
dx = REFx - c2x(1);
dy = REFy - c2y(1);
dz = REFz - c2z(1);
%
wingx = wingx + dx;
wingy = wingy + dy;
wingz = wingz + dz;
%
qcx = qcx + dx;
qcy = qcy + dy;
qcz = qcz + dz;
%
c2x = c2x + dx;
c2y = c2y + dy;
c2z = c2z + dz;
%
panex = panex + dx;
paney = paney + dy;
panez = panez + dz;


% Save output
wing = [wingx; wingy; wingz];
qc = [qcx; qcy; qcz];
c2 = [c2x; c2y; c2z];
pane = [panex; paney; panez];

