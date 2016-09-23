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

function I = refined_inertia_calc(cog)
%
% Warning: this is still work in progress due to the lack of y coordinates
% of some components!
%
% Use PLOTCGS resume table to compute actual inertia matrix.
% Resulting inertia matrix is computed with respect to centre of gravity at
% MTOW.
% Syntax:
%          I = refined_inertia_calc(cog)
%
%

% Recover weight and arms information
% n = 1;
% cog = aircraft.weight_balance.COG(:, :, n);
index = 1:26;
xi = cog(index, 1);
yi = cog(index, 2);
zi = cog(index, 3);
mi = cog(index, 4);

% Inertia matrix w.r.t. aircraft nose
%! Check if a factor of 2 is missing in y coordinates (2 engines, 2 fuel tanks...) 
Ixx = sum(mi.*(yi.^2 + zi.^2));
Iyy = sum(mi.*(xi.^2 + zi.^2));
Izz = sum(mi.*(xi.^2 + yi.^2));
Ixy = -sum(mi.*(xi.*yi));
Ixz = -sum(mi.*(xi.*zi));
Iyz = -sum(mi.*(yi.*zi));
Iyx = Ixy;
Izx = Ixz;
Izy = Iyz;

% Assembly matrix
I = [Ixx, Ixy, Ixz;...
    Iyx, Iyy, Iyz;...
    Izx, Izy, Izz];

% CoG at MTOW
index_cog = 27;
W = sum(cog(:, 4));
d = cog(index_cog, 1:3);

% Refer matrix to CoG
I = I - abs(W.*(crossm(d)*crossm(d)));
