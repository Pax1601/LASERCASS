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

% % function [acsynt, pdcylin] = conversion(acsynt, pdcylin)
%--------------------------------------------------------------------------
% CONVERSION.M converts into SI system. Overwrite the input to get output
% 
% INPUT
%   ACSYNT, out of SI system
%   PDCYLIN, out of SI system
% 
% OUTPUT
%   acsynt, into SI system
%   pdcylin, into SI system
% 
% 
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------

global deg2rad rad2deg
global ft2m m2ft
global in2m m2in
global sqft2sqm sqm2sqft
global lb2kg kg2lb
global g
global psi2pa pa2psi
global lbin32kgm3 kgm32lbin3

%--------------------------------------------------------------------------
% ANGLE conversion
% 
%--------------------------------------------------------------------------

deg2rad = pi/180;       % deg -> rad
rad2deg = 180/pi;       % rad -> deg


%--------------------------------------------------------------------------
% LENGTH conversion
% 
%--------------------------------------------------------------------------

ft2m = 0.3048;          % ft -> m
in2m = 0.0254;          % in -> m
m2ft = 1/ft2m;          % m -> ft
m2in = 1/in2m;          % m -> in


%--------------------------------------------------------------------------
% AREA conversion
% 
%--------------------------------------------------------------------------

sqft2sqm = ft2m^2;      % ft^2 -> m^2
sqm2sqft = 1/sqft2sqm;  % m^2 -> ft^2


%--------------------------------------------------------------------------
% WEIGHT conversion (mass)
% 
%--------------------------------------------------------------------------

lb2kg = 0.4536;         % lb -> kg
kg2lb = 1/lb2kg;        % kg -> lb


%--------------------------------------------------------------------------
% PRESSURE conversion
% 
%--------------------------------------------------------------------------

psi2pa = 6.8927*1e3;    % psi -> Pa
pa2psi = 1/psi2pa;      % Pa -> psi


%--------------------------------------------------------------------------
% DENSITY conversion
% 
%--------------------------------------------------------------------------

lbin32kgm3 = lb2kg/(in2m)^3;    % lb/in^3 -> kg/m^3
kgm32lbin3 = 1/lbin32kgm3;      % kg/m^3 -> lb/in^3
