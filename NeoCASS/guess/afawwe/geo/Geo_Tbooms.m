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
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%                 1.4     L. Travaglini    Creation
%
%*******************************************************************************
function geo = Geo_Tbooms(pdcylin, aircraft, geo)

%--------------------------------------------------------------------------------------------------
% Initialize structure
% 
geo.tbooms.bodl        = [];    % tail booms length                               [m] , scalar
geo.tbooms.WB          = [];    % tail booms weight                               [kg], scalar
geo.tbooms.Swet        = [];    % wetted surface                                  [m2], vector
geo.tbooms.R           = [];    % radius                                          [m] , scalar
geo.tbooms.dx          = [];    % step in x coordinate                            [m] , scalar
geo.tbooms.x           = [];    % domain in x coordinate                          [m] , vector
geo.tbooms.lenx        = [];    % nr of points in x direction                     [-] , scalar
% geo.tbooms.VolFus      = [];    % tail booms volume distribution in x coord       [m3], vector
% geo.tbooms.RatioVolFus = [];    % tail booms volume distribution ratio in x coord [-] , vector
% geo.tbooms.deltaVolFus = [];    % increment of volume ratio for each element      [-] , vector
geo.tbooms.A           = [];    % tail booms cross-sectional area                 [m2], scalar
geo.tbooms.P           = [];    % tail booms perimeter                            [m] , scalar
geo.tbooms.m           = [];    % tail booms structural geometry parameters       [-] , scalar
geo.tbooms.epsilon     = [];    % tail booms structural geometry parameters       [-] , scalar
geo.tbooms.Kmg         = [];    % tail booms structural geometry parameters       [-] , scalar
geo.tbooms.Kp          = [];    % tail booms structural geometry parameters       [-] , scalar
geo.tbooms.Kth         = [];    % tail booms structural geometry parameters       [-] , scalar
geo.tbooms.CAERO1.n    = [];    % vector containing,  number of elements 
geo.tbooms.n           = [];    % vector containing,  number of elements 
% 
%
geo.tbooms.x_nodes     = [];
geo.tbooms.x_nodes_1_2 = [];
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Basic calculations
%
geo.tbooms.bodl   = aircraft.Tailbooms.total_length;   
if aircraft.Tailbooms.symmetry
    geo.tbooms.WB = aircraft.weight_balance.COG(12,4,1)/2; %
else
    geo.tbooms.WB = aircraft.weight_balance.COG(12,4,1); %
end                              
geo.tbooms.x    = (aircraft.Tailbooms.x : (geo.tbooms.bodl / pdcylin.guess.tbooms) : aircraft.Tailbooms.x + geo.tbooms.bodl)';
geo.tbooms.dx   = (geo.tbooms.bodl / pdcylin.guess.tbooms)*ones(numel(geo.tbooms.x)-1, 1);
geo.tbooms.lenx = pdcylin.guess.tbooms  + 1;

geo.tbooms.n               = pdcylin.guess.tbooms;
geo.tbooms.CAERO1.n        = pdcylin.stick.ntbooms;
geo.tbooms.CAERO1.n_coarse = pdcylin.stick.ntbooms_coarse;
%
%--------------------------------------------------------------------------
% Right tailboom

% Save output
geo.tbooms.xx = geo.tbooms.x';
geo.tbooms.xx = [geo.tbooms.xx(:,1),geo.tbooms.xx(:,end)];
geo.tbooms.yy = ones(1,length(geo.tbooms.xx))*aircraft.Tailbooms.y;
geo.tbooms.zz = ones(1,length(geo.tbooms.xx))*aircraft.Tailbooms.z;

geo.tbooms.y = aircraft.Tailbooms.y;
geo.tbooms.z = aircraft.Tailbooms.z;

%--------------------------------------------------------------------------
% Define area / radius / perimeter / volume distribution 
%--------------------------------------------------------------------------
%
% Tail booms have constant section
geo.tbooms.R = aircraft.Tailbooms.diameter*0.5;
R_vec = repmat(geo.tbooms.R, numel(geo.tbooms.x), 1);

% Area enclosed in the cilinder
geo.tbooms.A  = pi*R_vec.^2;

% Structural area
geo.tbooms.As = pi*(R_vec.^2 - 0.97*R_vec.^2);

% Correspondent perimeter
geo.tbooms.P = 2*pi*R_vec;

% Wetted surface (single Tail boom)
Swet = meancouple(geo.tbooms.P).*geo.tbooms.dx;
geo.tbooms.Swet = Swet;

%--------------------------------------------------------------------------
% Tailbooms structural concept (?)
%--------------------------------------------------------------------------
%
switch pdcylin.tbooms.kcon
    case 1
        layoutf = 1;
    case 2
        layoutf = 2;
    case 3
        layoutf = 3;
    case 4
        layoutf = 4;
    case 5
        layoutf = 5;
    case 6
        layoutf = 6;
    case 7
        layoutf = 7;
end
% Load data file
fcoef = load('FuseStruPara.txt');
geo.tbooms.m       = fcoef(layoutf, 1);
geo.tbooms.epsilon = fcoef(layoutf, 2);
geo.tbooms.Kmg     = fcoef(layoutf, 3);
% accounts for the fact that not all of the shell material is available to resist for hoop stress
geo.tbooms.Kp      = fcoef(layoutf, 4);
geo.tbooms.Kth     = fcoef(layoutf, 5);