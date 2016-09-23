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
% The routine Geo_Fus.m calculates the geometric parameters used to
% define the fuselage.
% 
% 
% Called by:    AFaWWE.m
% 
% Calls:        Fus_Radius.m, Geo_Fus_ShapeT.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%     091119      1.3.9   Travaglini       Modification
%
% Modified by Travaglini, there are some small corrections about geometry
% definition
%*******************************************************************************
function [geo] = Geo_Fus(pdcylin, aircraft, geo)

%--------------------------------------------------------------------------------------------------
% Initialize structure
% 
geo.fus.bodl        = [];    % fuselage length                                 [m] , scalar
geo.fus.lengthN     = [];    % length of nose                                  [m] , scalar
geo.fus.lengthAB    = [];    % length of after body section                    [m] , scalar
geo.fus.lengthMID   = [];    % length of mid-section                           [m] , scalar
geo.fus.WB          = [];    % fuselage weight                                 [kg], scalar
geo.fus.Swet        = [];    % wetted surface                                  [m2], vector
% % % geo.fus.powmn       = [];    % power body law, nose                            [-] , scalar
% % % geo.fus.powmt       = [];    % power body law, tail                            [-] , scalar
geo.fus.R           = [];    % maximum radius in fuselage                      [m] , scalar
% % % geo.fus.VN          = [];    % volume of nose section                          [m3], scalar
% % % geo.fus.VAB         = [];    % volume of after body section                    [m3], scalar
% % % geo.fus.VMID        = [];    % volume of mid-section                           [m3], scalar
% % % geo.fus.VB          = [];    % fuselage volume                                 [m3], scalar
% % % geo.fus.SB          = [];    % fuselage planform area                          [m2], scalar
% % % geo.fus.AB          = [];    % fuselage surface area                           [m2], scalar
geo.fus.dx          = [];    % step in x coordinate                            [m] , scalar
geo.fus.x           = [];    % domain in x coordinate                          [m] , vector
geo.fus.lenx        = [];    % nr of points in x direction                     [-] , scalar

% geo.fus.VolFus      = [];    % fuselage volume distribution in x coord         [m3], vector
% geo.fus.RatioVolFus = [];    % fuselage volume distribution ratio in x coord   [-] , vector
% geo.fus.deltaVolFus = [];    % increment of volume ratio for each element      [-] , vector

geo.fus.r           = [];    % distribution of fuselage radius                 [m] , vector
geo.fus.A           = [];    % fuselage cross-sectional area                   [m2], scalar
geo.fus.P           = [];    % fuselage perimeter                              [m] , scalar
geo.fus.m           = [];    % fuselage structural geometry parameters         [-] , scalar
geo.fus.epsilon     = [];    % fuselage structural geometry parameters         [-] , scalar
geo.fus.Kmg         = [];    % fuselage structural geometry parameters         [-] , scalar
geo.fus.Kp          = [];    % fuselage structural geometry parameters         [-] , scalar
geo.fus.Kth         = [];    % fuselage structural geometry parameters         [-] , scalar
% % % geo.fus.xstick      = [];    % beam model mesh defined in interpolation procedure
geo.fus.CAERO1.n    = [];    % vector containing, for each sector with non-zero span, number of elements 
geo.fus.n           = [];    % vector containing, for each sector with non-zero span, number of elements 
%
% The following fields describe the stick model midpoints for real fuselage shape 
geo.fus.xx          = [];    % Points X-coord describing real fuselage shape   [m] , vector
geo.fus.yy          = [];    % Points Y-coord describing real fuselage shape   [m] , vector
geo.fus.zz          = [];    % Points Z-coord describing real fuselage shape   [m] , vector
%
geo.fus.x_nodes     = [];
geo.fus.x_nodes_1_2 = [];
%
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Basic calculations
%
geo.fus.bodl      = aircraft.fuselage.Total_fuselage_length;
geo.fus.lengthN   = aircraft.fuselage.Nose_length;
geo.fus.lengthAB  = aircraft.fuselage.Tail_length;
geo.fus.lengthMID = geo.fus.bodl - (geo.fus.lengthN + geo.fus.lengthAB);      
geo.fus.WB        = aircraft.weight_balance.COG(5,4,1);
                                      
geo.fus.dx   = geo.fus.bodl / pdcylin.guess.fus ;
geo.fus.x    = (0 : geo.fus.dx : geo.fus.bodl)';
geo.fus.lenx = pdcylin.guess.fus  + 1;

geo.fus.n = pdcylin.guess.fus;
geo.fus.CAERO1.n = pdcylin.stick.nfuse;
geo.fus.CAERO1.n_coarse = pdcylin.stick.nfuse_coarse;
if ~isfield(pdcylin.guess,'check')
  fprintf('\n\t\t- Fuselage computational mesh: %d elements.', pdcylin.stick.nfuse_coarse);
  fprintf('\n\t\t- Fuselage stick mesh: %d elements.', pdcylin.stick.nfuse);
end
%
%--------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------
% Extract fuselage shape to define in the stick model
%--------------------------------------------------------------------------------------------------
N = [20,2,2,20];
[params] = Geo_Fus_Shape(aircraft,[N,37]); 
geo.fus.params = params;
% Extract middle points coordinates, located in the plane of symmetry
xc = params.scx(1,:); 
zc = params.scz(1,:); 

% Extract points coordinate in the horizontal plane
thick_ver = (params.scz(10,:)-params.scz(28,:))./2;
thick_hor = (params.scy(19,:)-params.scy(1,:))./2; 
thick_dom = xc;

% Extract the minimum nr of points to describe the fuselage using a stick model 
% checking the gradient of mean line (symmetry plane)

% grad = diff(zc)./diff(xc); 
% ind = find(abs(diff(grad))>1e-9);
ind = [1,N(1),N(2)+N(1)-1,N(2)+N(1)+N(3)-2,sum(N)-3];
% xx = xc([1,1+ind,sum(N)-3]);
% zz = zc([1,1+ind,sum(N)-3]);
xx = xc(ind);
zz = zc(ind); 
 
yy = zeros(1,length(xx)); 
% Save output
geo.fus.xx = xx;
geo.fus.yy = yy;
geo.fus.zz = zz;
geo.fus.thick_dom = thick_dom';
geo.fus.thick_ver = thick_ver';
geo.fus.thick_hor = thick_hor';

%--------------------------------------------------------------------------------------------------
% Define area / radius / perimeter / volume distribution 
%--------------------------------------------------------------------------------------------------

% area at fore section (exact)
% Afore = pi/2 *( 2*aircraft.fuselage.a0_fore^2 + aircraft.fuselage.a1_fore^2 + aircraft.fuselage.b1_fore^2 );
% radius at fore section
% Rfore = sqrt( Afore/pi );
% x-location of fore section
% xfore = aircraft.fuselage.Nose_length;

% area at aft section
Aaft = pi/2 *( 2*aircraft.fuselage.a0_aft^2 + aircraft.fuselage.a1_aft^2 + aircraft.fuselage.b1_aft^2 );
% radius at aft section
Raft = sqrt( Aaft/pi );
%
geo.fus.R = Raft;
% x-location of aft section
% xaft = (geo.fus.bodl - (aircraft.fuselage.Tail_length + xfore))*aircraft.fuselage.fraction_fore + xfore ;

% define 5 main cross sections
% xA = [0; xfore; xaft; geo.fus.bodl-aircraft.fuselage.Tail_length; geo.fus.bodl];
% define correspondent 5 main cross section area
% A = [0; Afore; Aaft; Aaft; 0];

% Compute section area at each point xc
VD = [ aircraft.fuselage.Forefuse_X_sect_vertical_diameter , aircraft.fuselage.Aftfuse_X_sect_vertical_diameter ];

A = zeros(sum(N)-3,1);
for i = 1 : N(1)
    A(i) = pi/2 * (2*(thick_ver(i)/VD(1)*2*aircraft.fuselage.a0_fore)^2 + (thick_ver(i)/VD(1)*2*aircraft.fuselage.a1_fore)^2 + (thick_ver(i)/VD(1)*2*aircraft.fuselage.b1_fore)^2);
end

lfore = xc(N(2)+N(1)-1)-xc(N(1));
for i = N(1)+1 : N(1)+N(2)-1
    a0 = aircraft.fuselage.a0_fore + (aircraft.fuselage.a0_aft - aircraft.fuselage.a0_fore)/(lfore)*(xc(i)-xc(N(1)));
    a1 = aircraft.fuselage.a1_fore + (aircraft.fuselage.a1_aft - aircraft.fuselage.a1_fore)/(lfore)*(xc(i)-xc(N(1)));
    b1 = aircraft.fuselage.b1_fore + (aircraft.fuselage.b1_aft - aircraft.fuselage.b1_fore)/(lfore)*(xc(i)-xc(N(1)));
    A(i) = pi/2 * (2*a0^2 +a1^2 +b1^2);
end

for i = N(1)+N(2) : sum(N(1:3))-2
    A(i) = pi/2 *( 2*aircraft.fuselage.a0_aft^2 + aircraft.fuselage.a1_aft^2 + aircraft.fuselage.b1_aft^2 );
end

for i = sum(N(1:3))-2 : sum(N)-3
    A(i) = pi/2 *( 2*(thick_ver(i)/VD(2)*2*aircraft.fuselage.a0_aft)^2 + (thick_ver(i)/VD(2)*2*aircraft.fuselage.a1_aft)^2 + (thick_ver(i)/VD(2)*2*aircraft.fuselage.b1_aft)^2 );
end

% Linear interpolation for area
geo.fus.A = interp1(xc, A, geo.fus.x, 'linear', 'extrap');   

% Correspondent equivalent radius
geo.fus.r = sqrt(geo.fus.A/pi);

% Correspondent perimeter
geo.fus.P = 2*pi.*geo.fus.r;

% Compute volumes (this field is required for guess standard to work
% and it will be changed)

% Volume for each elemental sector in the mid-point
Vx_1_2 = 0.5*geo.fus.dx.*(geo.fus.A(2:end) + geo.fus.A(1:end-1));

% Domain
x_1_2 = ( geo.fus.x(1:end-1) + geo.fus.x(2:end) )./2;

% Add nose and tail zero val
Vx_1_2 = [0; Vx_1_2; 0];

% Correspondent domain
x_1_2 = [0; x_1_2; geo.fus.x(end)];

% Spline interpolation over nodes
V = spline( x_1_2, Vx_1_2, geo.fus.x );

%
geo.fus.deltaVolFus = V/sum(V);

% wetted surface
Swet = (geo.fus.P(2:end) + geo.fus.P(1:end-1)) .*geo.fus.dx*0.5;
geo.fus.Swet = Swet;
geo.fus.x_nodes = geo.fus.x';
%--------------------------------------------------------------------------
% Fuselage structural concept
%--------------------------------------------------------------------------
%
switch pdcylin.fus.kcon
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
geo.fus.m       = fcoef(layoutf, 1);
geo.fus.epsilon = fcoef(layoutf, 2);
geo.fus.Kmg     = fcoef(layoutf, 3);
% accounts for the fact that not all of the shell material is available to resist for hoop stress
geo.fus.Kp      = fcoef(layoutf, 4);
geo.fus.Kth     = fcoef(layoutf, 5);