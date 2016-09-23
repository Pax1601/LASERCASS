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
% Called by:    AFaWWE.m
% 
% Calls:        aero_geo.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080511      1.0     A. Da Ronch      Creation
%     091119      1.3.9   Travaglini       Modification
%
% This function is the same of Geo_Wing, but works on second wing (Wing2)
% Modified by Travaglini, adding controls on distribution of points and on
% span geometry (wing2 is the wing2 and is not the canard)
%*******************************************************************************
function [geo,pdcylin] = Geo_Wing2(pdcylin, aircraft, geo)

%--------------------------------------------------------------------------------------------------------------------------
% Initialize structure
%
geo.wing2.index              = [];    % indicate beginnning/end of sectors
%
geo.wing2.MAC_x              = [];    % MAC longitudinal location from fuselage nose                           [m]  , scalar
%
geo.wing2.xLE                = [];    % LE location in the symmetry plane                                      [m]  , scalar
geo.wing2.xQC                = [];    % QC location in the symmetry plane                                      [m]  , scalar
geo.wing2.xTE                = [];    % TE location in the symmetry plane                                      [m]  , scalar
geo.wing2.SP                 = [];    % plan area of wing                                                      [m2] , scalar
%
geo.wing2.b                  = [];    % wing span                                                              [m]  , scalar
geo.wing2.bS_inboard         = [];    % structural sempispan on quarter-chord line at inboard section          [m]  , scalar
geo.wing2.bS_midboard        = [];    % structural sempispan on quarter-chord line at midboard section         [m]  , scalar
geo.wing2.bS_outboard        = [];    % structural sempispan on quarter-chord line at outboard section         [m]  , scalar
geo.wing2.bS                 = [];    % structural sempispan on quarter-chord line                             [m]  , scalar
%
geo.wing2.CRp                = [];    % wing chord at symmetry plane                                           [m]  , scalar
geo.wing2.CR                 = [];    % wing chord at wing-fuselage connection                                 [m]  , scalar
geo.wing2.CR_kink1           = [];    % wing chord at kink 1                                                   [m]  , scalar
geo.wing2.CR_kink2           = [];    % wing chord at kink 2                                                   [m]  , scalar
geo.wing2.CT                 = [];    % wing chord at tip                                                      [m]  , scalar
%
geo.wing2.CSR                = [];    % wing structural chord at wing-fuselage connection                      [m]  , scalar
geo.wing2.CSR_kink1          = [];    % wing structural chord at kink 1                                        [m]  , scalar
geo.wing2.CSR_kink2          = [];    % wing structural chord at kink 2                                        [m]  , scalar
geo.wing2.CST                = [];    % structural chord at tip                                                [m]  , scalar
%
geo.wing2.Rt_root            = [];    % thickness ratio at wing root                                           [-]  , scalar
geo.wing2.Rt_kink1           = [];    % thickness ratio at kink1                                               [-]  , scalar
geo.wing2.Rt_kink2           = [];    % thickness ratio at kink2                                               [-]  , scalar
geo.wing2.Rt_tip             = [];    % thickness ratio at tip                                                 [-]  , scalar
%
geo.wing2.span_inboard       = [];    % span of inboard section, measured parallel to Y-global coordinate      [m]  , scalar
geo.wing2.span_midboard      = [];    % span of midboard section, measured parallel to Y-global coordinate     [m]  , scalar
geo.wing2.span_outboard      = [];    % span of outboard section, measured parallel to Y-global coordinate     [m]  , scalar
%
geo.wing2.lambdaQC_inboard   = [];    % QC sweep angle at inboard section                                      [rad], scalar
geo.wing2.lambdaQC_midboard  = [];    % QC sweep angle at midboard section                                     [rad], scalar
geo.wing2.lambdaQC_outboard  = [];    % QC sweep angle at outboard section                                     [rad], scalar
%
geo.wing2.lambdaLE_inboard   = [];    % LE sweep angle at inboard section                                      [rad], scalar
geo.wing2.lambdaLE_midboard  = [];    % LE sweep angle at midboard section                                     [rad], scalar
geo.wing2.lambdaLE_outboard  = [];    % LE sweep angle at outboard section                                     [rad], scalar
%
geo.wing2.lambdaC2_inboard   = [];    % C2 sweep angle at inboard section                                      [rad], scalar
geo.wing2.lambdaC2_midboard  = [];    % C2 sweep angle at midboard section                                     [rad], scalar
geo.wing2.lambdaC2_outboard  = [];    % C2 sweep angle at outboard section                                     [rad], scalar
%
geo.wing2.dihedral_inboard   = [];    % dihedral angle at inboard section                                      [rad], scalar
geo.wing2.dihedral_midboard  = [];    % dihedral angle at midboard section                                     [rad], scalar
geo.wing2.dihedral_outboard  = [];    % dihedral angle at outboard section                                     [rad], scalar
%
geo.wing2.dinc_inboard       = [];    % increment in incidence angle at inboard section                        [rad], scalar
geo.wing2.dinc_midboard      = [];    % increment in incidence angle at midboard section                       [rad], scalar
geo.wing2.dinc_outboard      = [];    % increment in incidence angle at outboard section                       [rad], scalar
geo.wing2.incidence_inboard  = [];    % incidence angle at inboard section                                     [rad], vector
geo.wing2.incidence_midboard = [];    % incidence angle at midboard section                                    [rad], vector
geo.wing2.incidence_outboard = [];    % incidence angle at outboard section                                    [rad], vector
geo.wing2.incidence          = [];    % incidence angle over the entire semi-wing                              [rad], vector
%
geo.wing2.SELL_inboard       = [];    % exposed area at inboard section                                        [m2] , scalar
geo.wing2.SELL_midboard      = [];    % exposed area at midboard section                                       [m2] , scalar
geo.wing2.SELL_outboard      = [];    % exposed area at outboard section                                       [m2] , scalar
geo.wing2.SELL               = [];    % exposed area for semiwing                                              [m2] , scalar
geo.wing2.Swet_inboard       = [];    % exposed area at inboard section                                        [m2] , vector
geo.wing2.Swet_midboard      = [];    % exposed area at midboard section                                       [m2] , vector
geo.wing2.Swet_outboard      = [];    % exposed area at outboard section                                       [m2] , vector
geo.wing2.Swet               = [];    % exposed area for semiwing                                              [m2] , vector
%
geo.wing2.dy_inboard         = [];    % step in inboard section                                                [m]  , scalar
geo.wing2.dy_midboard        = [];    % step in midboard section                                               [m]  , scalar
geo.wing2.dy_outboard        = [];    % step in outboard section                                               [m]  , scalar
geo.wing2.dy                 = [];    % step along wing-span                                                   [m]  , scalar
%
geo.wing2.leny               = [];    % nr of nodes along structural wing semispan                             [-]  , scalar
geo.wing2.y_inboard          = [];    % discretization along inboard section                                   [m]  , vector
geo.wing2.y_midboard         = [];    % discretization along midboard section                                  [m]  , vector
geo.wing2.y_outboard         = [];    % discretization along outboard section                                  [m]  , vector

geo.wing2.x_inboard          = [];    % discretization along inboard section                                   [m]  , vector
geo.wing2.x_midboard         = [];    % discretization along midboard section                                  [m]  , vector
geo.wing2.x_outboard         = [];    % discretization along outboard section                                  [m]  , vector
%
geo.wing2.spar_frac_inboard = [];
geo.wing2.spar_frac_outboard = [];
geo.wing2.spar_frac = [];
%
geo.wing2.y                  = [];    % discretization along structural wing semispan                          [m]  , vector
%
geo.wing2.rs_inboard         = [];    % Struct. wing chord measured parallel to X-global axis at inboard       [m]  , vector
geo.wing2.rs_midboard        = [];    % Struct. wing chord measured parallel to X-global axis at midboard      [m]  , vector
geo.wing2.rs_outboard        = [];    % Struct. wing chord measured parallel to X-global axis at outboard      [m]  , vector
geo.wing2.rs                 = [];    % Struct. wing chord measured parallel to X-global axis over semi-wing   [m]  , vector
%
geo.wing2.r_inboard          = [];    % Total wing chord measured parallel to X-global axis at inboard         [m]  , vector
geo.wing2.r_midboard         = [];    % Total wing chord measured parallel to X-global axis at midboard        [m]  , vector
geo.wing2.r_outboard         = [];    % Total wing chord measured parallel to X-global axis at outboard        [m]  , vector
geo.wing2.r                  = [];    % Total wing chord measured parallel to X-global axis over semi-wing     [m]  , vector
%
geo.wing2.Zs_inboard         = [];    % Struct. wing chord measured parallel to struct. semi-span at inboard   [m]  , vector
geo.wing2.Zs_midboard        = [];    % Struct. wing chord measured parallel to struct. semi-span at midboard  [m]  , vector
geo.wing2.Zs_outboard        = [];    % Struct. wing chord measured parallel to struct. semi-span at outboard  [m]  , vector
geo.wing2.Zs                 = [];    % Struct. wing chord measured parallel to struct. semi-span              [m]  , vector
%
geo.wing2.Z_inboard          = [];    % Total wing chord measured parallel to struct. semi-span at inboard     [m]  , vector
geo.wing2.Z_midboard         = [];    % Total wing chord measured parallel to struct. semi-span at midboard    [m]  , vector
geo.wing2.Z_outboard         = [];    % Total wing chord measured parallel to struct. semi-span at outboard    [m]  , vector
geo.wing2.Z                  = [];    % Total wing chord measured parallel to struct. semi-span over semi-wing [m]  , vector
%
geo.wing2.tbs_inboard        = [];    % wing box thickness at inboard                                          [m]  , vector
geo.wing2.tbs_midboard       = [];    % wing box thickness at midboard                                         [m]  , vector
geo.wing2.tbs_outboard       = [];    % wing box thickness at outboard                                         [m]  , vector
geo.wing2.tbs                = [];    % wing box thickness over semi-wing                                      [m]  , vector
geo.wing2.tcs                = [];    % thickness in the carrythrough structure                                [m]  , scalar
%
geo.wing2.epc                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.e                  = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.ep                 = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.ec                 = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.epw                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.Kgc                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing2.Kgw                = [];    % wing structural coefficients & exponents                               [-]  , scalar
%
% For the defined sectors, quantities are stored in appropriate vectors
geo.wing2.CAERO1.chord       = [];    % chord for each defined sector                                          [m]  , vector
geo.wing2.CAERO1.span        = [];    % span for each defined sector                                           [m]  , vector
geo.wing2.CAERO1.taper       = [];    % taper for each defined sector                                          [-]  , vector
geo.wing2.CAERO1.bS          = [];    % contains bS for each defined sectors                                   [m]  , vector
geo.wing2.CAERO1.SELL        = [];    % exposed aerodyn. area for each defined sector                          [m2] , vector
geo.wing2.CAERO1.dy          = [];    % step for each defined sector                                           [m]  , scalar
geo.wing2.CAERO1.dihedral    = [];    % dihedral for each defined sector                                       [deg], vector
geo.wing2.CAERO1.sweepLE     = [];    % LE sweep angle for each defined sector                                 [deg], vector
geo.wing2.CAERO1.sweepQC     = [];    % QC sweep angle for each defined sector                                 [deg], vector
geo.wing2.CAERO1.sweepC2     = [];    % C2 sweep angle for each defined sector                                 [deg], vector
geo.wing2.CAERO1.incidence   = [];    % incidence angle for each defined sector                                [deg], vector
geo.wing2.CAERO1.n           = [];    % number of elements for each defined sector                             [-]  , vector
geo.wing2.CAERO1.n_coarse    = [];    % number of elements for each defined sector                             [-]  , vector
geo.wing2.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                 [-]  , vector
geo.wing2.CAERO1.sup_control.frs = [];% Span fraction for the control surface                                  [-]  , vector
geo.wing2.CAERO1.sup_control.nme = [];% Name of control surface                                                [-]  , vector
geo.wing2.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.wing2.CAERO1.airfoil     = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
%
geo.wing2.csi_root           = [];    % distance from aero LE to struct box LE at root                         [m]  , scalar
geo.wing2.csi_kink1          = [];    % distance from aero LE to struct box LE at kink1                        [m]  , scalar
geo.wing2.csi_kink2          = [];    % distance from aero LE to struct box LE at kink2                        [m]  , scalar
geo.wing2.csi_tip            = [];    % distance from aero LE to struct box LE at tip                          [m]  , scalar
geo.wing2.ni_root            = [];    % distance from aero TE to struct box TE at root                         [m]  , scalar
geo.wing2.ni_kink1           = [];    % distance from aero TE to struct box TE at kink1                        [m]  , scalar
geo.wing2.ni_kink2           = [];    % distance from aero TE to struct box TE at kink2                        [m]  , scalar
geo.wing2.ni_tip             = [];    % distance from aero LE to struct box LE at tip                          [m]  , scalar
%**************************************************************************
geo.wing2.WING               = [];    % points defining each panel (3x(4*nr_sector))
geo.wing2.QC                 = [];    % points defining quarter chord line (3x(2*nr_sector))
geo.wing2.C2                 = [];    % points defining elastic line (3x(2*nr_sector))
geo.wing2.PANE               = [];    % points to write CAERO1 card
%**************************************************************************
geo.wing2.V                  = [];    % volume of struct box from actual station to tip                        [m]  , scalar
geo.wing2.cg                 = [];    % centroid of correspondent volume                                       [m]  , scalar
%
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% Basic calculations
%--------------------------------------------------------------------------------------------------------------------------

% Location of LE
geo.wing2.xLE = aircraft.wing2.longitudinal_location;
% Total wing area
geo.wing2.SP = aircraft.wing2.area;
% Wing span
geo.wing2.b = aircraft.wing2.span;
% Span of in / mid / out-board sector, measured parallel to global Y coordinate
geo.wing2.span_inboard  = aircraft.wing2.spanwise_kink1 *(geo.wing2.b/2)- geo.fus.R;
geo.wing2.span_midboard = (aircraft.wing2.spanwise_kink2 - aircraft.wing2.spanwise_kink1) *(geo.wing2.b/2);
geo.wing2.span_outboard = (1 - aircraft.wing2.spanwise_kink2) *(geo.wing2.b/2);

% Check on different span, want to obtain a uniform distribution of node
% both on guess model and smart model.
if ~isfield(pdcylin.guess,'check')

  Ltot = sum(abs([geo.wing2.span_inboard , geo.wing2.span_midboard , geo.wing2.span_outboard]));
  Ls = [geo.wing2.span_inboard , geo.wing2.span_midboard , geo.wing2.span_outboard];
  n = ceil( Ls * (pdcylin.guess.wing2.inboard + pdcylin.guess.wing2.midboard + pdcylin.guess.wing2.outboard)/Ltot );
  n(n==0) = 1;
  fprintf('\n\t\t- Wing2 computational mesh: %d, %d, %d elements.', n(1), n(2), n(3));
  m = ceil( Ls * (pdcylin.stick.nwing2_inboard + pdcylin.stick.nwing2_midboard + pdcylin.stick.nwing2_outboard )/Ltot );
  m(m==0) = 1;
  m2 = ceil( Ls * (pdcylin.stick.nwing2_inboard_coarse + pdcylin.stick.nwing2_midboard_coarse + ...
                     pdcylin.stick.nwing2_outboard_coarse )/Ltot);
  m2(m2==0) = 1;
  fprintf('\n\t\t- Wing2 stick mesh: %d, %d, %d elements.', m2(1), m2(2), m2(3));
  ny = ceil( Ls * (pdcylin.stick.ny.wing2_inboard + pdcylin.stick.ny.wing2_midboard + pdcylin.stick.ny.wing2_outboard )/Ltot );
  ny(ny==0) = 1;
  fprintf('\n\t\t- Wing2 spanwise aerodynamic mesh: %d, %d, %d elements.', ny(1), ny(2), ny(3));
%
    pdcylin.guess.wing2.inboard = n(1);
    pdcylin.guess.wing2.midboard = n(2);
    pdcylin.guess.wing2.outboard = n(3);
    pdcylin.stick.nwing2_inboard = m(1);
    pdcylin.stick.nwing2_midboard = m(2);
    pdcylin.stick.nwing2_outboard = m(3);
    pdcylin.stick.nwing2_inboard_coarse = m2(1);
    pdcylin.stick.nwing2_midboard_coarse= m2(2);
    pdcylin.stick.nwing2_outboard_coarse = m2(3);

end
MIN = 0.01*aircraft.wing2.span*0.5;
% Wing chord at symmetry plane
geo.wing2.CRp = 2*geo.wing2.SP/( geo.wing2.b*( ...
                               (1 + aircraft.wing2.taper_kink1) *...
                                  aircraft.wing2.spanwise_kink1 +...
                               (aircraft.wing2.taper_kink1 + aircraft.wing2.taper_kink2) *...
                                  (aircraft.wing2.spanwise_kink2 - aircraft.wing2.spanwise_kink1) +...
                               (aircraft.wing2.taper_kink2 + aircraft.wing2.taper_tip) *...
                                  (1 - aircraft.wing2.spanwise_kink2)) );
% Location of QC / TE at body-center line
geo.wing2.xQC = geo.wing2.xLE + 1/4 *geo.wing2.CRp;
geo.wing2.xTE = geo.wing2.xLE + geo.wing2.CRp;
% Tip chord
geo.wing2.CT = aircraft.wing2.taper_tip *geo.wing2.CRp;
% Structural wing chord at tip
geo.wing2.CST = (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip) *geo.wing2.CT;
% Thickness ratios
geo.wing2.Rt_root  = aircraft.wing2.thickness_root;
geo.wing2.Rt_kink1 = aircraft.wing2.thickness_kink1;
geo.wing2.Rt_kink2 = aircraft.wing2.thickness_kink2;
geo.wing2.Rt_tip   = aircraft.wing2.thickness_tip;
% Distance from aero LE to struct box LE
geo.wing2.csi_tip = aircraft.fuel.Fore_wing_spar_loc_tip*geo.wing2.CT;
% Distance from aero TE to struct box TE
geo.wing2.ni_tip  = aircraft.fuel.Aft_wing_spar_loc_tip *geo.wing2.CT;
% 
geo.wing2.index = 1;
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% inboard
%--------------------------------------------------------------------------------------------------------------------------

if (geo.wing2.span_inboard > MIN)

    geo.wing2.index = [geo.wing2.index; geo.wing2.index(end) + pdcylin.guess.wing2.inboard];
    % Aerodynamic wing chord at kink 1
    geo.wing2.CR_kink1 = geo.wing2.CRp *aircraft.wing2.taper_kink1;
    % Angles in [rad]
    geo.wing2.lambdaQC_inboard = aircraft.wing2.quarter_chord_sweep_inboard *(pi/180);
    geo.wing2.lambdaLE_inboard = aircraft.wing2.LE_sweep_inboard *(pi/180);    
    geo.wing2.dihedral_inboard = aircraft.wing2.dihedral_inboard *(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.wing2.dinc_inboard = (aircraft.wing2.kink1_incidence-aircraft.wing2.root_incidence)/pdcylin.guess.wing2.inboard;
    %
    if (geo.wing2.dinc_inboard ~= 0)
        geo.wing2.incidence_inboard = (aircraft.wing2.root_incidence:geo.wing2.dinc_inboard:aircraft.wing2.kink1_incidence)';
    else
        geo.wing2.incidence_inboard = aircraft.wing2.root_incidence*ones(pdcylin.guess.wing2.inboard+1, 1);
    end
    %
    % Wing chord at wing-fuselage connection 
    geo.wing2.CR = geo.wing2.CRp - geo.fus.R/(aircraft.wing2.spanwise_kink1 *(geo.wing2.b/2)) *(geo.wing2.CRp - geo.wing2.CR_kink1);
    
    %**********************************************************************************************
    c = geo.wing2.CR;
    b = geo.wing2.span_inboard;
    T = geo.wing2.CR_kink1/c;
    TW(1,1) = aircraft.wing2.root_incidence*pi/180;
    TW(2,1) = aircraft.wing2.kink1_incidence*pi/180;
    SW = geo.wing2.lambdaQC_inboard;
    dihed = geo.wing2.dihedral_inboard;
    ox = geo.wing2.xLE + geo.fus.R*tan(geo.wing2.lambdaQC_inboard) ; %% ????
    oy = 0;
    oz = aircraft.wing2.vertical_location;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_root + ...
                 (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_kik1 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1)/2;
    REFx = ox + alpha(1,1)*geo.wing2.CR;
    REFy = oy;
    REFz = oz;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.wing2.bS_inboard = norm(qc(:,2) - qc(:,1));
    geo.wing2.WING = [geo.wing2.WING wing];
    geo.wing2.QC   = [geo.wing2.QC qc];
    geo.wing2.C2   = [geo.wing2.C2 c2];
    geo.wing2.PANE = [geo.wing2.PANE pane];
    %**********************************************************************************************

    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing2.lambdaC2_inboard = atan( (geo.wing2.C2(1,end)-geo.wing2.C2(1,end-1))/(geo.wing2.C2(2,end)-geo.wing2.C2(2,end-1)) );
    %**********************************************************************
    
    % Structural wing chord at wing-fuselage connection
    geo.wing2.CSR = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing2.CR;    
    % Structural wing chord at kink1
    geo.wing2.CSR_kink1 = (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1) *geo.wing2.CR_kink1;    
    % Aerodynamic exposed area at inboard section
    geo.wing2.SELL_inboard = 0.5 *(geo.wing2.CR + geo.wing2.CR_kink1) *geo.wing2.span_inboard;
    % Step in inboard
    geo.wing2.dy_inboard = geo.wing2.bS_inboard/pdcylin.guess.wing2.inboard;
    % Node discretization in inboard
    geo.wing2.y_inboard = (0 : geo.wing2.dy_inboard : geo.wing2.bS_inboard)';
    % Structural wing chord in inboard measured along X axis
    geo.wing2.rs_inboard = geo.wing2.CSR - geo.wing2.y_inboard./geo.wing2.bS_inboard*(geo.wing2.CSR - geo.wing2.CSR_kink1);
    % Aerodynamic wing chord in inboard measured along X axis
    geo.wing2.r_inboard = geo.wing2.CR - geo.wing2.y_inboard./geo.wing2.bS_inboard*(geo.wing2.CR - geo.wing2.CR_kink1);
    geo.wing2.x_inboard = linspace(geo.wing2.C2(1,end-1),geo.wing2.C2(1,end),pdcylin.guess.wing2.inboard+1)';
    geo.wing2.spar_frac_inboard = linspace(alpha(1,1),alpha(2,1),length(geo.wing2.rs_inboard))';
    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing2.Swet_inboard = 0.5 *(geo.wing2.r_inboard(1:end-1)+geo.wing2.r_inboard(2:end)) .*(geo.wing2.span_inboard./pdcylin.guess.wing2.inboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    
    % Structural wing chord in inboard  measured perpendicular to structural chord
%     geo.wing2.Zs_inboard =
%     geo.wing2.rs_inboard.*cos(geo.wing2.lambdaQC_inboard);
    geo.wing2.Zs_inboard = geo.wing2.rs_inboard.*cos(geo.wing2.lambdaC2_inboard);
    
    % Aerodynamic wing chord in inboard measured perpendicular to structural chord 
%     geo.wing2.Z_inboard = geo.wing2.r_inboard.*cos(geo.wing2.lambdaQC_inboard);
    geo.wing2.Z_inboard = geo.wing2.r_inboard.*cos(geo.wing2.lambdaC2_inboard);
    
    % Thickness distribution
    t_root  = geo.wing2.Rt_root  *geo.wing2.CR;
    t_kink1 = geo.wing2.Rt_kink1 *geo.wing2.CR_kink1;
    geo.wing2.tbs_inboard = t_root - geo.wing2.y_inboard./geo.wing2.bS_inboard *(t_root - t_kink1);
    
    %
    % CAERO1 parameters
    geo.wing2.CAERO1.dihedral = [geo.wing2.CAERO1.dihedral; aircraft.wing2.dihedral_inboard];
    geo.wing2.CAERO1.sweepLE  = [geo.wing2.CAERO1.sweepLE ; aircraft.wing2.LE_sweep_inboard];    
    geo.wing2.CAERO1.sweepQC  = [geo.wing2.CAERO1.sweepQC ; aircraft.wing2.quarter_chord_sweep_inboard];
    geo.wing2.CAERO1.sweepC2  = [geo.wing2.CAERO1.sweepC2 ; geo.wing2.lambdaC2_inboard *180/pi];
    geo.wing2.CAERO1.bS       = [geo.wing2.CAERO1.bS      ; geo.wing2.bS_inboard];
    geo.wing2.CAERO1.span     = [geo.wing2.CAERO1.span    ; geo.wing2.span_inboard];    
    geo.wing2.CAERO1.SELL     = [geo.wing2.CAERO1.SELL    ; geo.wing2.SELL_inboard];    
    geo.wing2.CAERO1.dy       = [geo.wing2.CAERO1.dy      ; geo.wing2.dy_inboard];    
    geo.wing2.CAERO1.n        = [geo.wing2.CAERO1.n       ; pdcylin.stick.nwing2_inboard];
    geo.wing2.CAERO1.n_coarse = [geo.wing2.CAERO1.n_coarse       ; pdcylin.stick.nwing2_inboard_coarse];
    geo.wing2.CAERO1.airfoil  = [geo.wing2.CAERO1.airfoil ; {aircraft.wing2.airfoilRoot};...
                                                          {aircraft.wing2.airfoilKink1}];
    
    % Fraction chord and name of control surface
    rootc = aircraft.wing2.flap.root_chord;
    kink1c = aircraft.wing2.flap.kink1_chord;
    if rootc > 1
        rootc = rootc/100;
    end
    if kink1c > 1
        kink1c = kink1c/100;
    end
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; rootc];
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; kink1c];
    % Span fraction set to one
    geo.wing2.CAERO1.sup_control.frs = [geo.wing2.CAERO1.sup_control.frs; 1];
    geo.wing2.CAERO1.sup_control.nme = [geo.wing2.CAERO1.sup_control.nme; ' flap1r2'];
    
    % AELINK card: set to -1 (flap)
    geo.wing2.CAERO1.sup_control.typ = [geo.wing2.CAERO1.sup_control.typ; -1];
        
    geo.wing2.csi_root  = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing2.CR;
    geo.wing2.ni_root   = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing2.CR;
    geo.wing2.csi_kink1 = aircraft.fuel.Fore_wing_spar_loc_kik1 *geo.wing2.CR_kink1;
    geo.wing2.ni_kink1  = aircraft.fuel.Aft_wing_spar_loc_kin1  *geo.wing2.CR_kink1;
    
else

    geo.wing2.span_inboard = 0.0;
    geo.wing2.CR_kink1     = geo.wing2.CRp;
    geo.wing2.CR           = geo.wing2.CRp;
    geo.wing2.SELL_inboard = 0.0;
    geo.wing2.bS_inboard   = 0.0;
    
end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% midboard
%--------------------------------------------------------------------------------------------------------------------------

if (geo.wing2.span_midboard > MIN)

    geo.wing2.index = [geo.wing2.index; geo.wing2.index(end) + pdcylin.guess.wing2.midboard];
    % Aerodynamic wing chord at kink 2
    geo.wing2.CR_kink2 = geo.wing2.CRp *aircraft.wing2.taper_kink2;
    % Structural wing chord at kink2
    geo.wing2.CSR_kink2 = (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2) *geo.wing2.CR_kink2;
    
    if (geo.wing2.span_inboard <= MIN)
        
        % Update outboard span and move inboard sector to the intersection
        geo.wing2.span_midboard = aircraft.wing2.spanwise_kink2 *(geo.wing2.b/2) - geo.fus.R;
        geo.wing2.CR            = geo.wing2.CRp - geo.fus.R/geo.wing2.span_midboard *(geo.wing2.CRp - geo.wing2.CR_kink2);
        geo.wing2.CR_kink1      = geo.wing2.CR;
        geo.wing2.CSR           = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing2.CR;   
        geo.wing2.CSR_kink1     = (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1) *geo.wing2.CR_kink1;
        t_kink1                = geo.wing2.CR_kink1 *geo.wing2.Rt_kink1;
        geo.wing2.csi_root      = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing2.CR;
        geo.wing2.ni_root       = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing2.CR;
        geo.wing2.csi_kink1     = aircraft.fuel.Fore_wing_spar_loc_kik1 *geo.wing2.CR_kink1;
        geo.wing2.ni_kink1      = aircraft.fuel.Aft_wing_spar_loc_kin1  *geo.wing2.CR_kink1;
        
    end   
    
    % Angles
    geo.wing2.lambdaQC_midboard = aircraft.wing2.quarter_chord_sweep_midboard *(pi/180);
    geo.wing2.lambdaLE_midboard = aircraft.wing2.LE_sweep_midboard *(pi/180);      
    geo.wing2.dihedral_midboard = aircraft.wing2.dihedral_midboard *(pi/180);   
    % Angle of incidence [deg], at the end it's converted
    geo.wing2.dinc_midboard = (aircraft.wing2.kink2_incidence - aircraft.wing2.kink1_incidence)/pdcylin.guess.wing2.midboard;
    %
    if (geo.wing2.dinc_midboard ~= 0)
        geo.wing2.incidence_midboard = (aircraft.wing2.kink1_incidence : geo.wing2.dinc_midboard : aircraft.wing2.kink2_incidence)';
    else
        geo.wing2.incidence_midboard = aircraft.wing2.kink1_incidence *ones(pdcylin.guess.wing2.midboard+1, 1);
    end
    
    %**********************************************************************************************
    c = geo.wing2.CR_kink1;
    b = geo.wing2.span_midboard;
    T = geo.wing2.CR_kink2/c;
    TW(1,1) = aircraft.wing2.kink1_incidence*pi/180;
    TW(2,1) = aircraft.wing2.kink2_incidence*pi/180;
    SW = geo.wing2.lambdaQC_midboard;
    dihed = geo.wing2.dihedral_midboard;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_kik1 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_kin2 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2)/2;
             
    if (geo.wing2.span_inboard <= MIN)
        ox   = geo.wing2.xQC + geo.fus.R*tan(geo.wing2.lambdaQC_midboard) - geo.wing2.CR/4;
        oy   = 0;
        oz   = aircraft.wing2.vertical_location;
        REFx = ox + alpha(1,1) *geo.wing2.CR;
        REFy = oy;
        REFz = oz;
    else
        ox   = geo.wing2.QC( 1,end );
        oy   = geo.wing2.QC( 2,end );
        oz   = geo.wing2.QC( 3,end );
        REFx = geo.wing2.C2( 1,end );
        REFy = geo.wing2.C2( 2,end );
        REFz = geo.wing2.C2( 3,end );
    end

    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.wing2.bS_midboard = norm(qc(:,end) - qc(:,end-1));
    geo.wing2.WING = [geo.wing2.WING wing];
    geo.wing2.QC   = [geo.wing2.QC qc];
    geo.wing2.C2   = [geo.wing2.C2 c2];
    geo.wing2.PANE = [geo.wing2.PANE pane];
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing2.lambdaC2_midboard = atan( (geo.wing2.C2(1,end)-geo.wing2.C2(1,end-1))/(geo.wing2.C2(2,end)-geo.wing2.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at midboard section
    geo.wing2.SELL_midboard = 0.5 *(geo.wing2.CR_kink1 + geo.wing2.CR_kink2) *geo.wing2.span_midboard;
    % Step in midboard
    geo.wing2.dy_midboard = geo.wing2.bS_midboard/pdcylin.guess.wing2.midboard;
    % Node discretization in midboard
    geo.wing2.y_midboard = (0 : geo.wing2.dy_midboard : geo.wing2.bS_midboard)';
    % Structural wing chord in midboard measured along X axis
    geo.wing2.rs_midboard = geo.wing2.CSR_kink1-geo.wing2.y_midboard./geo.wing2.bS_midboard*(geo.wing2.CSR_kink1-geo.wing2.CSR_kink2);
    % Aerodynamic wing chord in midboard measured along X axis
    geo.wing2.r_midboard = geo.wing2.CR_kink1-geo.wing2.y_midboard./geo.wing2.bS_midboard*(geo.wing2.CR_kink1-geo.wing2.CR_kink2);
    
	%$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing2.Swet_midboard = 0.5 *(geo.wing2.r_midboard(1:end-1)+geo.wing2.r_midboard(2:end)) .*(geo.wing2.span_midboard./pdcylin.guess.wing2.midboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    % Structural wing chord in midboard  measured perpendicular to structural chord
%     geo.wing2.Zs_midboard = geo.wing2.rs_midboard.*cos(geo.wing2.lambdaQC_midboard);
    geo.wing2.Zs_midboard = geo.wing2.rs_midboard.*cos(geo.wing2.lambdaC2_midboard);
    
    % Aerodynamic wing chord in midboard  measured perpendicular to structural chord
%     geo.wing2.Z_midboard = geo.wing2.r_midboard.*cos(geo.wing2.lambdaQC_midboard);
    geo.wing2.Z_midboard = geo.wing2.r_midboard.*cos(geo.wing2.lambdaC2_midboard);
    geo.wing2.x_midboard = linspace(geo.wing2.C2(1,end-1),geo.wing2.C2(1,end),pdcylin.guess.wing2.midboard+1)';
    % Thickness distribution
    t_kink2 = geo.wing2.CR_kink2 *geo.wing2.Rt_kink2;
    geo.wing2.tbs_midboard = t_kink1 - geo.wing2.y_midboard./geo.wing2.bS_midboard *(t_kink1 - t_kink2);

    % CAERO1 parameters
    geo.wing2.CAERO1.dihedral = [geo.wing2.CAERO1.dihedral; aircraft.wing2.dihedral_midboard];   
    geo.wing2.CAERO1.sweepLE  = [geo.wing2.CAERO1.sweepLE ; aircraft.wing2.LE_sweep_midboard];      
    geo.wing2.CAERO1.sweepQC  = [geo.wing2.CAERO1.sweepQC ; aircraft.wing2.quarter_chord_sweep_midboard];    
    geo.wing2.CAERO1.sweepC2  = [geo.wing2.CAERO1.sweepC2 ; geo.wing2.lambdaC2_midboard *180/pi];
    geo.wing2.CAERO1.bS       = [geo.wing2.CAERO1.bS      ; geo.wing2.bS_midboard];
    geo.wing2.CAERO1.span     = [geo.wing2.CAERO1.span    ; geo.wing2.span_midboard];
    geo.wing2.CAERO1.SELL     = [geo.wing2.CAERO1.SELL    ; geo.wing2.SELL_midboard];
    geo.wing2.CAERO1.dy       = [geo.wing2.CAERO1.dy      ; geo.wing2.dy_midboard];
    geo.wing2.CAERO1.n        = [geo.wing2.CAERO1.n       ; pdcylin.stick.nwing2_midboard];
    geo.wing2.CAERO1.n_coarse = [geo.wing2.CAERO1.n_coarse       ; pdcylin.stick.nwing2_midboard_coarse];    
    geo.wing2.CAERO1.airfoil  = [geo.wing2.CAERO1.airfoil; {aircraft.wing2.airfoilKink1};...
                                                         {aircraft.wing2.airfoilKink2}];
    
    % Fraction chord and name of control surface    
    kink1c = aircraft.wing2.flap.kink1_chord;
    kink2c = aircraft.wing2.flap.kink2_chord;
    if kink1c > 1
        kink1c = kink1c / 100;
    end
    if kink2c > 1
        kink2c = kink2c / 100;
    end    
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; kink1c];
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; kink2c];
    % Span fraction set to one
    geo.wing2.CAERO1.sup_control.frs = [geo.wing2.CAERO1.sup_control.frs; 1];
    geo.wing2.CAERO1.sup_control.nme = [geo.wing2.CAERO1.sup_control.nme; ' flap2r2'];
    
    % AELINK card: set to -1 (flap)
    geo.wing2.CAERO1.sup_control.typ = [geo.wing2.CAERO1.sup_control.typ; -1];

    geo.wing2.csi_kink2 = aircraft.fuel.Fore_wing_spar_loc_kin2*geo.wing2.CR_kink2;
    geo.wing2.ni_kink2  = aircraft.fuel.Aft_wing_spar_loc_kin2 *geo.wing2.CR_kink2;

else
    
    geo.wing2.span_midboard = 0.0;
    geo.wing2.CR_kink2      = geo.wing2.CR_kink1;
    geo.wing2.SELL_midboard = 0.0;
    geo.wing2.bS_midboard   = 0.0;
    
end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% outboard
%--------------------------------------------------------------------------------------------------------------------------

if (geo.wing2.span_outboard > MIN)

    if (geo.wing2.span_midboard <= MIN)
        
        if (geo.wing2.span_inboard <= MIN)
            
            geo.wing2.span_outboard = (geo.wing2.b/2) - geo.fus.R;
            geo.wing2.CR            = geo.wing2.CRp - geo.fus.R/geo.wing2.span_outboard *(geo.wing2.CRp - geo.wing2.CT);
            geo.wing2.CR_kink1      = geo.wing2.CR;
            geo.wing2.CR_kink2      = geo.wing2.CR;
            geo.wing2.CSR           = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing2.CR;   
            geo.wing2.CSR_kink2     = (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2) *geo.wing2.CR_kink2;
            t_kink2                = geo.wing2.CR_kink2 *geo.wing2.Rt_kink2;
            
        else
            
            geo.wing2.CR_kink2  = geo.wing2.CR_kink1;
            geo.wing2.CSR_kink2 = (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2) *geo.wing2.CR_kink2;
            t_kink2 = geo.wing2.CR_kink2 *geo.wing2.Rt_kink2;
            
        end
        
        geo.wing2.csi_root  = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing2.CR;
        geo.wing2.ni_root   = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing2.CR;
        geo.wing2.csi_kink1 = aircraft.fuel.Fore_wing_spar_loc_kik1 *geo.wing2.CR_kink1;
        geo.wing2.ni_kink1  = aircraft.fuel.Aft_wing_spar_loc_kin1  *geo.wing2.CR_kink1;
        geo.wing2.csi_kink2 = aircraft.fuel.Fore_wing_spar_loc_kin2 *geo.wing2.CR_kink2;
        geo.wing2.ni_kink2  = aircraft.fuel.Aft_wing_spar_loc_kin2  *geo.wing2.CR_kink2;
        
    end
    
    geo.wing2.index = [geo.wing2.index; geo.wing2.index(end) + pdcylin.guess.wing2.outboard];
    % Angles
    geo.wing2.lambdaQC_outboard = aircraft.wing2.quarter_chord_sweep_outboard*(pi/180);
    geo.wing2.lambdaLE_outboard = aircraft.wing2.LE_sweep_outboard*(pi/180);
    geo.wing2.dihedral_outboard = aircraft.wing2.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.wing2.dinc_outboard = (aircraft.wing2.tip_incidence-aircraft.wing2.kink2_incidence)/pdcylin.guess.wing2.outboard;
    if (geo.wing2.dinc_outboard ~= 0)
        geo.wing2.incidence_outboard = (aircraft.wing2.kink2_incidence:geo.wing2.dinc_outboard:aircraft.wing2.tip_incidence)';
    else
        geo.wing2.incidence_outboard = aircraft.wing2.kink2_incidence*ones(pdcylin.guess.wing2.outboard+1, 1);
    end 
    
    %**********************************************************************************************
    c = geo.wing2.CR_kink2;
    b = geo.wing2.span_outboard;
    T = geo.wing2.CT/c;
    TW(1,1) = aircraft.wing2.kink2_incidence*pi/180;
    TW(2,1) = aircraft.wing2.tip_incidence*pi/180;
    SW = geo.wing2.lambdaQC_outboard;
    dihed = geo.wing2.dihedral_outboard;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_kin2 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_tip + ...
                 (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip)/2;
    
    if (geo.wing2.span_inboard <= MIN) && (geo.wing2.span_midboard <= MIN)
        ox = geo.wing2.xQC + geo.fus.R*tan(geo.wing2.lambdaQC_outboard) - geo.wing2.CR/4;
        oy = 0;
        oz = aircraft.wing2.vertical_location;
        REFx = ox + alpha(1,1) *geo.wing2.CR;
        REFy = oy;
        REFz = oz;
    else
        ox = geo.wing2.QC( 1,end );
        oy = geo.wing2.QC( 2,end );
        oz = geo.wing2.QC( 3,end );
        REFx = geo.wing2.C2( 1,end );
        REFy = geo.wing2.C2( 2,end );
        REFz = geo.wing2.C2( 3,end );
    end
    
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.wing2.bS_outboard = norm(qc(:,end)-qc(:,end-1));
    geo.wing2.WING = [geo.wing2.WING wing];
    geo.wing2.QC   = [geo.wing2.QC qc];
    geo.wing2.C2   = [geo.wing2.C2 c2];
    geo.wing2.PANE = [geo.wing2.PANE pane];
    %**********************************************************************************************    
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing2.lambdaC2_outboard = atan( (geo.wing2.C2(1,end)-geo.wing2.C2(1,end-1))/(geo.wing2.C2(2,end)-geo.wing2.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at outboard section
    geo.wing2.SELL_outboard = 0.5*(geo.wing2.CR_kink2+geo.wing2.CT)*geo.wing2.span_outboard;
    % Step in outboard
    geo.wing2.dy_outboard = geo.wing2.bS_outboard/pdcylin.guess.wing2.outboard;
    % Node discretization in outboard
    geo.wing2.y_outboard = (0 : geo.wing2.dy_outboard : geo.wing2.bS_outboard)';
    % Structural wing chord in outboard measured along X axis
    geo.wing2.rs_outboard = geo.wing2.CSR_kink2-geo.wing2.y_outboard./geo.wing2.bS_outboard*(geo.wing2.CSR_kink2-geo.wing2.CST);
    % Aerodynamic wing chord in outboard measured along X axis
    geo.wing2.r_outboard = geo.wing2.CR_kink2-geo.wing2.y_outboard./geo.wing2.bS_outboard*(geo.wing2.CR_kink2-geo.wing2.CT);
    
    geo.wing2.x_outboard = linspace(geo.wing2.C2(1,end-1),geo.wing2.C2(1,end),pdcylin.guess.wing2.outboard+1)';
    geo.wing2.spar_frac_outboard = linspace(alpha(1,1),alpha(2,1),length(geo.wing2.rs_outboard))';
    
	%$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing2.Swet_outboard = 0.5 *(geo.wing2.r_outboard(1:end-1)+geo.wing2.r_outboard(2:end)) .*(geo.wing2.span_outboard./pdcylin.guess.wing2.outboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    % Structural wing chord in outboard measured perpendicular to structural chord
%     geo.wing2.Zs_outboard = geo.wing2.rs_outboard.*cos(geo.wing2.lambdaQC_outboard);
    geo.wing2.Zs_outboard = geo.wing2.rs_outboard.*cos(geo.wing2.lambdaC2_outboard);
    
    % Aerodynamic wing chord in outboard measured perpendicular to structural chord
%     geo.wing2.Z_outboard = geo.wing2.r_outboard.*cos(geo.wing2.lambdaQC_outboard);
    geo.wing2.Z_outboard = geo.wing2.r_outboard.*cos(geo.wing2.lambdaC2_outboard);
    
    % Thickness distribution
    t_tip = geo.wing2.CT *geo.wing2.Rt_tip;
    geo.wing2.tbs_outboard = t_kink2 - geo.wing2.y_outboard./geo.wing2.bS_outboard*(t_kink2 - t_tip);    

    % CAERO1 parameters    
    geo.wing2.CAERO1.dihedral = [geo.wing2.CAERO1.dihedral; aircraft.wing2.dihedral_outboard]; 
    geo.wing2.CAERO1.sweepLE  = [geo.wing2.CAERO1.sweepLE ; aircraft.wing2.LE_sweep_outboard];    
    geo.wing2.CAERO1.sweepQC  = [geo.wing2.CAERO1.sweepQC; aircraft.wing2.quarter_chord_sweep_outboard];    
    geo.wing2.CAERO1.sweepC2  = [geo.wing2.CAERO1.sweepC2 ; geo.wing2.lambdaC2_outboard *180/pi];
    geo.wing2.CAERO1.bS       = [geo.wing2.CAERO1.bS; geo.wing2.bS_outboard];
    geo.wing2.CAERO1.span     = [geo.wing2.CAERO1.span; geo.wing2.span_outboard];
    geo.wing2.CAERO1.SELL     = [geo.wing2.CAERO1.SELL; geo.wing2.SELL_outboard];    
    geo.wing2.CAERO1.dy       = [geo.wing2.CAERO1.dy; geo.wing2.dy_outboard];
    geo.wing2.CAERO1.n        = [geo.wing2.CAERO1.n; pdcylin.stick.nwing2_outboard];
    geo.wing2.CAERO1.n_coarse = [geo.wing2.CAERO1.n_coarse; pdcylin.stick.nwing2_outboard_coarse];
    geo.wing2.CAERO1.airfoil  = [geo.wing2.CAERO1.airfoil; {aircraft.wing2.airfoilKink2};
                                                         {aircraft.wing2.airfoilTip}];
    
    % Fraction chord and name of control surface
    frc = aircraft.wing2.aileron.chord;
    if frc > 1
        frc = frc/100;
    end
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; kink2c];
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; frc];
    geo.wing2.CAERO1.sup_control.frc = [geo.wing2.CAERO1.sup_control.frc; frc];
    % Span fraction
    geo.wing2.CAERO1.sup_control.frs = [geo.wing2.CAERO1.sup_control.frs; aircraft.wing2.aileron.Span];
    geo.wing2.CAERO1.sup_control.nme = [geo.wing2.CAERO1.sup_control.nme; 'aileror2'];
    
    % AELINK card: set to 1 (aileron)
    geo.wing2.CAERO1.sup_control.typ = [geo.wing2.CAERO1.sup_control.typ; 1];

else 
    
    geo.wing2.span_outboard = 0.0;
    geo.wing2.bS_outboard   = 0.0;
    geo.wing2.SELL_outboard = 0.0;

end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% Save inboard and outboard parameters refering to correspondent parameter
%--------------------------------------------------------------------------------------------------------------------------

% From inboard
if (geo.wing2.span_inboard > MIN)    
    
    geo.wing2.y         = geo.wing2.y_inboard;
    geo.wing2.rs        = geo.wing2.rs_inboard;
    geo.wing2.r         = geo.wing2.r_inboard;
    geo.wing2.Zs        = geo.wing2.Zs_inboard;
    geo.wing2.Z         = geo.wing2.Z_inboard;
    geo.wing2.tbs       = geo.wing2.tbs_inboard;
    geo.wing2.incidence = geo.wing2.incidence_inboard;    
    geo.wing2.dy        = geo.wing2.dy_inboard .*ones( pdcylin.guess.wing2.inboard+1, 1);
    geo.wing2.Swet      = geo.wing2.Swet_inboard;
    geo.wing2.x         = geo.wing2.x_inboard;
    geo.wing2.spar_frac = geo.wing2.spar_frac_inboard;
end

% From midboard
if (geo.wing2.span_midboard > MIN)    
    
    if (geo.wing2.span_inboard > MIN)    
        
        geo.wing2.y         = [geo.wing2.y        ; geo.wing2.y(end)+geo.wing2.y_midboard(2:end)];
        geo.wing2.rs        = [geo.wing2.rs       ; geo.wing2.rs_midboard(2:end)];
        geo.wing2.r         = [geo.wing2.r        ; geo.wing2.r_midboard(2:end)];
        geo.wing2.Zs        = [geo.wing2.Zs       ; geo.wing2.Zs_midboard(2:end)];
        geo.wing2.Z         = [geo.wing2.Z        ; geo.wing2.Z_midboard(2:end)];
        geo.wing2.tbs       = [geo.wing2.tbs      ; geo.wing2.tbs_midboard(2:end)];
        geo.wing2.incidence = [geo.wing2.incidence; geo.wing2.incidence_midboard(2:end)];        
        geo.wing2.dy        = [geo.wing2.dy       ; geo.wing2.dy_midboard .*ones( pdcylin.guess.wing2.midboard, 1)];
        geo.wing2.Swet      = [geo.wing2.Swet; geo.wing2.Swet_midboard];
        geo.wing2.x         = [geo.wing2.x        ; geo.wing2.x(end)+geo.wing2.x_midboard(2:end)];
        geo.wing2.spar_frac = [geo.wing2.spar_frac; geo.wing2.spar_frac_midboard(2:end)];
    else
        
        geo.wing2.y         = geo.wing2.y_midboard;
        geo.wing2.rs        = geo.wing2.rs_midboard;
        geo.wing2.r         = geo.wing2.r_midboard;
        geo.wing2.Zs        = geo.wing2.Zs_midboard;
        geo.wing2.Z         = geo.wing2.Z_midboard;
        geo.wing2.tbs       = geo.wing2.tbs_midboard;
        geo.wing2.incidence = geo.wing2.incidence_midboard;        
        geo.wing2.dy        = geo.wing2.dy_midboard .*ones( pdcylin.guess.wing2.midboard+1, 1);
        geo.wing2.Swet      = geo.wing2.Swet_midboard;
        geo.wing2.x         = geo.wing2.x_midboard;
        geo.wing2.spar_frac = geo.wing2.spar_frac_midboard;
    end 
    
end

% From outboard
if (geo.wing2.span_outboard > MIN)    
    
    if (isempty(geo.wing2.y)~=1)        
        
        geo.wing2.y         = [geo.wing2.y        ; geo.wing2.y(end)+geo.wing2.y_outboard(2:end)];
        geo.wing2.rs        = [geo.wing2.rs       ; geo.wing2.rs_outboard(2:end)];
        geo.wing2.r         = [geo.wing2.r        ; geo.wing2.r_outboard(2:end)];
        geo.wing2.Zs        = [geo.wing2.Zs       ; geo.wing2.Zs_outboard(2:end)];
        geo.wing2.Z         = [geo.wing2.Z        ; geo.wing2.Z_outboard(2:end)];
        geo.wing2.tbs       = [geo.wing2.tbs      ; geo.wing2.tbs_outboard(2:end)];  
        geo.wing2.incidence = [geo.wing2.incidence; geo.wing2.incidence_outboard(2:end)];
        geo.wing2.dy        = [geo.wing2.dy       ; geo.wing2.dy_outboard .*ones( pdcylin.guess.wing2.outboard, 1)];
        geo.wing2.Swet      = [geo.wing2.Swet; geo.wing2.Swet_outboard];
        geo.wing2.x         = [geo.wing2.x       ; geo.wing2.x(end)+geo.wing2.x_outboard(2:end)];
        geo.wing2.spar_frac = [geo.wing2.spar_frac; geo.wing2.spar_frac_outboard(2:end)];
    else
        
        geo.wing2.y         = geo.wing2.y_outboard;
        geo.wing2.rs        = geo.wing2.rs_outboard;
        geo.wing2.r         = geo.wing2.r_outboard;
        geo.wing2.Zs        = geo.wing2.Zs_outboard;
        geo.wing2.Z         = geo.wing2.Z_outboard;
        geo.wing2.tbs       = geo.wing2.tbs_outboard;      
        geo.wing2.incidence = geo.wing2.incidence_outboard;        
        geo.wing2.dy        = geo.wing2.dy_outboard .*ones( pdcylin.guess.wing2.outboard+1, 1);
        geo.wing2.Swet      = geo.wing2.Swet_outboard;
        geo.wing2.x         = geo.wing2.x_outboard;
        geo.wing2.spar_frac = geo.wing2.spar_frac_outboard;

    end 
    
end

% Wetted surface vector [n_bar x 1]: account for upper and lower surface
geo.wing2.Swet = 2*geo.wing2.Swet;
%--------------------------------------------------------------------------------------------------------------------------

% Convert incidence angle
geo.wing2.incidence = geo.wing2.incidence.*(pi/180);
% Exposed area for total semi-wing
geo.wing2.SELL = geo.wing2.SELL_inboard + geo.wing2.SELL_midboard + geo.wing2.SELL_outboard;
% Number of nodes used to discretize structural semi-wing
geo.wing2.leny = length(geo.wing2.y);
% Carrythrough thickness
geo.wing2.tcs  = max(geo.wing2.tbs);
% Length of structural line as summation of each single sector
geo.wing2.bS   = geo.wing2.bS_inboard + geo.wing2.bS_midboard + geo.wing2.bS_outboard;

%**************************************************************************************************
% MAC longitudinal location respect to fuselage nose
macYnondim = ((aircraft.Reference_wing.non_dim_MAC_y_bar*geo.wing2.b/2)-geo.fus.R)/(geo.wing2.b/2-geo.fus.R);
MAC_y = macYnondim*geo.wing2.bS;
indMAC_y = find( MAC_y >= geo.wing2.y );
indsect_MAC = find( indMAC_y(end) >= geo.wing2.index );
geo.wing2.MAC_x = geo.wing2.QC(1,indsect_MAC(end)) +...
                 ( MAC_y-geo.wing2.QC(2,indsect_MAC(end)) )*tan(geo.wing2.CAERO1.sweepQC(indsect_MAC(end))*pi/180);
geo.wing2.MAC_y = MAC_y;
%**************************************************************************************************


%--------------------------------------------------------------------------------------------------------------------------
% Complete CAERO1 quantities: chord / taper / incidence
%--------------------------------------------------------------------------------------------------------------------------

geo.wing2.CAERO1.chord     = [geo.wing2.CAERO1.chord; geo.wing2.CR];
geo.wing2.CAERO1.incidence = [geo.wing2.CAERO1.incidence; aircraft.wing2.root_incidence];
%
if ( ((geo.wing2.span_inboard>MIN)&&(geo.wing2.span_midboard>MIN)) || ((geo.wing2.span_inboard>MIN)&&(geo.wing2.span_outboard>MIN)) )
    geo.wing2.CAERO1.chord     = [geo.wing2.CAERO1.chord; geo.wing2.CR_kink1];
    geo.wing2.CAERO1.incidence = [geo.wing2.CAERO1.incidence; aircraft.wing2.kink1_incidence];
end
%
if (geo.wing2.span_midboard > MIN) && (geo.wing2.span_outboard > MIN)
    geo.wing2.CAERO1.chord     = [geo.wing2.CAERO1.chord; geo.wing2.CR_kink2];
    geo.wing2.CAERO1.incidence = [geo.wing2.CAERO1.incidence; aircraft.wing2.kink2_incidence];
end
%
geo.wing2.CAERO1.chord     = [geo.wing2.CAERO1.chord; geo.wing2.CT];
geo.wing2.CAERO1.taper     = (geo.wing2.CAERO1.chord(2:end)./geo.wing2.CAERO1.chord(1:end-1));
geo.wing2.CAERO1.incidence = [geo.wing2.CAERO1.incidence; aircraft.wing2.tip_incidence];

%*****************************************
if isequal(pdcylin.stick.model.symmXZ, 1)
    nrsp = length(geo.wing2.CAERO1.taper);
    for i = 1:nrsp
        % copy the name
        geo.wing2.CAERO1.sup_control.nme(i+nrsp,:) = geo.wing2.CAERO1.sup_control.nme(i,:);
        % change the last letter
        geo.wing2.CAERO1.sup_control.nme(i+nrsp,end) = 'l';
    end
end
%*****************************************

%
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% Wing structural concept
%--------------------------------------------------------------------------------------------------------------------------

% Load file
wcoef = load('strwingcoef.txt');

geo.wing2.ep  = wcoef( pdcylin.wing2.kcon, 1 );
geo.wing2.e   = wcoef( pdcylin.wing2.kcon, 2 );
geo.wing2.epc = wcoef( pdcylin.wing2.kcon, 3 );
geo.wing2.ec  = wcoef( pdcylin.wing2.kcon, 4 );
geo.wing2.epw = wcoef( pdcylin.wing2.kcon, 5 );
geo.wing2.Kgc = wcoef( pdcylin.wing2.kcon, 6 );
geo.wing2.Kgw = wcoef( pdcylin.wing2.kcon, 7 );
%
%--------------------------------------------------------------------------------------------------------------------------
