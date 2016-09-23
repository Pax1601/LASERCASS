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
% Called by:    AFaWWE.m
% 
% Calls:        aero_geo.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080511      1.0     A. Da Ronch      Creation
%     091119      1.3.9   Travaglini       Modification
%
% Modified by Travaglini, adding controls on distribution of points and on
% span geometry
%*******************************************************************************
function [geo,pdcylin] = Geo_Wing(pdcylin, aircraft, geo)

% Initialize structure
%
geo.wing.index              = [];    % indicate beginnning/end of sectors
%
geo.wing.MAC_x              = [];    % MAC longitudinal location from fuselage nose                           [m]  , scalar
%
geo.wing.xLE                = [];    % LE location in the symmetry plane                                      [m]  , scalar
geo.wing.xQC                = [];    % QC location in the symmetry plane                                      [m]  , scalar
geo.wing.xTE                = [];    % TE location in the symmetry plane                                      [m]  , scalar
geo.wing.SP                 = [];    % plan area of wing                                                      [m2] , scalar
%
geo.wing.b                  = [];    % wing span                                                              [m]  , scalar
geo.wing.bS_inboard         = [];    % structural sempispan on quarter-chord line at inboard section          [m]  , scalar
geo.wing.bS_midboard        = [];    % structural sempispan on quarter-chord line at midboard section         [m]  , scalar
geo.wing.bS_outboard        = [];    % structural sempispan on quarter-chord line at outboard section         [m]  , scalar
geo.wing.bS                 = [];    % structural sempispan on quarter-chord line                             [m]  , scalar
%
geo.wing.CRp                = [];    % wing chord at symmetry plane                                           [m]  , scalar
geo.wing.CR                 = [];    % wing chord at wing-fuselage connection                                 [m]  , scalar
geo.wing.CR_kink1           = [];    % wing chord at kink 1                                                   [m]  , scalar
geo.wing.CR_kink2           = [];    % wing chord at kink 2                                                   [m]  , scalar
geo.wing.CT                 = [];    % wing chord at tip                                                      [m]  , scalar
%
geo.wing.CSR                = [];    % wing structural chord at wing-fuselage connection                      [m]  , scalar
geo.wing.CSR_kink1          = [];    % wing structural chord at kink 1                                        [m]  , scalar
geo.wing.CSR_kink2          = [];    % wing structural chord at kink 2                                        [m]  , scalar
geo.wing.CST                = [];    % structural chord at tip                                                [m]  , scalar
%
geo.wing.Rt_root            = [];    % thickness ratio at wing root                                           [-]  , scalar
geo.wing.Rt_kink1           = [];    % thickness ratio at kink1                                               [-]  , scalar
geo.wing.Rt_kink2           = [];    % thickness ratio at kink2                                               [-]  , scalar
geo.wing.Rt_tip             = [];    % thickness ratio at tip                                                 [-]  , scalar
%
geo.wing.span_inboard       = [];    % span of inboard section, measured parallel to Y-global coordinate      [m]  , scalar
geo.wing.span_midboard      = [];    % span of midboard section, measured parallel to Y-global coordinate     [m]  , scalar
geo.wing.span_outboard      = [];    % span of outboard section, measured parallel to Y-global coordinate     [m]  , scalar
%
geo.wing.lambdaQC_inboard   = [];    % QC sweep angle at inboard section                                      [rad], scalar
geo.wing.lambdaQC_midboard  = [];    % QC sweep angle at midboard section                                     [rad], scalar
geo.wing.lambdaQC_outboard  = [];    % QC sweep angle at outboard section                                     [rad], scalar
%
geo.wing.lambdaLE_inboard   = [];    % LE sweep angle at inboard section                                      [rad], scalar
geo.wing.lambdaLE_midboard  = [];    % LE sweep angle at midboard section                                     [rad], scalar
geo.wing.lambdaLE_outboard  = [];    % LE sweep angle at outboard section                                     [rad], scalar
%
geo.wing.lambdaC2_inboard   = [];    % C2 sweep angle at inboard section                                      [rad], scalar
geo.wing.lambdaC2_midboard  = [];    % C2 sweep angle at midboard section                                     [rad], scalar
geo.wing.lambdaC2_outboard  = [];    % C2 sweep angle at outboard section                                     [rad], scalar
%
geo.wing.dihedral_inboard   = [];    % dihedral angle at inboard section                                      [rad], scalar
geo.wing.dihedral_midboard  = [];    % dihedral angle at midboard section                                     [rad], scalar
geo.wing.dihedral_outboard  = [];    % dihedral angle at outboard section                                     [rad], scalar
%
geo.wing.dinc_inboard       = [];    % increment in incidence angle at inboard section                        [rad], scalar
geo.wing.dinc_midboard      = [];    % increment in incidence angle at midboard section                       [rad], scalar
geo.wing.dinc_outboard      = [];    % increment in incidence angle at outboard section                       [rad], scalar
geo.wing.incidence_inboard  = [];    % incidence angle at inboard section                                     [rad], vector
geo.wing.incidence_midboard = [];    % incidence angle at midboard section                                    [rad], vector
geo.wing.incidence_outboard = [];    % incidence angle at outboard section                                    [rad], vector
geo.wing.incidence          = [];    % incidence angle over the entire semi-wing                              [rad], vector
%
geo.wing.SELL_inboard       = [];    % exposed area at inboard section                                        [m2] , scalar
geo.wing.SELL_midboard      = [];    % exposed area at midboard section                                       [m2] , scalar
geo.wing.SELL_outboard      = [];    % exposed area at outboard section                                       [m2] , scalar
geo.wing.SELL               = [];    % exposed area for semiwing                                              [m2] , scalar
geo.wing.Swet_inboard       = [];    % exposed area at inboard section                                        [m2] , vector
geo.wing.Swet_midboard      = [];    % exposed area at midboard section                                       [m2] , vector
geo.wing.Swet_outboard      = [];    % exposed area at outboard section                                       [m2] , vector
geo.wing.Swet               = [];    % exposed area for semiwing                                              [m2] , vector
%
geo.wing.dy_inboard         = [];    % step in inboard section                                                [m]  , scalar
geo.wing.dy_midboard        = [];    % step in midboard section                                               [m]  , scalar
geo.wing.dy_outboard        = [];    % step in outboard section                                               [m]  , scalar
geo.wing.dy                 = [];    % step along wing-span                                                   [m]  , scalar
%
geo.wing.leny               = [];    % nr of nodes along structural wing semispan                             [-]  , scalar
geo.wing.y_inboard          = [];    % discretization along inboard section                                   [m]  , vector
geo.wing.y_midboard         = [];    % discretization along midboard section                                  [m]  , vector
geo.wing.y_outboard         = [];    % discretization along outboard section                                  [m]  , vector

geo.wing.x_inboard          = [];    % discretization along inboard section                                   [m]  , vector
geo.wing.x_midboard         = [];    % discretization along midboard section                                  [m]  , vector
geo.wing.x_outboard         = [];    % discretization along outboard section                                  [m]  , vector

geo.wing.y                  = [];    % discretization along structural wing semispan                          [m]  , vector
%
geo.wing.rs_inboard         = [];    % Struct. wing chord measured parallel to X-global axis at inboard       [m]  , vector
geo.wing.rs_midboard        = [];    % Struct. wing chord measured parallel to X-global axis at midboard      [m]  , vector
geo.wing.rs_outboard        = [];    % Struct. wing chord measured parallel to X-global axis at outboard      [m]  , vector
geo.wing.rs                 = [];    % Struct. wing chord measured parallel to X-global axis over semi-wing   [m]  , vector
%
geo.wing.r_inboard          = [];    % Total wing chord measured parallel to X-global axis at inboard         [m]  , vector
geo.wing.r_midboard         = [];    % Total wing chord measured parallel to X-global axis at midboard        [m]  , vector
geo.wing.r_outboard         = [];    % Total wing chord measured parallel to X-global axis at outboard        [m]  , vector
geo.wing.r                  = [];    % Total wing chord measured parallel to X-global axis over semi-wing     [m]  , vector
%
geo.wing.Zs_inboard         = [];    % Struct. wing chord measured parallel to struct. semi-span at inboard   [m]  , vector
geo.wing.Zs_midboard        = [];    % Struct. wing chord measured parallel to struct. semi-span at midboard  [m]  , vector
geo.wing.Zs_outboard        = [];    % Struct. wing chord measured parallel to struct. semi-span at outboard  [m]  , vector
geo.wing.Zs                 = [];    % Struct. wing chord measured parallel to struct. semi-span              [m]  , vector
%
geo.wing.spar_frac_inboard = [];
geo.wing.spar_frac_midboard = [];
geo.wing.spar_frac_outboard = [];
geo.wing.spar_frac = [];
%
geo.wing.Z_inboard          = [];    % Total wing chord measured parallel to struct. semi-span at inboard     [m]  , vector
geo.wing.Z_midboard         = [];    % Total wing chord measured parallel to struct. semi-span at midboard    [m]  , vector
geo.wing.Z_outboard         = [];    % Total wing chord measured parallel to struct. semi-span at outboard    [m]  , vector
geo.wing.Z                  = [];    % Total wing chord measured parallel to struct. semi-span over semi-wing [m]  , vector
%
geo.wing.tbs_inboard        = [];    % wing box thickness at inboard                                          [m]  , vector
geo.wing.tbs_midboard       = [];    % wing box thickness at midboard                                         [m]  , vector
geo.wing.tbs_outboard       = [];    % wing box thickness at outboard                                         [m]  , vector
geo.wing.tbs                = [];    % wing box thickness over semi-wing                                      [m]  , vector
geo.wing.tcs                = [];    % thickness in the carrythrough structure                                [m]  , scalar
%
geo.wing.epc                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.e                  = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.ep                 = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.ec                 = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.epw                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.Kgc                = [];    % wing structural coefficients & exponents                               [-]  , scalar
geo.wing.Kgw                = [];    % wing structural coefficients & exponents                               [-]  , scalar
%
% For the defined sectors, quantities are stored in appropriate vectors
geo.wing.CAERO1.chord       = [];    % chord for each defined sector                                          [m]  , vector
geo.wing.CAERO1.span        = [];    % span for each defined sector                                           [m]  , vector
geo.wing.CAERO1.taper       = [];    % taper for each defined sector                                          [-]  , vector
geo.wing.CAERO1.bS          = [];    % contains bS for each defined sectors                                   [m]  , vector
geo.wing.CAERO1.SELL        = [];    % exposed aerodyn. area for each defined sector                          [m2] , vector
geo.wing.CAERO1.dy          = [];    % step for each defined sector                                           [m]  , scalar
geo.wing.CAERO1.dihedral    = [];    % dihedral for each defined sector                                       [deg], vector
geo.wing.CAERO1.sweepLE     = [];    % LE sweep angle for each defined sector                                 [deg], vector
geo.wing.CAERO1.sweepQC     = [];    % QC sweep angle for each defined sector                                 [deg], vector
geo.wing.CAERO1.sweepC2     = [];    % C2 sweep angle for each defined sector                                 [deg], vector
geo.wing.CAERO1.incidence   = [];    % incidence angle for each defined sector                                [deg], vector
geo.wing.CAERO1.n           = [];    % number of elements for each defined sector                             [-]  , vector
geo.wing.CAERO1.n_coarse    = [];    % number of elements for each defined sector                             [-]  , vector
geo.wing.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                 [-]  , vector
geo.wing.CAERO1.sup_control.frs = [];% Span fraction for the control surface                                  [-]  , vector
geo.wing.CAERO1.sup_control.nme = [];% Name of control surface                                                [-]  , vector
geo.wing.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.wing.CAERO1.airfoil     = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
geo.wing.CAERO1.sup_control.position = [1];
%
geo.wing.csi_root           = [];    % distance from aero LE to struct box LE at root                         [m]  , scalar
geo.wing.csi_kink1          = [];    % distance from aero LE to struct box LE at kink1                        [m]  , scalar
geo.wing.csi_kink2          = [];    % distance from aero LE to struct box LE at kink2                        [m]  , scalar
geo.wing.csi_tip            = [];    % distance from aero LE to struct box LE at tip                          [m]  , scalar
geo.wing.ni_root            = [];    % distance from aero TE to struct box TE at root                         [m]  , scalar
geo.wing.ni_kink1           = [];    % distance from aero TE to struct box TE at kink1                        [m]  , scalar
geo.wing.ni_kink2           = [];    % distance from aero TE to struct box TE at kink2                        [m]  , scalar
geo.wing.ni_tip             = [];    % distance from aero LE to struct box LE at tip                          [m]  , scalar
%**************************************************************************
geo.wing.WING               = [];    % points defining each panel (3x(4*nr_sector))
geo.wing.QC                 = [];    % points defining quarter chord line (3x(2*nr_sector))
geo.wing.C2                 = [];    % points defining elastic line (3x(2*nr_sector))
geo.wing.PANE               = [];    % points to write CAERO1 card
%**************************************************************************
geo.wing.V                  = [];    % volume of struct box from actual station to tip                        [m]  , scalar
geo.wing.cg                 = [];    % centroid of correspondent volume                                       [m]  , scalar
%
lab = TRIMlabels;
labels = lab(:, 1);
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% Basic calculations
%--------------------------------------------------------------------------------------------------------------------------

% Location of LE
geo.wing.xLE = aircraft.wing1.longitudinal_location;
% Total wing area
geo.wing.SP = aircraft.wing1.area;
% Wing span
geo.wing.b = aircraft.wing1.span;
% Span of in / mid / out-board sector, measured parallel to global Y coordinate
geo.wing.span_inboard  = aircraft.wing1.spanwise_kink1 *(geo.wing.b/2)- geo.fus.R;
geo.wing.span_midboard = (aircraft.wing1.spanwise_kink2 - aircraft.wing1.spanwise_kink1) *(geo.wing.b/2);
geo.wing.span_outboard = (1 - aircraft.wing1.spanwise_kink2) *(geo.wing.b/2);
MIN = 0.01*aircraft.wing1.span*0.5;
% Check on different span, want to obtain a uniform distribution of node
% both on guess model and smart model.
if ~isfield(pdcylin.guess,'check')
%
  Ltot = sum(abs([geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard]));
  Ls = [geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard];
  index = find(Ls<MIN);
  n = ceil( Ls * (pdcylin.guess.wing.inboard + pdcylin.guess.wing.midboard + pdcylin.guess.wing.outboard)/Ltot );
  n(index) = 0;
  fprintf('\n\t\t- Wing computational mesh: %d, %d, %d elements.', n(1), n(2), n(3));
  m = ceil( Ls * (pdcylin.stick.nwing_inboard + pdcylin.stick.nwing_midboard + pdcylin.stick.nwing_outboard )/Ltot );
  m(index) = 0;
  m2 = ceil( Ls * (pdcylin.stick.nwing_inboard_coarse + pdcylin.stick.nwing_midboard_coarse + ...
                     pdcylin.stick.nwing_outboard_coarse )/Ltot);
  m2(index) = 0;
  fprintf('\n\t\t- Wing stick mesh: %d, %d, %d elements.', m2(1), m2(2), m2(3));
  ny = ceil( Ls * (pdcylin.stick.ny.wing_inboard + pdcylin.stick.ny.wing_midboard + pdcylin.stick.ny.wing_outboard )/Ltot );
  ny(index) = 0;
  fprintf('\n\t\t- Wing spanwise aerodynamic mesh: %d, %d, %d elements.', ny(1), ny(2), ny(3));
%
  pdcylin.guess.wing.inboard = n(1);
  pdcylin.guess.wing.midboard = n(2);
  pdcylin.guess.wing.outboard = n(3);
  pdcylin.stick.nwing_inboard = m(1);
  pdcylin.stick.nwing_midboard = m(2);
  pdcylin.stick.nwing_outboard = m(3);
  pdcylin.stick.nwing_inboard_coarse = m2(1);
  pdcylin.stick.nwing_midboard_coarse= m2(2);
  pdcylin.stick.nwing_outboard_coarse = m2(3);
  pdcylin.stick.ny.wing_inboard = ny(1);
  pdcylin.stick.ny.wing_midboard = ny(2);
  pdcylin.stick.ny.wing_outboard = ny(3);
%
  pdcylin.guess.wing.winglet = 0;
  pdcylin.stick.nwing_winglet = 0;
  pdcylin.stick.nwing_winglet_coarse =  0;
  pdcylin.stick.ny.wing_winglet = 0;
  pdcylin.stick.nx.wing_winglet = 0;
  pdcylin.stick.nx.sup_control.wing_winglet = 0;
%
  if aircraft.winglet.present == 1 && aircraft.winglet.Span>0
      pdcylin.guess.wing.winglet = max(ceil(n./[geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard]*aircraft.winglet.Span));
      pdcylin.stick.nwing_winglet = max(ceil(m./[geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard]*aircraft.winglet.Span));
      pdcylin.stick.nwing_winglet_coarse = max(ceil(m2./[geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard]*aircraft.winglet.Span));
      pdcylin.stick.ny.wing_winglet = max(ceil(ny./[geo.wing.span_inboard , geo.wing.span_midboard , geo.wing.span_outboard]*aircraft.winglet.Span));
      pdcylin.stick.nx.wing_winglet = pdcylin.stick.nx.wing_outboard + pdcylin.stick.nx.sup_control.wing_outboard;
      pdcylin.stick.nx.sup_control.wing_winglet = 0;
  end
end

% Wing chord at symmetry plane
geo.wing.CRp = 2*geo.wing.SP/( geo.wing.b*( ...
                               (1 + aircraft.wing1.taper_kink1) *...
                                  aircraft.wing1.spanwise_kink1 +...
                               (aircraft.wing1.taper_kink1 + aircraft.wing1.taper_kink2) *...
                                  (aircraft.wing1.spanwise_kink2 - aircraft.wing1.spanwise_kink1) +...
                               (aircraft.wing1.taper_kink2 + aircraft.wing1.taper_tip) *...
                                  (1 - aircraft.wing1.spanwise_kink2)) );
% Location of QC / TE at body-center line
geo.wing.xQC = geo.wing.xLE + 1/4 *geo.wing.CRp; 
geo.wing.xTE = geo.wing.xLE + geo.wing.CRp;
% Tip chord
geo.wing.CT = aircraft.wing1.taper_tip *geo.wing.CRp;
% Structural wing chord at tip
geo.wing.CST = (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip) *geo.wing.CT;
% Thickness ratios
%geo.wing.Rt_root  = aircraft.wing1.thickness_root;
geo.wing.Rt_root = eq_wbox_t(aircraft.wing1.airfoilRoot,[aircraft.fuel.Fore_wing_spar_loc_root, aircraft.fuel.Aft_wing_spar_loc_root]);
%geo.wing.Rt_kink1 = aircraft.wing1.thickness_kink1;
%geo.wing.Rt_kink2 = aircraft.wing1.thickness_kink2;
%geo.wing.Rt_tip   = aircraft.wing1.thickness_tip;
% Distance from aero LE to struct box LE
geo.wing.csi_tip = aircraft.fuel.Fore_wing_spar_loc_tip*geo.wing.CT;
% Distance from aero TE to struct box TE
geo.wing.ni_tip  = aircraft.fuel.Aft_wing_spar_loc_tip *geo.wing.CT;
% 
geo.wing.index = 1;
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% inboard
%--------------------------------------------------------------------------------------------------------------------------
flap1lab = str2_8ch_right(labels{17});

if (geo.wing.span_inboard > MIN)

    geo.wing.index = [geo.wing.index; geo.wing.index(end) + pdcylin.guess.wing.inboard];
    % Aerodynamic wing chord at kink 1
    geo.wing.CR_kink1 = geo.wing.CRp *aircraft.wing1.taper_kink1;
    % Angles in [rad]
    geo.wing.lambdaQC_inboard = aircraft.wing1.quarter_chord_sweep_inboard *(pi/180);
    geo.wing.lambdaLE_inboard = aircraft.wing1.LE_sweep_inboard *(pi/180);    
    geo.wing.dihedral_inboard = aircraft.wing1.dihedral_inboard *(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.wing.dinc_inboard = (aircraft.wing1.kink1_incidence-aircraft.wing1.root_incidence)/pdcylin.guess.wing.inboard;
    %
    if (geo.wing.dinc_inboard ~= 0)
        geo.wing.incidence_inboard = (aircraft.wing1.root_incidence:geo.wing.dinc_inboard:aircraft.wing1.kink1_incidence)';
    else
        geo.wing.incidence_inboard = aircraft.wing1.root_incidence*ones(pdcylin.guess.wing.inboard+1, 1);
    end
    %
    % Wing chord at wing-fuselage connection 
    geo.wing.CR = geo.wing.CRp - geo.fus.R/(aircraft.wing1.spanwise_kink1 *(geo.wing.b/2)) *(geo.wing.CRp - geo.wing.CR_kink1);
    
    %**********************************************************************************************
    c = geo.wing.CR;
    b = geo.wing.span_inboard;
    T = geo.wing.CR_kink1/c;
    TW(1,1) = aircraft.wing1.root_incidence*pi/180;
    TW(2,1) = aircraft.wing1.kink1_incidence*pi/180;
    SW = geo.wing.lambdaQC_inboard;
    dihed = geo.wing.dihedral_inboard;
    ox = geo.wing.xLE + geo.fus.R*tan(geo.wing.lambdaQC_inboard) ; %% ????
    oy = 0;
    oz = aircraft.wing1.vertical_location;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_root + ...
                 (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_kik1 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1)/2; 
    REFx = ox + alpha(1,1)*geo.wing.CR;
    REFy = oy;
    REFz = oz;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz, pdcylin);
    geo.wing.bS_inboard = norm(qc(:,2) - qc(:,1));
    geo.wing.WING = [geo.wing.WING wing];
    geo.wing.QC   = [geo.wing.QC qc];
    geo.wing.C2   = [geo.wing.C2 c2];
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************

    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing.lambdaC2_inboard = atan( (geo.wing.C2(1,end)-geo.wing.C2(1,end-1))/(geo.wing.C2(2,end)-geo.wing.C2(2,end-1)) );
    %**********************************************************************
    
    % Structural wing chord at wing-fuselage connection
    geo.wing.CSR = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing.CR;    
    % Structural wing chord at kink1
    geo.wing.CSR_kink1 = (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1) *geo.wing.CR_kink1;    
    % Aerodynamic exposed area at inboard section
    geo.wing.SELL_inboard = 0.5 *(geo.wing.CR + geo.wing.CR_kink1) *geo.wing.span_inboard;
    % Step in inboard
    geo.wing.dy_inboard = geo.wing.bS_inboard/pdcylin.guess.wing.inboard;
    % Node discretization in inboard
    geo.wing.y_inboard = (0 : geo.wing.dy_inboard : geo.wing.bS_inboard)';
    % Structural wing chord in inboard measured along X axis
    geo.wing.rs_inboard = geo.wing.CSR - geo.wing.y_inboard./geo.wing.bS_inboard*(geo.wing.CSR - geo.wing.CSR_kink1);
    % Aerodynamic wing chord in inboard measured along X axis
    geo.wing.r_inboard = geo.wing.CR - geo.wing.y_inboard./geo.wing.bS_inboard*(geo.wing.CR - geo.wing.CR_kink1);
    %%%
    geo.wing.x_inboard = linspace(geo.wing.C2(1,end-1),geo.wing.C2(1,end),pdcylin.guess.wing.inboard+1)';
    geo.wing.spar_frac_inboard = linspace(alpha(1,1),alpha(2,1),length(geo.wing.rs_inboard))';
    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing.Swet_inboard = 0.5 *(geo.wing.r_inboard(1:end-1)+geo.wing.r_inboard(2:end)) .*(geo.wing.span_inboard./pdcylin.guess.wing.inboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    
    % Structural wing chord in inboard  measured perpendicular to structural chord
%     geo.wing.Zs_inboard =
%     geo.wing.rs_inboard.*cos(geo.wing.lambdaQC_inboard);
    geo.wing.Zs_inboard = geo.wing.rs_inboard.*cos(geo.wing.lambdaC2_inboard);
    
    % Aerodynamic wing chord in inboard measured perpendicular to structural chord 
%     geo.wing.Z_inboard = geo.wing.r_inboard.*cos(geo.wing.lambdaQC_inboard);
    geo.wing.Z_inboard = geo.wing.r_inboard.*cos(geo.wing.lambdaC2_inboard);
    
    % Thickness distribution
    geo.wing.Rt_kink1 = eq_wbox_t(aircraft.wing1.airfoilKink1,[aircraft.fuel.Fore_wing_spar_loc_kik1, aircraft.fuel.Aft_wing_spar_loc_kin1]);
    t_root  = geo.wing.Rt_root  *geo.wing.CR;
    t_kink1 = geo.wing.Rt_kink1 *geo.wing.CR_kink1;
    geo.wing.tbs_inboard = t_root - geo.wing.y_inboard./geo.wing.bS_inboard *(t_root - t_kink1);
    
    %
    % CAERO1 parameters
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_inboard];
    geo.wing.CAERO1.sweepLE  = [geo.wing.CAERO1.sweepLE ; aircraft.wing1.LE_sweep_inboard];    
    geo.wing.CAERO1.sweepQC  = [geo.wing.CAERO1.sweepQC ; aircraft.wing1.quarter_chord_sweep_inboard];
    geo.wing.CAERO1.sweepC2  = [geo.wing.CAERO1.sweepC2 ; geo.wing.lambdaC2_inboard *180/pi];
    geo.wing.CAERO1.bS       = [geo.wing.CAERO1.bS      ; geo.wing.bS_inboard];
    geo.wing.CAERO1.span     = [geo.wing.CAERO1.span    ; geo.wing.span_inboard];    
    geo.wing.CAERO1.SELL     = [geo.wing.CAERO1.SELL    ; geo.wing.SELL_inboard];    
    geo.wing.CAERO1.dy       = [geo.wing.CAERO1.dy      ; geo.wing.dy_inboard];    
    geo.wing.CAERO1.n        = [geo.wing.CAERO1.n       ; pdcylin.stick.nwing_inboard];
    geo.wing.CAERO1.n_coarse = [geo.wing.CAERO1.n_coarse       ; pdcylin.stick.nwing_inboard_coarse];
    geo.wing.CAERO1.airfoil  = [geo.wing.CAERO1.airfoil ; {aircraft.wing1.airfoilRoot};...
                                                          {aircraft.wing1.airfoilKink1}];
    if aircraft.wing1.flap.present == 1
        % Fraction chord and name of control surface
        rootc = aircraft.wing1.flap.root_chord;
        kink1c = aircraft.wing1.flap.kink1_chord;
        if rootc > 1
            rootc = rootc/100;
        end
        if kink1c > 1
            kink1c = kink1c/100;
        end
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; rootc];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; kink1c];
        % Span fraction set to one
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 1];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; flap1lab];
        
        % AELINK card: set to -1 (flap)
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; -1];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];
    else
        kink1c =0;
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 0];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; '    none'];
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];
    end    
    geo.wing.csi_root  = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing.CR;
    geo.wing.ni_root   = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing.CR;
    geo.wing.csi_kink1 = aircraft.fuel.Fore_wing_spar_loc_kik1 *geo.wing.CR_kink1;
    geo.wing.ni_kink1  = aircraft.fuel.Aft_wing_spar_loc_kin1  *geo.wing.CR_kink1;
    
else

    geo.wing.span_inboard = 0.0;
    geo.wing.CR_kink1     = geo.wing.CRp;
    geo.wing.CR           = geo.wing.CRp;
    geo.wing.SELL_inboard = 0.0;
    geo.wing.bS_inboard   = 0.0;
    
end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% midboard
%--------------------------------------------------------------------------------------------------------------------------
flap2lab = str2_8ch_right(labels{18});

if (geo.wing.span_midboard > MIN)

    geo.wing.index = [geo.wing.index; geo.wing.index(end) + pdcylin.guess.wing.midboard];
    % Aerodynamic wing chord at kink 2
    geo.wing.CR_kink2 = geo.wing.CRp *aircraft.wing1.taper_kink2;
    % Structural wing chord at kink2
    geo.wing.CSR_kink2 = (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2) *geo.wing.CR_kink2;
    
    if (geo.wing.span_inboard <= MIN)
        
        % Update outboard span and move inboard sector to the intersection
        geo.wing.span_midboard = aircraft.wing1.spanwise_kink2 *(geo.wing.b/2) - geo.fus.R;
        geo.wing.CR            = geo.wing.CRp - geo.fus.R/geo.wing.span_midboard *(geo.wing.CRp - geo.wing.CR_kink2);
        geo.wing.CR_kink1      = geo.wing.CR;
        % use kink2 data, kink1 is positioned at wing-fuse intersection
        geo.wing.Rt_kink1 = geo.wing.Rt_root;
        geo.wing.CSR           = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing.CR;   
        geo.wing.CSR_kink1     = geo.wing.CSR;
        t_kink1                = geo.wing.CR_kink1 *geo.wing.Rt_kink1;
        geo.wing.csi_root      = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing.CR;
        geo.wing.ni_root       = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing.CR;
        geo.wing.csi_kink1     = geo.wing.csi_root;
        geo.wing.ni_kink1      = geo.wing.ni_root;
        
    end   
    
    % Angles
    geo.wing.lambdaQC_midboard = aircraft.wing1.quarter_chord_sweep_midboard *(pi/180);
    geo.wing.lambdaLE_midboard = aircraft.wing1.LE_sweep_midboard *(pi/180);      
    geo.wing.dihedral_midboard = aircraft.wing1.dihedral_midboard *(pi/180);   
    % Angle of incidence [deg], at the end it's converted
    geo.wing.dinc_midboard = (aircraft.wing1.kink2_incidence - aircraft.wing1.kink1_incidence)/pdcylin.guess.wing.midboard;
    %
    if (geo.wing.dinc_midboard ~= 0)
        geo.wing.incidence_midboard = (aircraft.wing1.kink1_incidence : geo.wing.dinc_midboard : aircraft.wing1.kink2_incidence)';
    else
        geo.wing.incidence_midboard = aircraft.wing1.kink1_incidence *ones(pdcylin.guess.wing.midboard+1, 1);
    end
    
    %**********************************************************************************************
    c = geo.wing.CR_kink1;
    b = geo.wing.span_midboard;
    T = geo.wing.CR_kink2/c;
    TW(1,1) = aircraft.wing1.kink1_incidence*pi/180;
    TW(2,1) = aircraft.wing1.kink2_incidence*pi/180;
    SW = geo.wing.lambdaQC_midboard;
    dihed = geo.wing.dihedral_midboard;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_kik1 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin1 - aircraft.fuel.Fore_wing_spar_loc_kik1)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_kin2 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2)/2;
             
    if (geo.wing.span_inboard <= MIN)
        ox   = geo.wing.xQC + geo.fus.R*tan(geo.wing.lambdaQC_midboard) - geo.wing.CR/4;
        oy   = 0;
        oz   = aircraft.wing1.vertical_location;
        REFx = ox + alpha(1,1) *geo.wing.CR;
        REFy = oy;
        REFz = oz;
    else
        ox   = geo.wing.QC( 1,end );
        oy   = geo.wing.QC( 2,end );
        oz   = geo.wing.QC( 3,end );
        REFx = geo.wing.C2( 1,end );
        REFy = geo.wing.C2( 2,end );
        REFz = geo.wing.C2( 3,end );
    end

    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz, pdcylin);
    geo.wing.bS_midboard = norm(qc(:,end) - qc(:,end-1));
    geo.wing.WING = [geo.wing.WING wing];
    geo.wing.QC   = [geo.wing.QC qc];
    geo.wing.C2   = [geo.wing.C2 c2];
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing.lambdaC2_midboard = atan( (geo.wing.C2(1,end)-geo.wing.C2(1,end-1))/(geo.wing.C2(2,end)-geo.wing.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at midboard section
    geo.wing.SELL_midboard = 0.5 *(geo.wing.CR_kink1 + geo.wing.CR_kink2) *geo.wing.span_midboard;
    % Step in midboard
    geo.wing.dy_midboard = geo.wing.bS_midboard/pdcylin.guess.wing.midboard;
    % Node discretization in midboard
    geo.wing.y_midboard = (0 : geo.wing.dy_midboard : geo.wing.bS_midboard)';
    % Structural wing chord in midboard measured along X axis
    geo.wing.rs_midboard = geo.wing.CSR_kink1-geo.wing.y_midboard./geo.wing.bS_midboard*(geo.wing.CSR_kink1-geo.wing.CSR_kink2);
    % Aerodynamic wing chord in midboard measured along X axis
    geo.wing.r_midboard = geo.wing.CR_kink1-geo.wing.y_midboard./geo.wing.bS_midboard*(geo.wing.CR_kink1-geo.wing.CR_kink2);
    
    geo.wing.x_midboard = linspace(geo.wing.C2(1,end-1),geo.wing.C2(1,end),pdcylin.guess.wing.midboard+1)';
    geo.wing.spar_frac_midboard = linspace(alpha(1,1),alpha(2,1),length(geo.wing.rs_midboard))';
	%$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing.Swet_midboard = 0.5 *(geo.wing.r_midboard(1:end-1)+geo.wing.r_midboard(2:end)) .*(geo.wing.span_midboard./pdcylin.guess.wing.midboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    % Structural wing chord in midboard  measured perpendicular to structural chord
%     geo.wing.Zs_midboard = geo.wing.rs_midboard.*cos(geo.wing.lambdaQC_midboard);
    geo.wing.Zs_midboard = geo.wing.rs_midboard.*cos(geo.wing.lambdaC2_midboard);
    
    % Aerodynamic wing chord in midboard  measured perpendicular to structural chord
%     geo.wing.Z_midboard = geo.wing.r_midboard.*cos(geo.wing.lambdaQC_midboard);
    geo.wing.Z_midboard = geo.wing.r_midboard.*cos(geo.wing.lambdaC2_midboard);
    
    % Thickness distribution
    geo.wing.Rt_kink2 = eq_wbox_t(aircraft.wing1.airfoilKink2,[aircraft.fuel.Fore_wing_spar_loc_kin2, aircraft.fuel.Aft_wing_spar_loc_kin2]);
    t_kink2 = geo.wing.CR_kink2 *geo.wing.Rt_kink2;
    geo.wing.tbs_midboard = t_kink1 - geo.wing.y_midboard./geo.wing.bS_midboard *(t_kink1 - t_kink2);

    % CAERO1 parameters
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_midboard];   
    geo.wing.CAERO1.sweepLE  = [geo.wing.CAERO1.sweepLE ; aircraft.wing1.LE_sweep_midboard];      
    geo.wing.CAERO1.sweepQC  = [geo.wing.CAERO1.sweepQC ; aircraft.wing1.quarter_chord_sweep_midboard];    
    geo.wing.CAERO1.sweepC2  = [geo.wing.CAERO1.sweepC2 ; geo.wing.lambdaC2_midboard *180/pi];
    geo.wing.CAERO1.bS       = [geo.wing.CAERO1.bS      ; geo.wing.bS_midboard];
    geo.wing.CAERO1.span     = [geo.wing.CAERO1.span    ; geo.wing.span_midboard];
    geo.wing.CAERO1.SELL     = [geo.wing.CAERO1.SELL    ; geo.wing.SELL_midboard];
    geo.wing.CAERO1.dy       = [geo.wing.CAERO1.dy      ; geo.wing.dy_midboard];
    geo.wing.CAERO1.n        = [geo.wing.CAERO1.n       ; pdcylin.stick.nwing_midboard];
    geo.wing.CAERO1.n_coarse = [geo.wing.CAERO1.n_coarse       ; pdcylin.stick.nwing_midboard_coarse];    
    geo.wing.CAERO1.airfoil  = [geo.wing.CAERO1.airfoil; {aircraft.wing1.airfoilKink1};...
                                                         {aircraft.wing1.airfoilKink2}];
    if aircraft.wing1.flap.present == 1
        % Fraction chord and name of control surface
        kink1c = aircraft.wing1.flap.kink1_chord;
        kink2c = aircraft.wing1.flap.kink2_chord;
        if kink1c > 1
            kink1c = kink1c / 100;
        end
        if kink2c > 1
            kink2c = kink2c / 100;
        end
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; kink1c];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; kink2c];
        % Span fraction set to one
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 1];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; flap2lab];
        
        % AELINK card: set to -1 (flap)
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; -1];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];
    else
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 0];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; '    none'];
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];
    end

    geo.wing.csi_kink2 = aircraft.fuel.Fore_wing_spar_loc_kin2*geo.wing.CR_kink2;
    geo.wing.ni_kink2  = aircraft.fuel.Aft_wing_spar_loc_kin2 *geo.wing.CR_kink2;

else
    
    geo.wing.span_midboard = 0.0;
    geo.wing.CR_kink2      = geo.wing.CR_kink1;
    geo.wing.SELL_midboard = 0.0;
    geo.wing.bS_midboard   = 0.0;
    kink2c = kink1c;
end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% outboard
%--------------------------------------------------------------------------------------------------------------------------
aileronlab = str2_8ch_right(labels{19});

if (geo.wing.span_outboard > MIN)

    if (geo.wing.span_midboard <= MIN)
        
        if (geo.wing.span_inboard <= MIN)
            
            geo.wing.span_outboard = (geo.wing.b/2) - geo.fus.R;
            geo.wing.CR            = geo.wing.CRp - geo.fus.R/geo.wing.span_outboard *(geo.wing.CRp - geo.wing.CT);
            geo.wing.CR_kink1      = geo.wing.CR;
            geo.wing.CR_kink2      = geo.wing.CR;
            geo.wing.CSR           = (aircraft.fuel.Aft_wing_spar_loc_root - aircraft.fuel.Fore_wing_spar_loc_root) *geo.wing.CR;   
            geo.wing.CSR_kink2     = geo.wing.CSR;
            t_kink2                = geo.wing.CR_kink2 *geo.wing.Rt_root;
            
        else
            
            geo.wing.CR_kink2  = geo.wing.CR_kink1;
            geo.wing.CSR_kink2 = geo.wing.CSR_kink1;
            t_kink2 = t_kink1;
            
        end
        
        geo.wing.csi_root  = aircraft.fuel.Fore_wing_spar_loc_root *geo.wing.CR;
        geo.wing.ni_root   = aircraft.fuel.Aft_wing_spar_loc_root  *geo.wing.CR;
        geo.wing.csi_kink1 = aircraft.fuel.Fore_wing_spar_loc_kik1 *geo.wing.CR_kink1;
        geo.wing.ni_kink1  = aircraft.fuel.Aft_wing_spar_loc_kin1  *geo.wing.CR_kink1;
        geo.wing.csi_kink2 = aircraft.fuel.Fore_wing_spar_loc_kin2 *geo.wing.CR_kink2;
        geo.wing.ni_kink2  = aircraft.fuel.Aft_wing_spar_loc_kin2  *geo.wing.CR_kink2;
        
    end
    
    geo.wing.index = [geo.wing.index; geo.wing.index(end) + pdcylin.guess.wing.outboard];
    % Angles
    geo.wing.lambdaQC_outboard = aircraft.wing1.quarter_chord_sweep_outboard*(pi/180);
    geo.wing.lambdaLE_outboard = aircraft.wing1.LE_sweep_outboard*(pi/180);
    geo.wing.dihedral_outboard = aircraft.wing1.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.wing.dinc_outboard = (aircraft.wing1.tip_incidence-aircraft.wing1.kink2_incidence)/pdcylin.guess.wing.outboard;
    if (geo.wing.dinc_outboard ~= 0)
        geo.wing.incidence_outboard = (aircraft.wing1.kink2_incidence:geo.wing.dinc_outboard:aircraft.wing1.tip_incidence)';
    else
        geo.wing.incidence_outboard = aircraft.wing1.kink2_incidence*ones(pdcylin.guess.wing.outboard+1, 1);
    end 
    
    %**********************************************************************************************
    c = geo.wing.CR_kink2;
    b = geo.wing.span_outboard;
    T = geo.wing.CT/c;
    TW(1,1) = aircraft.wing1.kink2_incidence*pi/180;
    TW(2,1) = aircraft.wing1.tip_incidence*pi/180;
    SW = geo.wing.lambdaQC_outboard;
    dihed = geo.wing.dihedral_outboard;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_kin2 + ...
                 (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_tip + ...
                 (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip)/2;
    
    if (geo.wing.span_inboard <= MIN) && (geo.wing.span_midboard <= MIN)
        ox = geo.wing.xQC + geo.fus.R*tan(geo.wing.lambdaQC_outboard) - geo.wing.CR/4;
        oy = 0;
        oz = aircraft.wing1.vertical_location;
        REFx = ox + alpha(1,1) *geo.wing.CR;
        REFy = oy;
        REFz = oz;
    else
        ox = geo.wing.QC( 1,end );
        oy = geo.wing.QC( 2,end );
        oz = geo.wing.QC( 3,end );
        REFx = geo.wing.C2( 1,end );
        REFy = geo.wing.C2( 2,end );
        REFz = geo.wing.C2( 3,end );
    end
    
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz, pdcylin);
    geo.wing.bS_outboard = norm(qc(:,end)-qc(:,end-1));
    geo.wing.WING = [geo.wing.WING wing];
    geo.wing.QC   = [geo.wing.QC qc];
    geo.wing.C2   = [geo.wing.C2 c2];
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************    
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing.lambdaC2_outboard = atan( (geo.wing.C2(1,end)-geo.wing.C2(1,end-1))/(geo.wing.C2(2,end)-geo.wing.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at outboard section
    geo.wing.SELL_outboard = 0.5*(geo.wing.CR_kink2+geo.wing.CT)*geo.wing.span_outboard;
    % Step in outboard
    geo.wing.dy_outboard = geo.wing.bS_outboard/pdcylin.guess.wing.outboard;
    % Node discretization in outboard
    geo.wing.y_outboard = (0 : geo.wing.dy_outboard : geo.wing.bS_outboard)';
    % Structural wing chord in outboard measured along X axis
    geo.wing.rs_outboard = geo.wing.CSR_kink2-geo.wing.y_outboard./geo.wing.bS_outboard*(geo.wing.CSR_kink2-geo.wing.CST);
    % Aerodynamic wing chord in outboard measured along X axis
    geo.wing.r_outboard = geo.wing.CR_kink2-geo.wing.y_outboard./geo.wing.bS_outboard*(geo.wing.CR_kink2-geo.wing.CT);
    
    geo.wing.x_outboard = linspace(geo.wing.C2(1,end-1),geo.wing.C2(1,end),pdcylin.guess.wing.outboard+1)';
    geo.wing.spar_frac_outboard = linspace(alpha(1,1),alpha(2,1),length(geo.wing.rs_outboard))';

	%$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing.Swet_outboard = 0.5 *(geo.wing.r_outboard(1:end-1)+geo.wing.r_outboard(2:end)) .*(geo.wing.span_outboard./pdcylin.guess.wing.outboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    % Structural wing chord in outboard measured perpendicular to structural chord
%     geo.wing.Zs_outboard = geo.wing.rs_outboard.*cos(geo.wing.lambdaQC_outboard);
    geo.wing.Zs_outboard = geo.wing.rs_outboard.*cos(geo.wing.lambdaC2_outboard);
    
    % Aerodynamic wing chord in outboard measured perpendicular to structural chord
%     geo.wing.Z_outboard = geo.wing.r_outboard.*cos(geo.wing.lambdaQC_outboard);
    geo.wing.Z_outboard = geo.wing.r_outboard.*cos(geo.wing.lambdaC2_outboard);
    
    % Thickness distribution
    geo.wing.Rt_tip = eq_wbox_t(aircraft.wing1.airfoilTip,[aircraft.fuel.Fore_wing_spar_loc_tip, aircraft.fuel.Aft_wing_spar_loc_tip]);
    t_tip = geo.wing.CT *geo.wing.Rt_tip;
    geo.wing.tbs_outboard = t_kink2 - geo.wing.y_outboard./geo.wing.bS_outboard*(t_kink2 - t_tip);    

    % CAERO1 parameters    
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_outboard]; 
    geo.wing.CAERO1.sweepLE  = [geo.wing.CAERO1.sweepLE ; aircraft.wing1.LE_sweep_outboard];    
    geo.wing.CAERO1.sweepQC  = [geo.wing.CAERO1.sweepQC; aircraft.wing1.quarter_chord_sweep_outboard];    
    geo.wing.CAERO1.sweepC2  = [geo.wing.CAERO1.sweepC2 ; geo.wing.lambdaC2_outboard *180/pi];
    geo.wing.CAERO1.bS       = [geo.wing.CAERO1.bS; geo.wing.bS_outboard];
    geo.wing.CAERO1.span     = [geo.wing.CAERO1.span; geo.wing.span_outboard];
    geo.wing.CAERO1.SELL     = [geo.wing.CAERO1.SELL; geo.wing.SELL_outboard];    
    geo.wing.CAERO1.dy       = [geo.wing.CAERO1.dy; geo.wing.dy_outboard];
    geo.wing.CAERO1.n        = [geo.wing.CAERO1.n; pdcylin.stick.nwing_outboard];
    geo.wing.CAERO1.n_coarse = [geo.wing.CAERO1.n_coarse; pdcylin.stick.nwing_outboard_coarse];
    geo.wing.CAERO1.airfoil  = [geo.wing.CAERO1.airfoil; {aircraft.wing1.airfoilKink2};
                                                         {aircraft.wing1.airfoilTip}];
    
    if aircraft.wing1.aileron.present == 1                                                 
        % Fraction chord and name of control surface
        frc = aircraft.wing1.aileron.chord;
        if frc > 1
            frc = frc/100;
        end
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; frc];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; frc];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; frc];
        % Span fraction
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; aircraft.wing1.aileron.Span];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; aileronlab];
        
        % AELINK card: set to 1 (aileron)
        if aircraft.Horizontal_tail.present
           geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 1];
           geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;aircraft.wing1.aileron.position];
        else
           geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
           geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;aircraft.wing1.aileron.position];
        end
    else
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 0];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; '    none'];
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];
    end

else 
    
    geo.wing.span_outboard = 0.0;
    geo.wing.bS_outboard   = 0.0;
    geo.wing.SELL_outboard = 0.0;

end
%--------------------------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------------------------
% winglet
%--------------------------------------------------------------------------------------------------------------------------
if (aircraft.winglet.present == 1) && aircraft.winglet.Span>0
    
    
    
    geo.wing.index = [geo.wing.index; geo.wing.index(end) + pdcylin.guess.wing.winglet];
    % Angles
    geo.wing.lambdaQC_winglet = atan2(aircraft.winglet.Span*tan(aircraft.winglet.LE_sweep*pi/180) + 0.25*(aircraft.winglet.taper_ratio-1)*geo.wing.CT , aircraft.winglet.Span);
    geo.wing.lambdaLE_winglet = aircraft.winglet.LE_sweep*(pi/180);
    geo.wing.dihedral_winglet = aircraft.winglet.Cant_angle*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.wing.dinc_winglet= (aircraft.winglet.tip_incidence-aircraft.winglet.root_incidence)/pdcylin.guess.wing.winglet;
    if (geo.wing.dinc_winglet ~= 0)
        geo.wing.incidence_winglet = (aircraft.winglet.root_incidence:geo.wing.dinc_winglet:aircraft.winglet.tip_incidence)';
    else
        geo.wing.incidence_winglet = aircraft.winglet.root_incidence*ones(pdcylin.guess.wing.winglet+1, 1);
    end 
    
    %**********************************************************************************************
    c = geo.wing.CT;
    b = aircraft.winglet.Span;
    T = aircraft.winglet.taper_ratio;
    TW(1,1) = aircraft.winglet.root_incidence*pi/180;
    TW(2,1) = aircraft.winglet.tip_incidence*pi/180;
    SW = geo.wing.lambdaQC_winglet;
    dihed = geo.wing.dihedral_winglet;
%    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_kin2 + ...
%                 (aircraft.fuel.Aft_wing_spar_loc_kin2 - aircraft.fuel.Fore_wing_spar_loc_kin2)/2;
    alpha(1,1) = aircraft.fuel.Fore_wing_spar_loc_tip + ...
                 (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip)/2;
    alpha(2,1) = aircraft.fuel.Fore_wing_spar_loc_tip + ...
                 (aircraft.fuel.Aft_wing_spar_loc_tip - aircraft.fuel.Fore_wing_spar_loc_tip)/2;
    
    
        ox = geo.wing.QC( 1,end );
        oy = geo.wing.QC( 2,end );
        oz = geo.wing.QC( 3,end );
        REFx = geo.wing.C2( 1,end );
        REFy = geo.wing.C2( 2,end );
        REFz = geo.wing.C2( 3,end );
    
    
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz, pdcylin);
    geo.wing.bS_winglet = norm(qc(:,end)-qc(:,end-1));
    geo.wing.WING = [geo.wing.WING wing];
    geo.wing.QC   = [geo.wing.QC qc];
    geo.wing.C2   = [geo.wing.C2 c2];
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************    
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.wing.lambdaC2_winglet = atan( (geo.wing.C2(1,end)-geo.wing.C2(1,end-1))/(geo.wing.C2(2,end)-geo.wing.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at winglet section
    geo.wing.SELL_winglet = 0.5*(geo.wing.CT + geo.wing.CT*T)*aircraft.winglet.Span;
    % Step in winglet
    geo.wing.dy_winglet = geo.wing.bS_winglet/pdcylin.guess.wing.winglet;
    % Node discretization in winglet
    geo.wing.y_winglet = (0 : geo.wing.dy_winglet : geo.wing.bS_winglet)';
    % Structural wing chord in winglet measured along X axis
    geo.wing.rs_winglet = geo.wing.CST-geo.wing.y_winglet./geo.wing.bS_winglet*(geo.wing.CST-geo.wing.CST*T);
    % Aerodynamic wing chord in winglet measured along X axis
    geo.wing.r_winglet = geo.wing.CT-geo.wing.y_winglet./geo.wing.bS_winglet*(geo.wing.CT-geo.wing.CT*T);
    
    geo.wing.x_winglet = linspace(geo.wing.C2(1,end-1),geo.wing.C2(1,end),pdcylin.guess.wing.winglet+1)';
    geo.wing.spar_frac_winglet = linspace(alpha(1,1),alpha(2,1),length(geo.wing.rs_winglet))';

	%$$$$$$$$$$$$$$$$$$$$$$$
    geo.wing.Swet_winglet = 0.5 *(geo.wing.r_winglet(1:end-1)+geo.wing.r_winglet(2:end)) .*(aircraft.winglet.Span./pdcylin.guess.wing.winglet);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    
    % Structural wing chord in winglet measured perpendicular to structural chord
%     geo.wing.Zs_winglet = geo.wing.rs_winglet.*cos(geo.wing.lambdaQC_winglet);
    geo.wing.Zs_winglet = geo.wing.rs_winglet.*cos(geo.wing.lambdaC2_winglet);
    
    % Aerodynamic wing chord in outboard measured perpendicular to structural chord
%     geo.wing.Z_outboard = geo.wing.r_outboard.*cos(geo.wing.lambdaQC_outboard);
    geo.wing.Z_winglet = geo.wing.r_winglet.*cos(geo.wing.lambdaC2_winglet);
    
    % Thickness distribution
    t_tipw = geo.wing.CT *geo.wing.Rt_tip;
    geo.wing.tbs_winglet = t_tip - geo.wing.y_winglet./geo.wing.bS_winglet*(t_tip - t_tipw);    

    % CAERO1 parameters    
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.winglet.Cant_angle]; 
    geo.wing.CAERO1.sweepLE  = [geo.wing.CAERO1.sweepLE ; aircraft.winglet.LE_sweep];    
    geo.wing.CAERO1.sweepQC  = [geo.wing.CAERO1.sweepQC; geo.wing.lambdaQC_winglet*180/pi];    
    geo.wing.CAERO1.sweepC2  = [geo.wing.CAERO1.sweepC2 ; geo.wing.lambdaC2_winglet *180/pi];
    geo.wing.CAERO1.bS       = [geo.wing.CAERO1.bS; geo.wing.bS_winglet];
    geo.wing.CAERO1.span     = [geo.wing.CAERO1.span; aircraft.winglet.Span];
    geo.wing.CAERO1.SELL     = [geo.wing.CAERO1.SELL; geo.wing.SELL_winglet];    
    geo.wing.CAERO1.dy       = [geo.wing.CAERO1.dy; geo.wing.dy_winglet];
    geo.wing.CAERO1.n        = [geo.wing.CAERO1.n; pdcylin.stick.nwing_winglet];
    geo.wing.CAERO1.n_coarse = [geo.wing.CAERO1.n_coarse; pdcylin.stick.nwing_winglet_coarse];
    geo.wing.CAERO1.airfoil  = [geo.wing.CAERO1.airfoil; {aircraft.wing1.airfoilTip};
                                                         {aircraft.wing1.airfoilTip}];
    
    % Fraction chord and name of control surface
%     frc = aircraft.wing1.aileron.chord;
%     if frc > 1
%         frc = frc/100;
%     end
    geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    % Span fraction
    geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 0];
    geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; '    none'];
    
    % AELINK card: set to 1 (aileron)
    geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
    
  
    geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
    geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];


end

%--------------------------------------------------------------------------------------------------------------------------
% Save inboard and outboard parameters refering to correspondent parameter
%--------------------------------------------------------------------------------------------------------------------------

% From inboard
if (geo.wing.span_inboard > MIN)    
    
    geo.wing.y         = geo.wing.y_inboard;
    geo.wing.rs        = geo.wing.rs_inboard;
    geo.wing.r         = geo.wing.r_inboard;
    geo.wing.Zs        = geo.wing.Zs_inboard;
    geo.wing.Z         = geo.wing.Z_inboard;
    geo.wing.tbs       = geo.wing.tbs_inboard;
    geo.wing.incidence = geo.wing.incidence_inboard;    
    geo.wing.dy        = geo.wing.dy_inboard .*ones( pdcylin.guess.wing.inboard+1, 1);
    geo.wing.Swet      = geo.wing.Swet_inboard;
    geo.wing.x         = geo.wing.x_inboard;
    geo.wing.spar_frac = geo.wing.spar_frac_inboard;
    
end

% From midboard
if (geo.wing.span_midboard > MIN)    
    
    if (geo.wing.span_inboard > MIN)    
        
        geo.wing.y         = [geo.wing.y        ; geo.wing.y(end)+geo.wing.y_midboard(2:end)];
        geo.wing.rs        = [geo.wing.rs       ; geo.wing.rs_midboard(2:end)];
        geo.wing.r         = [geo.wing.r        ; geo.wing.r_midboard(2:end)];
        geo.wing.Zs        = [geo.wing.Zs       ; geo.wing.Zs_midboard(2:end)];
        geo.wing.Z         = [geo.wing.Z        ; geo.wing.Z_midboard(2:end)];
        geo.wing.tbs       = [geo.wing.tbs      ; geo.wing.tbs_midboard(2:end)];
        geo.wing.incidence = [geo.wing.incidence; geo.wing.incidence_midboard(2:end)];        
        geo.wing.dy        = [geo.wing.dy       ; geo.wing.dy_midboard .*ones( pdcylin.guess.wing.midboard, 1)];
        geo.wing.Swet      = [geo.wing.Swet; geo.wing.Swet_midboard];
        geo.wing.x         = [geo.wing.x        ; geo.wing.x_midboard(2:end)];
        geo.wing.spar_frac = [geo.wing.spar_frac; geo.wing.spar_frac_midboard(2:end)];
        
    else
        
        geo.wing.y         = geo.wing.y_midboard;
        geo.wing.rs        = geo.wing.rs_midboard;
        geo.wing.r         = geo.wing.r_midboard;
        geo.wing.Zs        = geo.wing.Zs_midboard;
        geo.wing.Z         = geo.wing.Z_midboard;
        geo.wing.tbs       = geo.wing.tbs_midboard;
        geo.wing.incidence = geo.wing.incidence_midboard;        
        geo.wing.dy        = geo.wing.dy_midboard .*ones( pdcylin.guess.wing.midboard+1, 1);
        geo.wing.Swet      = geo.wing.Swet_midboard;
        geo.wing.x         = geo.wing.x_midboard;
        geo.wing.spar_frac = geo.wing.spar_frac_midboard;

    end 
    
end

% From outboard
if (geo.wing.span_outboard > MIN)    
    
    if (isempty(geo.wing.y)~=1)        
        
        geo.wing.y         = [geo.wing.y        ; geo.wing.y(end)+geo.wing.y_outboard(2:end)];
        geo.wing.rs        = [geo.wing.rs       ; geo.wing.rs_outboard(2:end)];
        geo.wing.r         = [geo.wing.r        ; geo.wing.r_outboard(2:end)];
        geo.wing.Zs        = [geo.wing.Zs       ; geo.wing.Zs_outboard(2:end)];
        geo.wing.Z         = [geo.wing.Z        ; geo.wing.Z_outboard(2:end)];
        geo.wing.tbs       = [geo.wing.tbs      ; geo.wing.tbs_outboard(2:end)];  
        geo.wing.incidence = [geo.wing.incidence; geo.wing.incidence_outboard(2:end)];
        geo.wing.dy        = [geo.wing.dy       ; geo.wing.dy_outboard .*ones( pdcylin.guess.wing.outboard, 1)];
        geo.wing.Swet      = [geo.wing.Swet; geo.wing.Swet_outboard];
        geo.wing.x         = [geo.wing.x        ; geo.wing.x_outboard(2:end)];
        geo.wing.spar_frac = [geo.wing.spar_frac; geo.wing.spar_frac_outboard(2:end)];

    else
        
        geo.wing.y         = geo.wing.y_outboard;
        geo.wing.rs        = geo.wing.rs_outboard;
        geo.wing.r         = geo.wing.r_outboard;
        geo.wing.Zs        = geo.wing.Zs_outboard;
        geo.wing.Z         = geo.wing.Z_outboard;
        geo.wing.tbs       = geo.wing.tbs_outboard;      
        geo.wing.incidence = geo.wing.incidence_outboard;        
        geo.wing.dy        = geo.wing.dy_outboard .*ones( pdcylin.guess.wing.outboard+1, 1);
        geo.wing.Swet      = geo.wing.Swet_outboard;
        geo.wing.x         = geo.wing.x_outboard;
        geo.wing.spar_frac = geo.wing.spar_frac_outboard;
    end 
    
end

if (aircraft.winglet.present == 1) && aircraft.winglet.Span>0
    geo.wing.y         = [geo.wing.y        ; geo.wing.y(end)+geo.wing.y_winglet(2:end)];
    geo.wing.rs        = [geo.wing.rs       ; geo.wing.rs_winglet(2:end)];
    geo.wing.r         = [geo.wing.r        ; geo.wing.r_winglet(2:end)];
    geo.wing.Zs        = [geo.wing.Zs       ; geo.wing.Zs_winglet(2:end)];
    geo.wing.Z         = [geo.wing.Z        ; geo.wing.Z_winglet(2:end)];
    geo.wing.tbs       = [geo.wing.tbs      ; geo.wing.tbs_winglet(2:end)];
    geo.wing.incidence = [geo.wing.incidence; geo.wing.incidence_winglet(2:end)];
    geo.wing.dy        = [geo.wing.dy       ; geo.wing.dy_winglet .*ones( pdcylin.guess.wing.winglet, 1)];
    geo.wing.Swet      = [geo.wing.Swet; geo.wing.Swet_winglet];
    geo.wing.x         = [geo.wing.x        ; geo.wing.x_winglet(2:end)];
    geo.wing.spar_frac = [geo.wing.spar_frac; geo.wing.spar_frac_winglet(2:end)];

end

% Wetted surface vector [n_bar x 1]: account for upper and lower surface
geo.wing.Swet = 2*geo.wing.Swet;
%--------------------------------------------------------------------------------------------------------------------------


% Convert incidence angle
geo.wing.incidence = geo.wing.incidence.*(pi/180);
% Exposed area for total semi-wing
geo.wing.SELL = geo.wing.SELL_inboard + geo.wing.SELL_midboard + geo.wing.SELL_outboard;

% Number of nodes used to discretize structural semi-wing
geo.wing.leny = length(geo.wing.y);
% Carrythrough thickness
geo.wing.tcs  = max(geo.wing.tbs);
% Length of structural line as summation of each single sector
if (aircraft.winglet.present == 1) && aircraft.winglet.Span>0
    geo.wing.SELL = geo.wing.SELL + geo.wing.SELL_winglet;
end
geo.wing.bS   = geo.wing.bS_inboard + geo.wing.bS_midboard + geo.wing.bS_outboard;

%**************************************************************************************************
% MAC longitudinal location respect to fuselage nose
macYnondim = ((aircraft.Reference_wing.non_dim_MAC_y_bar*geo.wing.b/2)-geo.fus.R)/(geo.wing.b/2-geo.fus.R);
MAC_y = macYnondim*geo.wing.bS;
indMAC_y = find( MAC_y >= geo.wing.y );
indsect_MAC = find( indMAC_y(end) >= geo.wing.index );
geo.wing.MAC_x = geo.wing.QC(1,indsect_MAC(end)) +...
                 ( MAC_y-geo.wing.QC(2,indsect_MAC(end)) )*tan(geo.wing.CAERO1.sweepQC(indsect_MAC(end))*pi/180);
geo.wing.MAC_y = MAC_y; 
%**************************************************************************************************

%--------------------------------------------------------------------------------------------------------------------------
% Complete CAERO1 quantities: chord / taper / incidence
%--------------------------------------------------------------------------------------------------------------------------

geo.wing.CAERO1.chord     = [geo.wing.CAERO1.chord; geo.wing.CR];
geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence; aircraft.wing1.root_incidence];
%
if ( ((geo.wing.span_inboard>MIN)&&(geo.wing.span_midboard>MIN)) || ((geo.wing.span_inboard>MIN)&&(geo.wing.span_outboard>MIN)) )
    geo.wing.CAERO1.chord     = [geo.wing.CAERO1.chord; geo.wing.CR_kink1];
    geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence; aircraft.wing1.kink1_incidence];
end
%
if (geo.wing.span_midboard > MIN) && (geo.wing.span_outboard > MIN)
    geo.wing.CAERO1.chord     = [geo.wing.CAERO1.chord; geo.wing.CR_kink2];
    geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence; aircraft.wing1.kink2_incidence];
end
%
geo.wing.CAERO1.chord     = [geo.wing.CAERO1.chord; geo.wing.CT];
geo.wing.CAERO1.taper     = (geo.wing.CAERO1.chord(2:end)./geo.wing.CAERO1.chord(1:end-1));
geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence; aircraft.wing1.tip_incidence];


if (aircraft.winglet.present == 1) && aircraft.winglet.Span>0
    geo.wing.CAERO1.chord     = [geo.wing.CAERO1.chord; geo.wing.CT*aircraft.winglet.taper_ratio];
    geo.wing.CAERO1.taper     = (geo.wing.CAERO1.chord(2:end)./geo.wing.CAERO1.chord(1:end-1));
    geo.wing.CAERO1.incidence = [geo.wing.CAERO1.incidence; aircraft.winglet.tip_incidence];
end


%*****************************************
if isequal(pdcylin.stick.model.symmXZ, 1)
    nrsp = length(geo.wing.CAERO1.taper);
    for i = 1:nrsp
        % copy the name
        geo.wing.CAERO1.sup_control.nme(i+nrsp,:) = geo.wing.CAERO1.sup_control.nme(i,:);
        % change the last letter
        geo.wing.CAERO1.sup_control.nme(i+nrsp,end) = 'l';
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

if pdcylin.wing.kcon <=6 

    geo.wing.ep  = wcoef( pdcylin.wing.kcon, 1 );
    geo.wing.e   = wcoef( pdcylin.wing.kcon, 2 );
    geo.wing.epc = wcoef( pdcylin.wing.kcon, 3 );
    geo.wing.ec  = wcoef( pdcylin.wing.kcon, 4 );
    geo.wing.epw = wcoef( pdcylin.wing.kcon, 5 );
    geo.wing.Kgc = wcoef( pdcylin.wing.kcon, 6 );
    geo.wing.Kgw = wcoef( pdcylin.wing.kcon, 7 );

else
    geo.wing.ep  = 0;
    geo.wing.e   = 0;
    geo.wing.epc = 0;
    geo.wing.ec  = 0;
    geo.wing.epw = 0;
    geo.wing.Kgc = 0;
    geo.wing.Kgw = 0;
    
end
%
%--------------------------------------------------------------------------------------------------------------------------
