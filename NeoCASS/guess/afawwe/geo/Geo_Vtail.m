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
%     091015      1.3.7   L. Travaglini    Modification
%     091028      1.3.7   A. De Gaspari    Modification
%
% Modification by Travaglini: adding controls on distribution of points and on
% span geometry
%*******************************************************************************
function [geo,pdcylin] = Geo_Vtail(pdcylin, aircraft, geo)

%---------------------------------------------------------------------------------------------------------------------------
% Initialize structure
%---------------------------------------------------------------------------------------------------------------------------
geo.vtail.index             = [];   % index recognising end of sectors, coincident with kinks
%
geo.vtail.xLE               = [];   % LE location along x-coordinate                                        [m], scalar
geo.vtail.xQC               = [];   % QC location along x-coordinate                                        [m], scalar
geo.vtail.xTE               = [];   % TE location along x-coordinate                                        [m], scalar
geo.vtail.zLE               = [];   % LE location along z-coordinate                                        [m], scalar
%
geo.vtail.SP                = [];   % planform area of vertical tail                                        [m2] , scalar
geo.vtail.b                 = [];   % span of vertical tail                                                 [m]  , scalar
geo.vtail.span_inboard      = [];   % Span of inboard sector, measured parallel to global Z coordinate
geo.vtail.span_outboard     = [];   % Span of outboard sector, measured parallel to global Z coordinate
geo.vtail.CR                = [];   % root vert. tail chord                                                 [m]  , scalar
geo.vtail.CR_kink           = [];   % vert. tail chord at kink                                              [m]  , scalar
geo.vtail.CT                = [];   % tip chord                                                             [m]  , scalar
%
geo.vtail.lambdaQC_inboard  = [];   % QC sweep angle at inboard section                                     [rad], scalar
geo.vtail.lambdaQC_outboard = [];   % QC sweep angle at outboard section                                    [rad], scalar
% 
geo.vtail.lambdaC2_inboard = [];    % C2 sweep angle at inboard section                                     [rad], scalar
geo.vtail.lambdaC2_outboard = [];   % C2 sweep angle at outboard section                                    [rad], scalar
% 
geo.vtail.lambdaLE_inboard  = [];   % LE sweep angle at inboard section                                     [rad], scalar
geo.vtail.lambdaLE_outboard = [];   % LE sweep angle at outboard section                                    [rad], scalar
%
geo.vtail.lambdaC2_inboard  = [];   % C2 sweep angle at inboard section                                     [rad], scalar
geo.vtail.lambdaC2_outboard = [];   % C2 sweep angle at outboard section                                    [rad], scalar
%
geo.vtail.dihedral_inboard  = [];   % dihedral angle at inboard section                                     [rad], scalar
geo.vtail.dihedral_outboard = [];   % dihedral angle at inboard section                                     [rad], scalar
%

geo.vtail.bS_inboard        = [];   % sempispan on quarter-chord line at inboard section                    [m]  , scalar
geo.vtail.bS_outboard       = [];   % sempispan on quarter-chord line at outboard section                   [m]  , scalar
geo.vtail.bS                = [];   % sempispan on quarter-chord line                                       [m]  , scalar
%
geo.vtail.SELL_inboard      = [];   % exposed area at inboard section                                       [m2] , scalar
geo.vtail.SELL_outboard     = [];   % exposed area at outboard section                                      [m2] , scalar
geo.vtail.SELL              = [];   % exposed area for semiwing                                             [m2] , scalar
geo.vtail.Swet_inboard      = [];   % exposed area at inboard section                                       [m2] , vector
geo.vtail.Swet_outboard     = [];   % exposed area at outboard section                                      [m2] , vector
geo.vtail.Swet              = [];   % exposed area for semiwing                                             [m2] , vector
%
geo.vtail.Rt_root           = [];   % thickness ratio at wing root                                          [-]  , scalar
geo.vtail.Rt_kink           = [];   % thickness ratio at kink                                               [-]  , scalar
geo.vtail.Rt_tip            = [];   % thickness ratio at wing tip                                           [-]  , scalar
%
geo.vtail.dy_inboard        = [];   % discretization in inboard section                                     [m]  , scalar
geo.vtail.dy_outboard       = [];   % discretization in outboard section                                    [m]  , scalar
geo.vtail.dy                = [];   % step sector by sector                                                 [m]  , vector
geo.vtail.y_inboard         = [];   % discretization along inboard section                                  [m]  , vector
geo.vtail.y_outboard        = [];   % discretization along outboard section                                 [m]  , vector
geo.vtail.y                 = [];   % discretization along structural vert. tail semispan                   [m]  , vector
geo.vtail.leny              = [];   % nodes along structural wing semispan                                  [-]  , scalar
%
geo.vtail.r_inboard         = [];   % total chord // to vehicle long. axis inboard                          [m]  , vector
geo.vtail.r_outboard        = [];   % total chord // to vehicle long. axis outboard                         [m]  , vector
geo.vtail.r                 = [];   % total chord // to vehicle long. axis                                  [m]  , vector
geo.vtail.rs_inboard         = [];
geo.vtail.rs_outboard        = [];
geo.vtail.rs                 = [];

geo.vtail.Z_inboard         = [];   % total chord |- to struct. semispan inboard                            [m]  , vector
geo.vtail.Z_outboard        = [];   % total chord |- to struct. semispan outboard                           [m]  , vector
geo.vtail.Z                 = [];   % total chord |- to struct. semispan                                    [m]  , vector
geo.vtail.Zs_inboard         = [];
geo.vtail.Zs_outboard        = [];
geo.vtail.Zs                 = [];
%
geo.vtail.tbs_inboard       = [];   % thickness vert. tail box inboard                                      [-]  , vector
geo.vtail.tbs_outboard      = [];   % thickness vert. tail box outboard                                     [-]  , vector
geo.vtail.tbs               = [];   % thickness vert. tail box                                              [-]  , vector
geo.vtail.tcs               = [];   % thickness in the carrythrough structure                               [m]  , scalar
%
geo.vtail.epc               = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.e                 = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.ep                = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.ec                = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.epw               = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.Kgc               = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.vtail.Kgw               = [];   % vert. tail structural coefficients & exponents                        [-]  , scalar
%
geo.vtail.spar_frac_inboard = [];
geo.vtail.spar_frac_outboard = [];
geo.vtail.spar_frac = [];
%
geo.vtail.n                 = [];   % vector containing, for each sector with non-zero span, number of elements 
%
geo.vtail.CAERO1.chord      = [];   % chord for each defined sector                                          [m]  , vector
geo.vtail.CAERO1.span       = [];   % span for each defined sector                                           [m]  , vector
geo.vtail.CAERO1.taper      = [];   % taper for each defined sector                                          [-]  , vector
geo.vtail.CAERO1.bS         = [];   % contains bS for each defined sectors                                   [m]  , vector
geo.vtail.CAERO1.dy         = [];   % step for each defined sector                                           [m]  , scalar
geo.vtail.CAERO1.dihedral   = [];   % dihedral for each defined sector                                       [deg], vector
geo.vtail.CAERO1.sweepLE    = [];   % LE sweep angle for each defined sector                                 [deg], vector
geo.vtail.CAERO1.sweepQC    = [];   % QC sweep angle for each defined sector                                 [deg], vector
geo.vtail.CAERO1.sweepC2    = [];   % C2 sweep angle for each defined sector                                 [deg], vector
geo.vtail.CAERO1.incidence  = [];   % incidence angle for each defined sector                                [deg], vector
geo.vtail.CAERO1.n          = [];   % number of elements for each defined sector                             [-]  , vector
geo.vtail.CAERO1.n_coarse   = [];   % number of elements for each defined sector                             [-]  , vector
geo.vtail.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                 [-]  , vector
geo.vtail.CAERO1.sup_control.nme = [];% Name of control surface                                                [-]  , vector%
geo.vtail.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.vtail.CAERO1.airfoil    = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
%
geo.vtail.csi_root          = [];
geo.vtail.ni_root           = [];
geo.vtail.csi_kink          = [];
geo.vtail.ni_kink           = [];
geo.vtail.csi_tip           = [];
geo.vtail.ni_tip            = [];
%
%**************************************************************************
geo.vtail.WING              = [];   % points defining each panel (3x(4*nr_sector))
geo.vtail.QC                = [];   % points defining quarter chord line (3x(2*nr_sector))
geo.vtail.C2                = [];   % points defining elastic line (3x(2*nr_sector))
geo.vtail.PANE              = [];   % points to write CAERO1 card
geo.vtail.x_nodes           = [];
geo.vtail.x_nodes_1_2       = [];
%**************************************************************************
%
geo.vtail.V                 = []; 
geo.vtail.cg                = [];
%
lab = TRIMlabels;
labels = lab(:, 1);
%---------------------------------------------------------------------------------------------------------------------------
geo.vtail.intersect = 'none';
%---------------------------------------------------------------------------------------------------------------------------
% Basic calculations
%---------------------------------------------------------------------------------------------------------------------------

% 
% planform area of vertical tail
geo.vtail.SP = aircraft.Vertical_tail.area;
% span of vertical tail
geo.vtail.b = aircraft.Vertical_tail.span;
% Span of inboard sector, measured parallel to global Z coordinate
geo.vtail.span_inboard = aircraft.Vertical_tail.spanwise_kink*geo.vtail.b;
% Span of outboard sector, measured parallel to global Z coordinate
geo.vtail.span_outboard = aircraft.Vertical_tail.span - geo.vtail.span_inboard;

% Check on different span, want to obtain a uniform distribution of node
% both on guess model and smart model.
MIN = 0.01*aircraft.Vertical_tail.span; 

if ~isfield(pdcylin.guess,'check')
%  
  Ltot = aircraft.Vertical_tail.span;
  Ls = [geo.vtail.span_inboard , geo.vtail.span_outboard];
  index = find(Ls<MIN);
  n = ceil( Ls * (pdcylin.guess.vert.inboard + pdcylin.guess.vert.outboard )/Ltot );
  n(index) = 0;
  fprintf('\n\t\t- Vtail computational mesh: %d, %d elements.', n(1), n(2));
  m = ceil( Ls * (pdcylin.stick.nvtail_inboard + pdcylin.stick.nvtail_outboard )/Ltot );
  m(index) = 0;
  m2 = ceil( Ls * (pdcylin.stick.nvtail_inboard_coarse +  pdcylin.stick.nvtail_outboard_coarse )/Ltot );
  m2(index) = 0;
  fprintf('\n\t\t- Vtail stick mesh: %d, %d elements.', m2(1), m2(2));
  ny = ceil( Ls * (pdcylin.stick.ny.vert_inboard  + pdcylin.stick.ny.vert_outboard )/Ltot );
  ny(index) = 0;
  fprintf('\n\t\t- Vtail spanwise aerodynamic mesh: %d, %d elements.', ny(1), ny(2));
  pdcylin.guess.vert.inboard = n(1);
  pdcylin.guess.vert.outboard = n(2);

  pdcylin.stick.nvtail_inboard = m(1);
  pdcylin.stick.nvtail_outboard = m(2);

  pdcylin.stick.nvtail_inboard_coarse = m2(1);
  pdcylin.stick.nvtail_outboard_coarse = m2(2);

  pdcylin.stick.ny.vert_inboard = ny(1);
  pdcylin.stick.ny.vert_outboard = ny(2);
end

% Root chord
geo.vtail.CR = 2*geo.vtail.SP/(geo.vtail.b*( (1+aircraft.Vertical_tail.taper_kink)*aircraft.Vertical_tail.spanwise_kink +...
               (aircraft.Vertical_tail.taper_kink + aircraft.Vertical_tail.taper_tip)*(1 - aircraft.Vertical_tail.spanwise_kink)));
% Struct. chord at tip
geo.vtail.CSR = (pdcylin.spar.VT.Aft_VT_spar_loc_root - pdcylin.spar.VT.Fore_VT_spar_loc_root) * geo.vtail.CR;
           
% LE location along x-coordinate
geo.vtail.xLE = aircraft.Vertical_tail.longitudinal_location;
% QC location along x-coordinate
geo.vtail.xQC = geo.vtail.xLE + 1/4*geo.vtail.CR;
% TE location along x-coordinate
geo.vtail.xTE = geo.vtail.xLE +geo.vtail.CR;
% LE location along z-coordinate for vertical tail
geo.vtail.zLE = aircraft.Vertical_tail.vertical_location;
% Tip chord
geo.vtail.CT = geo.vtail.CR*aircraft.Vertical_tail.taper_tip;

geo.vtail.CST = geo.vtail.CT * (pdcylin.spar.VT.Aft_VT_spar_loc_tip - pdcylin.spar.VT.Fore_VT_spar_loc_tip);

% Thickness ratio at root
%geo.vtail.Rt_root = aircraft.Vertical_tail.thickness_root;
geo.vtail.Rt_root = eq_wbox_t(aircraft.Vertical_tail.airfoilRoot,[pdcylin.spar.VT.Aft_VT_spar_loc_root, pdcylin.spar.VT.Fore_VT_spar_loc_root]);
t_root = geo.vtail.Rt_root *geo.vtail.CR;
%
%**************************************************************************
% Initialize the position of the first node
geo.vtail.index = 1;
% define fuel storege / structural box dimensions as fraction of the local aerodynamic chord;
% if values are equal to zeros and local chord, structural box is set to coincide with aerodynamic chord; 
geo.vtail.csi_root = geo.vtail.CR *pdcylin.spar.VT.Fore_VT_spar_loc_root;
geo.vtail.ni_root = geo.vtail.CR *pdcylin.spar.VT.Aft_VT_spar_loc_root;
geo.vtail.csi_tip = geo.vtail.CT *pdcylin.spar.VT.Fore_VT_spar_loc_kink;
geo.vtail.ni_tip  = geo.vtail.CT *pdcylin.spar.VT.Aft_VT_spar_loc_kink;
%**************************************************************************
%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% inboard
%---------------------------------------------------------------------------------------------------------------------------
rudlab = str2_8ch_right(labels{21});

if (geo.vtail.span_inboard > MIN)
    
    % Update index for node at outer position-inboard sector
    geo.vtail.index = [geo.vtail.index; geo.vtail.index(end) + pdcylin.guess.vert.inboard];
    % Structural chord at kink
    geo.vtail.CR_kink = geo.vtail.CR *aircraft.Vertical_tail.taper_kink;    
    geo.vtail.CSR_kink = (pdcylin.spar.VT.Aft_VT_spar_loc_kink - pdcylin.spar.VT.Fore_VT_spar_loc_kink) *geo.vtail.CR_kink;
    % Angles
    geo.vtail.lambdaQC_inboard = atan2(geo.vtail.CR_kink*0.25+tan(aircraft.Vertical_tail.LE_sweep_inboard*pi/180)*geo.vtail.span_inboard-geo.vtail.CR*0.25,geo.vtail.span_inboard);
    %     geo.vtail.lambdaQC_inboard =
    %     aircraft.Vertical_tail.quarter_chord_sweep_inboard *(pi/180);
    aircraft.Vertical_tail.quarter_chord_sweep_inboard = geo.vtail.lambdaQC_inboard*180/pi;
    geo.vtail.lambdaLE_inboard = aircraft.Vertical_tail.LE_sweep_inboard *(pi/180);
    %
    geo.vtail.csi_kink = geo.vtail.CR_kink *pdcylin.spar.VT.Fore_VT_spar_loc_kink;
    geo.vtail.ni_kink = geo.vtail.CR_kink *pdcylin.spar.VT.Aft_VT_spar_loc_kink;
    geo.vtail.dihedral_inboard = (90+aircraft.Vertical_tail.dihedral_inboard)*(pi/180);
    %
    %**********************************************************************************************
    c = geo.vtail.CR;
    b = geo.vtail.span_inboard;
    T = geo.vtail.CR_kink/c;
    TW(1,1) = 0 *pi/180;
    TW(2,1) = 0 *pi/180;
    SW = geo.vtail.lambdaQC_inboard;
    dihed = geo.vtail.dihedral_inboard;
    ox = geo.vtail.xLE;
    oy = 0;
    oz = geo.vtail.zLE;
    alpha(1,1) = (geo.vtail.csi_root + (geo.vtail.ni_root - geo.vtail.csi_root)/2)/geo.vtail.CR;
    alpha(2,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
    REFx = ox + alpha(1,1) *geo.vtail.CR;
    REFy = oy;
    REFz = oz;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.vtail.bS_inboard = norm(qc(:,2) - qc(:,1));
    geo.vtail.WING = [geo.vtail.WING wing];
    geo.vtail.QC   = [geo.vtail.QC qc];
    geo.vtail.C2   = [geo.vtail.C2 c2];
    geo.vtail.PANE = [geo.vtail.PANE pane];
    % Enforce y-coordinates to be zeros 'coz of numerical noise
%     if dihed *180/pi == 90
%         geo.vtail.WING(2,:) = 0.0;
%         geo.vtail.QC(2,:)   = 0.0;
%         geo.vtail.C2(2,:)   = 0.0;
%         geo.vtail.PANE(2,:) = 0.0;
%     end
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.vtail.lambdaC2_inboard = atan( (geo.vtail.C2(1,end)-geo.vtail.C2(1,end-1))/(geo.vtail.C2(3,end)-geo.vtail.C2(3,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at inboard section
    geo.vtail.SELL_inboard = 0.5 *(geo.vtail.CR + geo.vtail.CR_kink) *geo.vtail.span_inboard;
    % Thickness ratio at kink
%    geo.vtail.Rt_kink = aircraft.Vertical_tail.thickness_kink;
    geo.vtail.Rt_kink = eq_wbox_t(aircraft.Vertical_tail.airfoilKink,[pdcylin.spar.VT.Aft_VT_spar_loc_kink, pdcylin.spar.VT.Fore_VT_spar_loc_kink]);
    % Step in inboard section
    geo.vtail.dy_inboard = geo.vtail.bS_inboard/pdcylin.guess.vert.inboard;
    % Domain in inboard section
    geo.vtail.y_inboard = (0 : geo.vtail.dy_inboard : geo.vtail.bS_inboard)';
    % Total chord // to vehicle long. axis inboard
    geo.vtail.r_inboard = geo.vtail.CR - geo.vtail.y_inboard./geo.vtail.bS_inboard *(geo.vtail.CR - geo.vtail.CR_kink);
    
    geo.vtail.rs_inboard = geo.vtail.CSR - geo.vtail.y_inboard./geo.vtail.bS_inboard *(geo.vtail.CSR - geo.vtail.CSR_kink);
    
    geo.vtail.x_inboard = linspace( geo.vtail.C2(1,end-1),geo.vtail.C2(1,end),pdcylin.guess.vert.inboard+1)';
    geo.vtail.spar_frac_inboard = linspace(alpha(1,1),alpha(2,1),length(geo.vtail.rs_inboard))';
    
    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.vtail.Swet_inboard = 0.5 *(geo.vtail.r_inboard(1:end-1)+geo.vtail.r_inboard(2:end)) .*(geo.vtail.span_inboard./pdcylin.guess.vert.inboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    % Total chord |- to struct. semispan inboard
    geo.vtail.Z_inboard = geo.vtail.r_inboard .*cos(geo.vtail.lambdaC2_inboard);
    
    geo.vtail.Zs_inboard = geo.vtail.rs_inboard .*cos(geo.vtail.lambdaC2_inboard);
    
    % Thickness distribution
    t_kink = geo.vtail.Rt_kink *geo.vtail.CR_kink;
    geo.vtail.tbs_inboard = t_root - geo.vtail.y_inboard./geo.vtail.bS_inboard *(t_root - t_kink);
    
    % CAERO1 quantities
    geo.vtail.CAERO1.span    = [geo.vtail.CAERO1.span;    geo.vtail.span_inboard];
    geo.vtail.CAERO1.bS      = [geo.vtail.CAERO1.bS;      geo.vtail.bS_inboard];
    geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral; 90+aircraft.Vertical_tail.dihedral_inboard];
    geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE; aircraft.Vertical_tail.LE_sweep_inboard];
    geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC; aircraft.Vertical_tail.quarter_chord_sweep_inboard];
    geo.vtail.CAERO1.sweepC2 = [geo.vtail.CAERO1.sweepC2 ; geo.vtail.lambdaC2_inboard *180/pi];
    geo.vtail.CAERO1.dy      = [geo.vtail.CAERO1.dy;      geo.vtail.dy_inboard];
    geo.vtail.CAERO1.n       = [geo.vtail.CAERO1.n;       pdcylin.stick.nvtail_inboard];
    geo.vtail.CAERO1.n_coarse= [geo.vtail.CAERO1.n_coarse;       pdcylin.stick.nvtail_inboard_coarse];
    geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil; {aircraft.Vertical_tail.airfoilRoot};...
                                                          {aircraft.Vertical_tail.airfoilKink}];
    
    % Fraction chord and name of control surfaceaircraft.Vertical_tail.Rudder.chord
    if aircraft.Vertical_tail.Rudder.present == 1
        frc = aircraft.Vertical_tail.Rudder.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; frc];
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; frc];
        if isequal(aircraft.Vertical_tail.Twin_tail, 1)
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rudlab];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; [rudlab(2:end), 'l']];
        else
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rudlab];
        end
        
        % AELINK card: set to 1
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ; 1];
    else
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; 0];
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; 0];
        if isequal(aircraft.Vertical_tail.Twin_tail, 1)
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
            geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
        else
            geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
        end
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ; 0];
    end

else

    geo.vtail.span_inboard     = 0.0;
    geo.vtail.CR_kink          = geo.vtail.CR;
    geo.vtail.CSR_kink = geo.vtail.CR_kink *(pdcylin.spar.VT.Aft_VT_spar_loc_root - pdcylin.spar.VT.Fore_VT_spar_loc_root);
    geo.vtail.lambdaQC_inboard = 0.0;
    geo.vtail.lambdaLE_inboard = 0.0;
    %
    geo.vtail.csi_kink = 0;
    geo.vtail.ni_kink  = geo.vtail.CR_kink;
    %
    geo.vtail.bS_inboard       = 0.0;
    geo.vtail.Rt_kink          = geo.vtail.Rt_root;
    geo.vtail.dy_inboard       = 0.0;
    geo.vtail.y_inboard        = [];
    geo.vtail.x_inboard        = [];
    geo.vtail.r_inboard        = geo.vtail.CR;
    geo.vtail.Z_inboard        = geo.vtail.r_inboard;
    geo.vtail.SELL_inboard = 0.0;
    t_kink = t_root;

end
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% outboard
%---------------------------------------------------------------------------------------------------------------------------

if (geo.vtail.span_outboard > MIN) %0.01*geo.vtail.b)
    
    % Update index for node at outer position-inboard sector
    geo.vtail.index = [geo.vtail.index; geo.vtail.index(end) + pdcylin.guess.vert.outboard];
    % Angles
%     geo.vtail.lambdaQC_outboard = aircraft.Vertical_tail.quarter_chord_sweep_outboard *(pi/180);  
    geo.vtail.lambdaQC_outboard = atan2(-geo.vtail.CR_kink*0.25+tan(aircraft.Vertical_tail.LE_sweep_outboard*pi/180)*geo.vtail.span_outboard+geo.vtail.CT*0.25,geo.vtail.span_outboard);
    aircraft.Vertical_tail.quarter_chord_sweep_outboard = geo.vtail.lambdaQC_outboard*180/pi;
    geo.vtail.lambdaLE_outboard = (90+aircraft.Vertical_tail.LE_sweep_outboard) *(pi/180);    
    
    geo.vtail.csi_kink = geo.vtail.CR_kink *pdcylin.spar.VT.Fore_VT_spar_loc_kink;
    geo.vtail.ni_kink = geo.vtail.CR_kink *pdcylin.spar.VT.Aft_VT_spar_loc_kink;
    geo.vtail.dihedral_outboard = (90+aircraft.Vertical_tail.dihedral_outboard)*(pi/180);
    %****************************************************************************************************
    if (geo.vtail.span_inboard <= MIN)
        ox = geo.vtail.xLE;
        oy = 0;
        oz = geo.vtail.zLE;
        alpha(1,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
        alpha(2,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;
        REFx = ox + alpha(1,1) *geo.vtail.CR;
        REFy = oy;
        REFz = oz;
    else
        ox = geo.vtail.QC( 1,end )-geo.vtail.CR_kink*0.25;
        oy = geo.vtail.QC( 2,end );
        oz = geo.vtail.QC( 3,end );
        alpha(1,1) = (geo.vtail.csi_kink + (geo.vtail.ni_kink - geo.vtail.csi_kink)/2)/geo.vtail.CR_kink;
        alpha(2,1) = (geo.vtail.csi_tip + (geo.vtail.ni_tip - geo.vtail.csi_tip)/2)/geo.vtail.CT;
        REFx = geo.vtail.C2( 1,end );
        REFy = geo.vtail.C2( 2,end );
        REFz = geo.vtail.C2( 3,end );
    end
    %****************************************************************************************************
    
    %**********************************************************************************************
    c = geo.vtail.CR_kink;
    b = geo.vtail.span_outboard;
    T = geo.vtail.CT/c;
    TW(1,1) = 0 *pi/180;
    TW(2,1) = 0 *pi/180;
    SW = geo.vtail.lambdaQC_outboard;
    dihed = geo.vtail.dihedral_outboard;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.vtail.bS_outboard = norm(qc(:,2) - qc(:,1));
    geo.vtail.WING = [geo.vtail.WING wing];
    geo.vtail.QC   = [geo.vtail.QC qc];
    geo.vtail.C2   = [geo.vtail.C2 c2];
    geo.vtail.PANE = [geo.vtail.PANE pane];
    %**********************************************************************************************

    %**********************************************************************
    % Extract sweep angle - structural line
    geo.vtail.lambdaC2_outboard = atan( (geo.vtail.C2(1,end)-geo.vtail.C2(1,end-1))/(geo.vtail.C2(3,end)-geo.vtail.C2(3,end-1)) );
    %**********************************************************************
    
    geo.vtail.SELL_outboard = 0.5 *(geo.vtail.CR_kink + geo.vtail.CT) *geo.vtail.span_outboard;
%    geo.vtail.Rt_tip        = aircraft.Vertical_tail.thickness_tip; 
    geo.vtail.Rt_tip = eq_wbox_t(aircraft.Vertical_tail.airfoilTip,[pdcylin.spar.VT.Aft_VT_spar_loc_tip, pdcylin.spar.VT.Fore_VT_spar_loc_tip]);
    geo.vtail.dy_outboard   = geo.vtail.bS_outboard/pdcylin.guess.vert.outboard;    
    geo.vtail.y_outboard    = (0 : geo.vtail.dy_outboard : geo.vtail.bS_outboard)';    
    geo.vtail.r_outboard    = geo.vtail.CR_kink - geo.vtail.y_outboard./geo.vtail.bS_outboard *(geo.vtail.CR_kink - geo.vtail.CT);    
    
    geo.vtail.rs_outboard = geo.vtail.CSR_kink - geo.vtail.y_outboard./geo.vtail.bS_outboard *(geo.vtail.CSR_kink - geo.vtail.CST);    
    
    geo.vtail.x_outboard = linspace( geo.vtail.C2(1,end-1),geo.vtail.C2(1,end),pdcylin.guess.vert.outboard+1)';
    geo.vtail.spar_frac_outboard = linspace(alpha(1,1),alpha(2,1),length(geo.vtail.rs_outboard))';

    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.vtail.Swet_outboard = 0.5 *(geo.vtail.r_outboard(1:end-1)+geo.vtail.r_outboard(2:end)) .*(geo.vtail.span_outboard./pdcylin.guess.vert.outboard);
    %$$$$$$$$$$$$$$$$$$$$$$$  
    geo.vtail.Z_outboard    = geo.vtail.r_outboard .*cos(geo.vtail.lambdaC2_outboard);    
    
    geo.vtail.Zs_outboard    = geo.vtail.rs_outboard .*cos(geo.vtail.lambdaC2_outboard);    
    
    % Thickness distribution
    t_tip = geo.vtail.Rt_tip *geo.vtail.CT;
    geo.vtail.tbs_outboard = t_kink - geo.vtail.y_outboard./geo.vtail.bS_outboard *(t_kink - t_tip);
    
    % CAERO1 quantities    
    geo.vtail.CAERO1.span    = [geo.vtail.CAERO1.span;    geo.vtail.span_outboard];    
    geo.vtail.CAERO1.dihedral = [geo.vtail.CAERO1.dihedral; 90+aircraft.Vertical_tail.dihedral_outboard];
    geo.vtail.CAERO1.bS      = [geo.vtail.CAERO1.bS;      geo.vtail.bS_outboard];    
    geo.vtail.CAERO1.sweepLE = [geo.vtail.CAERO1.sweepLE; aircraft.Vertical_tail.LE_sweep_outboard];  
    geo.vtail.CAERO1.sweepQC = [geo.vtail.CAERO1.sweepQC; aircraft.Vertical_tail.quarter_chord_sweep_outboard];
    geo.vtail.CAERO1.sweepC2 = [geo.vtail.CAERO1.sweepC2 ; geo.vtail.lambdaC2_outboard *180/pi];
    geo.vtail.CAERO1.dy      = [geo.vtail.CAERO1.dy;      geo.vtail.dy_outboard];    
    geo.vtail.CAERO1.n       = [geo.vtail.CAERO1.n;       pdcylin.stick.nvtail_outboard];
    geo.vtail.CAERO1.n_coarse= [geo.vtail.CAERO1.n_coarse;       pdcylin.stick.nvtail_outboard_coarse];   
    geo.vtail.CAERO1.airfoil = [geo.vtail.CAERO1.airfoil; {aircraft.Vertical_tail.airfoilKink};...
                                                          {aircraft.Vertical_tail.airfoilTip}];
    
    % Fraction chord and name of control surface
    if aircraft.Vertical_tail.Rudder.present == 1
        rud2lab = rudlab;
        rud2lab(rud2lab=='1') = '2';
        frc = aircraft.Vertical_tail.Rudder.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; frc];
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; frc];
        if (geo.vtail.span_inboard <= MIN)
            if isequal(aircraft.Vertical_tail.Twin_tail, 1)
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rudlab];
                geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; [rudlab(2:end), 'l']];
            else
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rudlab];
            end
        else
            if isequal(aircraft.Vertical_tail.Twin_tail, 1)
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rud2lab];
                geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; [rudlab(2:end), 'l'];[rud2lab(2:end), 'l']];
            else
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; rud2lab];
            end
        end
        % AELINK card: set to 1
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ; 1];
        
    else
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; 0];
        geo.vtail.CAERO1.sup_control.frc = [geo.vtail.CAERO1.sup_control.frc; 0];
        if (geo.vtail.span_inboard <= MIN)
            if isequal(aircraft.Vertical_tail.Twin_tail, 1)
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
                geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
            else
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
            end
        else
            if isequal(aircraft.Vertical_tail.Twin_tail, 1)
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
                geo.vtail2.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'; '    none'];
            else
                geo.vtail.CAERO1.sup_control.nme = [geo.vtail.CAERO1.sup_control.nme; '    none'];
            end
        end
        geo.vtail.CAERO1.sup_control.typ = [geo.vtail.CAERO1.sup_control.typ; 0];
    end
    
    

else

    geo.vtail.span_outboard     = 0.0;
    geo.vtail.CT                = geo.vtail.CR_kink;
    geo.vtail.lambdaQC_outboard = 0.0;
    geo.vtail.lambdaLE_outboard = 0.0;
    geo.vtail.bS_outboard       = 0.0;
    geo.vtail.Rt_tip            = geo.vtail.Rt_kink;
    geo.vtail.dy_outboard       = 0.0;
    geo.vtail.y_outboard        = [];
    geo.vtail.x_outboard        = [];
    geo.vtail.r_outboard        = geo.vtail.CR_kink;
    geo.vtail.Z_outboard        = geo.vtail.r_outboard;
    geo.vtail.tbs_outboard      = geo.vtail.r_outboard;   
    geo.vtail.SELL_outboard     = 0.0;

end




%---------------------------------------------------------------------------------------------------------------------------

%---------------------------------------------------------------------------------------------------------------------------
% Save inboard and outboard parameters refering to correspondent parameter
%---------------------------------------------------------------------------------------------------------------------------

% From inboard
if (geo.vtail.span_inboard > MIN)
    
    geo.vtail.y   = geo.vtail.y_inboard;
    geo.vtail.r   = geo.vtail.r_inboard;
    geo.vtail.rs   = geo.vtail.rs_inboard;
    geo.vtail.Z   = geo.vtail.Z_inboard;
    geo.vtail.Zs   = geo.vtail.Zs_inboard;
    geo.vtail.tbs = geo.vtail.tbs_inboard;    
    geo.vtail.dy  = geo.vtail.dy_inboard .*ones( pdcylin.guess.vert.inboard+1, 1);
    geo.vtail.Swet = geo.vtail.Swet_inboard;
    geo.vtail.x   = geo.vtail.x_inboard;
    geo.vtail.spar_frac = geo.vtail.spar_frac_inboard;

end

% From outboard
if (geo.vtail.span_outboard > MIN)
    
    if (geo.vtail.span_inboard > MIN)
        
        geo.vtail.y   = [geo.vtail.y;   geo.vtail.y(end) + geo.vtail.y_outboard(2:end)];
        geo.vtail.r   = [geo.vtail.r;   geo.vtail.r_outboard(2:end)];
        geo.vtail.rs   = [geo.vtail.rs;   geo.vtail.rs_outboard(2:end)];
        geo.vtail.Z   = [geo.vtail.Z;   geo.vtail.Z_outboard(2:end)];
        geo.vtail.Zs   = [geo.vtail.Zs;   geo.vtail.Zs_outboard(2:end)];
        geo.vtail.tbs = [geo.vtail.tbs; geo.vtail.tbs_outboard(2:end)];        
        geo.vtail.dy  = [geo.vtail.dy;  geo.vtail.dy_inboard .*ones( pdcylin.guess.vert.outboard, 1);];
        geo.vtail.Swet = [geo.vtail.Swet; geo.vtail.Swet_outboard];
        geo.vtail.x   = [geo.vtail.x;    geo.vtail.x_outboard(2:end)];
        geo.vtail.spar_frac   = [geo.vtail.spar_frac; geo.vtail.spar_frac_outboard(2:end)];

    else
        
        geo.vtail.y   = geo.vtail.y_outboard;
        geo.vtail.r   = geo.vtail.r_outboard;
        geo.vtail.rs   = geo.vtail.rs_outboard;
        geo.vtail.Z   = geo.vtail.Z_outboard;
        geo.vtail.Zs   = geo.vtail.Zs_outboard;
        geo.vtail.tbs = geo.vtail.tbs_outboard;        
        geo.vtail.dy  = geo.vtail.dy_outboard .*ones( pdcylin.guess.vert.outboard+1, 1);
        geo.vtail.Swet = geo.vtail.Swet_outboard;
        geo.vtail.x   = geo.vtail.x_outboard;
        geo.vtail.spar_frac = geo.vtail.spar_frac_outboard;
    end 
    
end

% Wetted surface vector [n_bar x 1]: account for upper and lower surface
geo.vtail.Swet = 2*geo.vtail.Swet;

%
% Exposed area for total semi-wing
geo.vtail.SELL = geo.vtail.SELL_inboard + geo.vtail.SELL_outboard;        
%
geo.vtail.leny = length(geo.vtail.y);
geo.vtail.tcs  = geo.vtail.tbs(1);
geo.vtail.bS   = geo.vtail.bS_inboard + geo.vtail.bS_outboard;
%
%---------------------------------------------------------------------------------------------------------------------------

%---------------------------------------------------------------------------------------------------------------------------
% CAERO1 quantities: chord, taper, incidence
%
geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord; geo.vtail.CR];
%
if (geo.vtail.span_outboard > MIN) && (geo.vtail.span_inboard > MIN)
    geo.vtail.CAERO1.chord = [geo.vtail.CAERO1.chord; geo.vtail.CR_kink];
end
%
geo.vtail.CAERO1.chord     = [geo.vtail.CAERO1.chord; geo.vtail.CT];
geo.vtail.CAERO1.taper     = (geo.vtail.CAERO1.chord(2:end)./geo.vtail.CAERO1.chord(1:end-1));
% geo.vtail.CAERO1.dihedral  = zeros(length(geo.vtail.CAERO1.chord)-1, 1);
geo.vtail.CAERO1.incidence = zeros(length(geo.vtail.CAERO1.chord), 1);
% geo.vtail.CAERO1.dihedral(:,1)  = 90;
geo.vtail.CAERO1.incidence(:,1) = 0;
%
%---------------------------------------------------------------------------------------------------------------------------

%---------------------------------------------------------------------------------------------------------------------------
% Vertical tail structural concept
%---------------------------------------------------------------------------------------------------------------------------

% Load txt-input file
wcoef = load('strwingcoef.txt');

if pdcylin.vtail.kcon <=6 

    geo.vtail.ep  = wcoef( pdcylin.vtail.kcon, 1 );
    geo.vtail.e   = wcoef( pdcylin.vtail.kcon, 2 );
    geo.vtail.epc = wcoef( pdcylin.vtail.kcon, 3 );
    geo.vtail.ec  = wcoef( pdcylin.vtail.kcon, 4 );
    geo.vtail.epw = wcoef( pdcylin.vtail.kcon, 5 );
    geo.vtail.Kgc = wcoef( pdcylin.vtail.kcon, 6 );
    geo.vtail.Kgw = wcoef( pdcylin.vtail.kcon, 7 );

else
    geo.vtail.ep  = 0;
    geo.vtail.e   = 0;
    geo.vtail.epc = 0;
    geo.vtail.ec  = 0;
    geo.vtail.epw = 0;
    geo.vtail.Kgc = 0;
    geo.vtail.Kgw = 0;
    
end
%
%---------------------------------------------------------------------------------------------------------------------------
