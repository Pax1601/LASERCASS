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
function [geo, pdcylin] = Geo_Htail(pdcylin, aircraft, geo)
%---------------------------------------------------------------------------------------------------------------------------
% Initialize structure
%---------------------------------------------------------------------------------------------------------------------------
geo.htail.index = [];               % index recognising end of sectors, coincident with kinks
%
geo.htail.zLE = [];                 % LE location along z-coordinate [m], scalar
geo.htail.twc = [];                 % fuselage diam or vertical tail thick depending on stabilizer location [m]  , scalar
geo.htail.SP = [];                  % planform area of vertical tail                                        [m2] , scalar
geo.htail.b = [];                   % span of vertical tail                                                 [m]  , scalar
geo.htail.span_inboard = [];        % span of inboard section, measured parallel to Y-global coordinate     [m]  , scalar
geo.htail.span_outboard = [];       % span of outboard section, measured parallel to Y-global coordinate    [m]  , scalar
geo.htail.CRp = [];                 % hor. tail chord at symmetry plane                                     [m]  , scalar
geo.htail.xLE = [];                 % LE location along x-coordinate                                        [m], scalar
geo.htail.CT = [];                  % tip chord                                                             [m]  , scalar
geo.htail.Rt_root = [];             % thickness ratio at wing root                                          [-]  , scalar
geo.htail.Rt_kink = [];             % thickness ratio at kink                                               [-]  , scalar
geo.htail.Rt_tip = [];              % thickness ratio at wing tip                                           [-]  , scalar
geo.htail.xQC = [];                 % QC location along x-coordinate                                        [m], scalar
geo.htail.xTE = [];                 % TE location along x-coordinate                                        [m], scalar
geo.htail.CR = [];                  % root vert. tail chord                                                 [m]  , scalar
geo.htail.CR_kink = [];             % vert. tail chord at kink                                              [m]  , scalar
%
geo.htail.lambdaQC_inboard = [];    % QC sweep angle at inboard section                                     [rad], scalar
geo.htail.lambdaQC_outboard = [];   % QC sweep angle at outboard section                                    [rad], scalar
%
geo.htail.lambdaC2_inboard = [];    % C2 sweep angle at inboard section                                     [rad], scalar
geo.htail.lambdaC2_outboard = [];   % C2 sweep angle at outboard section                                    [rad], scalar
%
geo.htail.lambdaLE_inboard = [];    % LE sweep angle at inboard section                                     [rad], scalar
geo.htail.lambdaLE_outboard = [];   % LE sweep angle at outboard section                                    [rad], scalar
%
geo.htail.dihedral_inboard  = [];   % dihedral angle at inboard section                                     [rad], scalar
geo.htail.dihedral_outboard = [];   % dihedral angle at inboard section                                     [rad], scalar
%
geo.htail.dinc_inboard = [];
geo.htail.dinc_outboard = [];
geo.htail.incidence_inboard = [];
geo.htail.incidence_outboard = [];
geo.htail.incidence = [];
%
geo.htail.bS_inboard = [];          % sempispan on quarter-chord line at inboard section                    [m]  , scalar
geo.htail.bS_outboard = [];         % sempispan on quarter-chord line at outboard section                   [m]  , scalar
geo.htail.bS = [];                  % sempispan on quarter-chord line                                       [m]  , scalar
%
geo.htail.spar_frac_inboard = [];
geo.htail.spar_frac_outboard = [];
geo.htail.spar_frac = [];
%
geo.htail.SELL_inboard = [];        % exposed area at inboard section                                       [m2] , scalar
geo.htail.SELL_outboard = [];       % exposed area at outboard section                                      [m2] , scalar
geo.htail.SELL = [];                % exposed area for semiwing                                             [m2] , scalar
geo.htail.Swet_inboard = [];        % exposed area at inboard section                                       [m2] , vector
geo.htail.Swet_outboard = [];       % exposed area at outboard section                                      [m2] , vector
geo.htail.Swet = [];                % exposed area for semiwing                                             [m2] , vector
%
geo.htail.dy_inboard = [];          % discretization in inboard section                                     [m]  , scalar
geo.htail.dy_outboard = [];         % discretization in outboard section                                    [m]  , scalar
geo.htail.dy = [];
geo.htail.y_inboard = [];           % discretization along inboard section                                  [m]  , vector
geo.htail.y_outboard = [];          % discretization along outboard section                                 [m]  , vector
geo.htail.x_inboard = [];           % discretization along inboard section                                  [m]  , vector
geo.htail.x_outboard = [];          % discretization along outboard section                                 [m]  , vector

geo.htail.y = [];                   % discretization along structural vert. tail semispan                   [m]  , vector
geo.htail.leny = [];                % nodes along structural wing semispan                                  [-]  , scalar
%
geo.htail.r_inboard = [];           % total chord // to vehicle long. axis inboard                          [m]  , vector
geo.htail.r_outboard = [];          % total chord // to vehicle long. axis outboard                         [m]  , vector
geo.htail.r = [];                   % total chord // to vehicle long. axis                                  [m]  , vector
geo.htail.rs_inboard = [];
geo.htail.rs_outboard = [];
geo.htail.rs = [];

geo.htail.Z_inboard = [];           % total chord |- to struct. semispan inboard                            [m]  , vector
geo.htail.Z_outboard = [];          % total chord |- to struct. semispan outboard                           [m]  , vector
geo.htail.Z = [];                   % total chord |- to struct. semispan                                    [m]  , vector
geo.htail.Zs_inboard = [];           % total chord |- to struct. semispan inboard                            [m]  , vector
geo.htail.Zs_outboard = [];          % total chord |- to struct. semispan outboard                           [m]  , vector
geo.htail.Zs = [];                   % total chord |- to struct. semispan                                    [m]  , vector
%
geo.htail.Rt_inboard = [];          % thickness ratio inboard                                               [-]  , vector
geo.htail.Rt_outboard = [];         % thickness ratio outboard                                              [-]  , vector
geo.htail.Rt = [];                  % thickness ratio                                                       [-]  , vector
geo.htail.tbs_inboard = [];         % thickness vert. tail box inboard                                      [-]  , vector
geo.htail.tbs_outboard = [];        % thickness vert. tail box outboard                                     [-]  , vector
geo.htail.tbs = [];                 % thickness vert. tail box                                              [-]  , vector
geo.htail.tcs = [];                 % thickness in the carrythrough structure                               [m]  , scalar
%
geo.htail.epc = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.e = [];                   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.ep = [];                  % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.ec = [];                  % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.epw = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.Kgc = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.htail.Kgw = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
%
geo.htail.n = [];                   % nr of elements for sectors used in the geo                            [-]  , vector
%
% For the defined sectors, quantities are stored in appropriate vectors
geo.htail.CAERO1.chord      = [];    % chord for each defined sector                                          [m]  , vector
geo.htail.CAERO1.span       = [];    % span for each defined sector                                           [m]  , vector
geo.htail.CAERO1.taper      = [];    % taper for each defined sector                                          [-]  , vector
geo.htail.CAERO1.bS         = [];    % contains bS for each defined sectors                                   [m]  , vector
geo.htail.CAERO1.SELL       = [];    % exposed aerodyn. area for each defined sector                          [m2] , vector
geo.htail.CAERO1.dy         = [];    % step for each defined sector                                           [m]  , scalar
geo.htail.CAERO1.dihedral   = [];    % dihedral for each defined sector                                       [deg], vector
geo.htail.CAERO1.sweepLE    = [];    % LE sweep angle for each defined sector                                 [deg], vector
geo.htail.CAERO1.sweepQC    = [];    % QC sweep angle for each defined sector                                 [deg], vector
geo.htail.CAERO1.sweepC2    = [];    % C2 sweep angle for each defined sector                                 [deg], vector
geo.htail.CAERO1.incidence  = [];    % incidence angle for each defined sector                                [deg], vector
geo.htail.CAERO1.n          = [];    % number of elements for each defined sector                             [-]  , vector
geo.htail.CAERO1.n_coarse   = [];    % number of elements for each defined sector                             [-]  , vector
geo.htail.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                [-]  , vector
geo.htail.CAERO1.sup_control.nme = [];% Name of control surface                                               [-]  , vector
geo.htail.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.htail.CAERO1.airfoil    = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
%
geo.htail.csi_root          = [];
geo.htail.ni_root           = [];
geo.htail.csi_kink          = [];
geo.htail.ni_kink           = [];
geo.htail.csi_tip           = [];
geo.htail.ni_tip            = [];
%
%**************************************************************************
geo.htail.WING              = [];   % points defining each panel (3x(4*nr_sector))
geo.htail.QC                = [];   % points defining quarter chord line (3x(2*nr_sector))
geo.htail.C2                = [];   % points defining elastic line (3x(2*nr_sector))
geo.htail.PANE              = [];   % points to write CAERO1 card
%**************************************************************************
geo.htail.x_nodes     = [];
geo.htail.x_nodes_1_2 = [];
%
geo.htail.V                 = [];
geo.htail.cg                = [];
%
% Modified by S.R. 17/02/10
%
lab = TRIMlabels;
labels = lab(:, 1);
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% Basic calculation
%---------------------------------------------------------------------------------------------------------------------------

% % T-tail configuration
% if (aircraft.Horizontal_tail.vertical_location > (aircraft.Vertical_tail.vertical_location + aircraft.Vertical_tail.span))
%     aircraft.Horizontal_tail.vertical_location = (aircraft.Vertical_tail.vertical_location + aircraft.Vertical_tail.span);
% end
geo.htail.index = 1;
% LE location along z-coordinate
geo.htail.zLE = aircraft.Horizontal_tail.vertical_location;
% LE location along x-coordinate
geo.htail.xLE = aircraft.Horizontal_tail.longitudinal_location;
% planform area of horizontal tail
geo.htail.SP = aircraft.Horizontal_tail.area;
% span of vertical tail (2 different way for the same number)
geo.htail.b = aircraft.Horizontal_tail.span;
% Root chord
geo.htail.CRp = 2*geo.htail.SP/( geo.htail.b*((1 + aircraft.Horizontal_tail.taper_kink)*aircraft.Horizontal_tail.spanwise_kink +...
    (aircraft.Horizontal_tail.taper_kink + aircraft.Horizontal_tail.taper_tip)*(1 - aircraft.Horizontal_tail.spanwise_kink)) );
% QC location along x-coordinate in symmetry plane
geo.htail.xQC = geo.htail.xLE + 1/4*geo.htail.CRp;
% CT thickness
geo.htail.twc = 0;
%****************************************************************************************
FUSE = 1; % ht fuselage intersection is the default case
geo.htail.intersect = 'none';
% check if vtail intersection
if isequal(pdcylin.stick.model.vert, 1)
  INTX = 0;
  INTY = 0;
  INTZ = 0;
  TWIN = 0;
  if isfield(aircraft.Vertical_tail,'Twin_tail') && aircraft.Vertical_tail.Twin_tail
    TWIN = geo.wing.b*aircraft.Vertical_tail.Twin_tail_span;
  end
  %1 check spanwise
  if (geo.htail.b>TWIN)
    INTY = 1;
  %2 check longitudinal
    MIDP = geo.htail.xLE + geo.htail.CRp/2;
    for k=1:size(geo.vtail.WING,2)/4
      midp1 = (geo.vtail.WING(1,(k-1)*4+1)+geo.vtail.WING(1,(k-1)*4+2))/2;
      midp2 = (geo.vtail.WING(1,(k-1)*4+3)+geo.vtail.WING(1,(k-1)*4+4))/2;
      if MIDP>midp1 && MIDP<midp2
        INTX = 1;
        break;
      end
    end
    %3 check vertical
    if(geo.htail.zLE>aircraft.Vertical_tail.vertical_location && geo.htail.zLE<=aircraft.Vertical_tail.vertical_location+geo.vtail.b)
      INTZ = 1;
    end
  end              
  %
  SUMINT = INTX+INTY+INTZ;
  %  vt and ht intersection  
  if (SUMINT==3)
    FUSE = 0;
    geo.htail.intersect = 'vert';
    geo.vtail.intersect = 'hori';
  end
  if (TWIN==0) % INTY and INTX not considered
    geo.htail.intersect = 'vert';
    geo.vtail.intersect = 'hori';
    if(INTZ)
      % define step along global Z-axis in the vertical tail
      if geo.vtail.span_inboard==0
        dz = geo.vtail.b/(pdcylin.guess.vert.outboard);
        z = (0 : dz : geo.vtail.b)';
      elseif geo.vtail.span_outboard==0
        dz = geo.vtail.b/(pdcylin.guess.vert.inboard);
        z = (0 : dz : geo.vtail.b)';
      else
        dz1 = geo.vtail.span_inboard/(pdcylin.guess.vert.inboard);
        dz2 = geo.vtail.span_outboard/(pdcylin.guess.vert.outboard);
        z =  [(0 : dz1 : geo.vtail.span_inboard)' ; (geo.vtail.span_inboard+dz2:dz2:geo.vtail.b)'];
      end
      % interpolate thickness distribution in the vertical tail
      geo.htail.twc = interp1(z, geo.vtail.tbs, (geo.htail.zLE-geo.vtail.zLE), 'linear', 'extrap');
      FUSE = 0;
    elseif(geo.htail.zLE>=aircraft.Vertical_tail.vertical_location+geo.vtail.b)
      geo.htail.twc = geo.vtail.tbs(end);
      FUSE = 0;
    end
  end
end
%****************************************************************************************
if (FUSE)
  if isequal(pdcylin.stick.model.fuse, 1)
    Zfus = interp1(geo.fus.xx,geo.fus.zz,geo.htail.xLE+geo.htail.CRp*0.5, 'linear', 'extrap');
    Rfus = spline(geo.fus.x,geo.fus.r,geo.htail.xLE+geo.htail.CRp*0.5);
    if (geo.htail.zLE <= Zfus+Rfus) || (geo.htail.zLE >= Zfus-Rfus)
      geo.htail.intersect = 'fuse';
      DZ = abs(geo.htail.zLE-Zfus);
      if abs(DZ)<=1e-6
        geo.htail.twc = Rfus*2;
      else
        theta = acos(DZ/Rfus);
        geo.htail.twc = Rfus*sin(theta)*2;
      end
    end
  end
end
%****************************************************************************************
%
% Span of inboard sector, measured parallel to global Y coordinate
geo.htail.span_inboard = aircraft.Horizontal_tail.spanwise_kink *(geo.htail.b/2) - (geo.htail.twc/2);
% Span of outboard sector, measured parallel to global Y coordinate
geo.htail.span_outboard = (1 - aircraft.Horizontal_tail.spanwise_kink) *(geo.htail.b/2);

MIN = 0.01*aircraft.Horizontal_tail.span*0.5;

% Check on different span, want to obtain a uniform distribution of node
% both on guess model and smart model.
if ~isfield(pdcylin.guess,'check')
    Ltot = (aircraft.Horizontal_tail.span-geo.htail.twc)*0.5;
    Ls = [geo.htail.span_inboard , geo.htail.span_outboard];
    index = find(Ls<MIN);
    n = ceil(Ls * (pdcylin.guess.hori.inboard + pdcylin.guess.hori.outboard )/Ltot );
    n(index) = 0;
%    if n(1)<0
%        n(2) = n(2)+n(1);
%        n(1) = 0;
%    end
    m = ceil(Ls * (pdcylin.stick.nhtail_inboard  + pdcylin.stick.nhtail_outboard )/Ltot );
    m(index) = 0;
%    if m(1)<0
%        m(2) = m(2)+m(1);
%        m(1) = 0;
%    end
    m2 = ceil(Ls * (pdcylin.stick.nhtail_inboard_coarse +  pdcylin.stick.nhtail_outboard_coarse )/Ltot );
    m2(index) = 0;
%    if m2(1)<0
%        m2(2) = m2(2)+m2(1);
%        m2(1) = 0;
%    end
    ny = ceil(Ls * (pdcylin.stick.ny.hori_inboard  + pdcylin.stick.ny.hori_outboard )/Ltot );
    ny(index) = 0;
%    if ny(1)<0
%        ny(2) = ny(2)+ny(1);
%        ny(1) = 0;
%    end
%
    if(geo.htail.twc ==0)
        pdcylin.stick.nhtail_carryth_coarse = 0;
        pdcylin.stick.nhtail_carryth = 0;
    end
    fprintf('\n\t\t- Htail computational mesh: %d, %d elements.', n(1), n(2));
    fprintf('\n\t\t- Htail stick mesh: %d, %d elements.', m2(1), m2(2));
    fprintf('\n\t\t- Htail spanwise aerodynamic mesh: %d, %d elements.', ny(1), ny(2));
%
    pdcylin.guess.hori.inboard = n(1);
    pdcylin.guess.hori.outboard = n(2);
    
    pdcylin.stick.nhtail_inboard = m(1);
    pdcylin.stick.nhtail_outboard = m(2);
    
    pdcylin.stick.nhtail_inboard_coarse = m2(1);
    pdcylin.stick.nhtail_outboard_coarse = m2(2);
    
    pdcylin.stick.ny.hori_inboard = ny(1);
    pdcylin.stick.ny.hori_outboard = ny(2);
end

% Tip chord
geo.htail.CT = geo.htail.CRp *aircraft.Horizontal_tail.taper_tip;
% Struct. chord at tip
geo.htail.CST = (pdcylin.spar.HT.Aft_HT_spar_loc_tip - pdcylin.spar.HT.Fore_HT_spar_loc_tip) *geo.htail.CT;
%
geo.htail.csi_tip = pdcylin.spar.HT.Fore_HT_spar_loc_tip *geo.htail.CT;
geo.htail.ni_tip = pdcylin.spar.HT.Aft_HT_spar_loc_tip *geo.htail.CT;
%
% thickness ratio at root
%geo.htail.Rt_root = aircraft.Horizontal_tail.thickness_root;
geo.htail.Rt_root = eq_wbox_t(aircraft.Horizontal_tail.airfoilRoot,[pdcylin.spar.HT.Aft_HT_spar_loc_root, pdcylin.spar.HT.Fore_HT_spar_loc_root]);
% Thickness ratio at kink1
%geo.htail.Rt_kink = aircraft.Horizontal_tail.thickness_kink;
% thickness ratio at tip
% TE location along x-coordinate in symmetry plane
geo.htail.xTE = geo.htail.xLE + geo.htail.CRp;
%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% inboard
%---------------------------------------------------------------------------------------------------------------------------
elevlab = str2_8ch_right(labels{20});
if (geo.htail.span_inboard > MIN)
    
    geo.htail.index = [geo.htail.index; geo.htail.index(end) + pdcylin.guess.hori.inboard];
    %
    % Aerodynamic wing chord at kink
    geo.htail.CR_kink = geo.htail.CRp *aircraft.Horizontal_tail.taper_kink;
    geo.htail.CSR_kink = (pdcylin.spar.HT.Aft_HT_spar_loc_kink - pdcylin.spar.HT.Fore_HT_spar_loc_kink) *geo.htail.CR_kink;
    % Angles
    geo.htail.lambdaQC_inboard = aircraft.Horizontal_tail.quarter_chord_sweep_inboard*(pi/180);
    geo.htail.lambdaLE_inboard = aircraft.Horizontal_tail.LE_sweep_inboard*(pi/180);
    geo.htail.dihedral_inboard = aircraft.Horizontal_tail.dihedral_inboard*(pi/180);
    %
    % Angle of incidence [deg], at the end it's converted
    geo.htail.dinc_inboard = (aircraft.Horizontal_tail.kink_incidence-aircraft.Horizontal_tail.root_incidence)/pdcylin.guess.hori.inboard;
    %
    if (geo.htail.dinc_inboard ~= 0)
        geo.htail.incidence_inboard = (aircraft.Horizontal_tail.root_incidence : geo.htail.dinc_inboard : aircraft.Horizontal_tail.kink_incidence)';
    else
        geo.htail.incidence_inboard = aircraft.Horizontal_tail.root_incidence*ones(pdcylin.guess.hori.inboard+1, 1);
    end
    %
    % Wing chord at wing-fuselage connection
    geo.htail.CR = geo.htail.CRp - (geo.htail.twc/2)/(aircraft.Horizontal_tail.spanwise_kink *(geo.htail.b/2))*(geo.htail.CRp-geo.htail.CR_kink);
    geo.htail.CSR = (pdcylin.spar.HT.Aft_HT_spar_loc_root - pdcylin.spar.HT.Fore_HT_spar_loc_root) *geo.htail.CR;
    %
    geo.htail.csi_root = geo.htail.CR *pdcylin.spar.HT.Fore_HT_spar_loc_root;
    geo.htail.ni_root  = geo.htail.CR *pdcylin.spar.HT.Aft_HT_spar_loc_root;
    geo.htail.csi_kink = geo.htail.CR_kink *pdcylin.spar.HT.Fore_HT_spar_loc_kink;
    geo.htail.ni_kink  = geo.htail.CR_kink *pdcylin.spar.HT.Aft_HT_spar_loc_kink;
    
    %**********************************************************************************************
    c = geo.htail.CR;
    b = geo.htail.span_inboard;
    T = geo.htail.CR_kink/c;
    TW(1,1) = aircraft.Horizontal_tail.root_incidence*pi/180;
    TW(2,1) = aircraft.Horizontal_tail.kink_incidence*pi/180;
    SW = geo.htail.lambdaQC_inboard;
    dihed = geo.htail.dihedral_inboard;
    ox = geo.htail.xQC + geo.htail.twc/2*tan(geo.htail.lambdaQC_inboard) - geo.htail.CR/4;
    oy = 0;
    oz = geo.htail.zLE;
    alpha(1,1) = (geo.htail.csi_root+(geo.htail.ni_root-geo.htail.csi_root)/2)/geo.htail.CR;
    alpha(2,1) = (geo.htail.csi_kink+(geo.htail.ni_kink-geo.htail.csi_kink)/2)/geo.htail.CR_kink;
    REFx = ox + alpha(1,1)*geo.htail.CR;
    REFy = oy;
    REFz = oz;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.htail.bS_inboard = norm(qc(:,2) - qc(:,1));
    geo.htail.WING = [geo.htail.WING wing];
    geo.htail.QC   = [geo.htail.QC qc];
    geo.htail.C2   = [geo.htail.C2 c2];
    geo.htail.PANE = [geo.htail.PANE pane];
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.htail.lambdaC2_inboard = atan( (geo.htail.C2(1,end)-geo.htail.C2(1,end-1))/(geo.htail.C2(2,end)-geo.htail.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at inboard section
    geo.htail.SELL_inboard = 0.5*(geo.htail.CR+geo.htail.CR_kink)*geo.htail.span_inboard;
    % Step in inboard
    geo.htail.dy_inboard = geo.htail.bS_inboard/pdcylin.guess.hori.inboard;
    % Node discretization in inboard
    geo.htail.y_inboard = (0:geo.htail.dy_inboard:geo.htail.bS_inboard)';
    % Aerodynamic wing chord in inboard measured along X axis
    geo.htail.r_inboard = geo.htail.CR-geo.htail.y_inboard./geo.htail.bS_inboard*(geo.htail.CR-geo.htail.CR_kink);
    % Structural wing chord in inboard measured along X axis
    geo.htail.rs_inboard = geo.htail.CSR-geo.htail.y_inboard./geo.htail.bS_inboard*(geo.htail.CSR-geo.htail.CSR_kink);
    %
    geo.htail.Swet_inboard = 0.5 *(geo.htail.r_inboard(1:end-1)+geo.htail.r_inboard(2:end)) .*(geo.htail.span_inboard./pdcylin.guess.hori.inboard);
    %
    geo.htail.x_inboard = linspace( geo.htail.C2(1,end-1),geo.htail.C2(1,end),pdcylin.guess.hori.inboard+1)';
    geo.htail.spar_frac_inboard = linspace(alpha(1,1),alpha(2,1),length(geo.htail.rs_inboard))';
    % Aerodynamic wing chord in inboard measured perpendicular to structural chord
    %     geo.htail.Z_inboard = geo.htail.r_inboard.*cos(geo.htail.lambdaQC_inboard);
    geo.htail.Z_inboard = geo.htail.r_inboard.*cos(geo.htail.lambdaC2_inboard);
    
    % Structural wing chord in inboard measured perpendicular to structural chord
    %     geo.htail.Zs_inboard = geo.htail.rs_inboard.*cos(geo.htail.lambdaQC_inboard);
    geo.htail.Zs_inboard = geo.htail.rs_inboard.*cos(geo.htail.lambdaC2_inboard);
    
    
    % Thickness
    t_root = geo.htail.Rt_root *geo.htail.CR;
    geo.htail.Rt_kink = eq_wbox_t(aircraft.Horizontal_tail.airfoilKink,[pdcylin.spar.HT.Aft_HT_spar_loc_kink, pdcylin.spar.HT.Fore_HT_spar_loc_kink]);
    t_kink = geo.htail.Rt_kink *geo.htail.CR_kink;
    geo.htail.tbs_inboard = t_root - geo.htail.y_inboard./geo.htail.bS_inboard*(t_root - t_kink);
    
    % CAERO1 parameters
    geo.htail.CAERO1.span     = [geo.htail.CAERO1.span    ; geo.htail.span_inboard];
    geo.htail.CAERO1.bS       = [geo.htail.CAERO1.bS      ; geo.htail.bS_inboard];
    geo.htail.CAERO1.SELL     = [geo.htail.CAERO1.SELL    ; geo.htail.SELL_inboard];
    geo.htail.CAERO1.dy       = [geo.htail.CAERO1.dy      ; geo.htail.dy_inboard];
    geo.htail.CAERO1.dihedral = [geo.htail.CAERO1.dihedral; aircraft.Horizontal_tail.dihedral_inboard];
    geo.htail.CAERO1.sweepLE  = [geo.htail.CAERO1.sweepLE ; aircraft.Horizontal_tail.LE_sweep_inboard];
    geo.htail.CAERO1.sweepQC  = [geo.htail.CAERO1.sweepQC ; aircraft.Horizontal_tail.quarter_chord_sweep_inboard];
    geo.htail.CAERO1.sweepC2  = [geo.htail.CAERO1.sweepC2 ; geo.htail.lambdaC2_inboard *180/pi];
    geo.htail.CAERO1.n        = [geo.htail.CAERO1.n       ; pdcylin.stick.nhtail_inboard];
    geo.htail.CAERO1.n_coarse = [geo.htail.CAERO1.n_coarse       ; pdcylin.stick.nhtail_inboard_coarse];
    geo.htail.CAERO1.airfoil  = [geo.htail.CAERO1.airfoil; {aircraft.Horizontal_tail.airfoilRoot};...
        {aircraft.Horizontal_tail.airfoilKink}];
    
    if aircraft.Horizontal_tail.Elevator.present == 1
        % Fraction chord and name of control surface
        frc = aircraft.Horizontal_tail.Elevator.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; frc];
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; frc];
        geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme; elevlab];
        
        % AELINK card: set to -1 (flap)
        geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ; -1];
        
    else
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; 0];
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; 0];
        geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme;'    none'];
        
        % AELINK card: set to -1 (flap)
        geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ; 0];
    end
    
else
    
    geo.htail.span_inboard = 0.0;
    geo.htail.CR_kink = geo.htail.CRp *1; % updated in the next section
    geo.htail.CR      = geo.htail.CRp;    % updated in the next section
    geo.htail.lambdaQC_inboard = 0.0; %aircraft.Horizontal_tail.quarter_chord_sweep_inboard*(pi/180);
    geo.htail.lambdaLE_inboard = 0.0; %aircraft.Horizontal_tail.LE_sweep_inboard*(pi/180);
    geo.htail.dihedral_inboard = 0.0; %aircraft.Horizontal_tail.dihedral_inboard*(pi/180);
    geo.htail.dihedral_inboard  = 0.0;
    geo.htail.incidence_inboard = 0.0;
    geo.htail.bS_inboard = 0.0;
    geo.htail.SELL_inboard = 0.0;
    
end
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% outboard
%---------------------------------------------------------------------------------------------------------------------------

if (geo.htail.span_outboard > MIN)
    
    if (geo.htail.span_inboard <= MIN)
        
        % Update outboard span and move inboard sector to the intersection
        geo.htail.span_outboard = (geo.htail.b - geo.htail.twc)/2;
        geo.htail.CR = geo.htail.CRp - (geo.htail.twc/2)/geo.htail.span_outboard*(geo.htail.CR_kink-geo.htail.CT);
        geo.htail.CSR = geo.htail.CR *(pdcylin.spar.HT.Aft_HT_spar_loc_root - pdcylin.spar.HT.Fore_HT_spar_loc_root);
        geo.htail.CR_kink = geo.htail.CR;
        geo.htail.CSR_kink = geo.htail.CSR;
        geo.htail.Rt_kink = geo.htail.Rt_root;
        t_kink = geo.htail.Rt_kink *geo.htail.CR_kink;
        %
        geo.htail.csi_kink = geo.htail.CR_kink *pdcylin.spar.HT.Fore_HT_spar_loc_root;
        geo.htail.ni_kink  = geo.htail.CR_kink *pdcylin.spar.HT.Aft_HT_spar_loc_root;
        geo.htail.csi_root = geo.htail.csi_kink;
        geo.htail.ni_root = geo.htail.ni_kink;
        %
        
    end
    
    geo.htail.index = [geo.htail.index; geo.htail.index(end) + pdcylin.guess.hori.outboard];
    % Angles
    geo.htail.lambdaQC_outboard = aircraft.Horizontal_tail.quarter_chord_sweep_outboard*(pi/180);
    geo.htail.lambdaLE_outboard = aircraft.Horizontal_tail.LE_sweep_outboard*(pi/180);
    geo.htail.dihedral_outboard = aircraft.Horizontal_tail.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.htail.dinc_outboard = (aircraft.Horizontal_tail.tip_incidence-aircraft.Horizontal_tail.kink_incidence)/pdcylin.guess.hori.outboard;
    %
    if (geo.htail.dinc_outboard ~= 0)
        geo.htail.incidence_outboard = (aircraft.Horizontal_tail.kink_incidence:geo.htail.dinc_outboard:aircraft.Horizontal_tail.tip_incidence)';
    else
        geo.htail.incidence_outboard = aircraft.Horizontal_tail.kink_incidence*ones(pdcylin.guess.hori.outboard+1, 1);
    end
    
    %**********************************************************************************************
    c = geo.htail.CR_kink;
    b = geo.htail.span_outboard;
    T = geo.htail.CT/c;
    TW(1,1) = aircraft.Horizontal_tail.kink_incidence *(pi/180);
    TW(2,1) = aircraft.Horizontal_tail.tip_incidence *(pi/180);
    SW = geo.htail.lambdaQC_outboard;
    dihed = geo.htail.dihedral_outboard;
    alpha(1,1) = (geo.htail.csi_kink + (geo.htail.ni_kink - geo.htail.csi_kink)/2)/geo.htail.CR_kink;
    alpha(2,1) = (geo.htail.csi_tip + (geo.htail.ni_tip - geo.htail.csi_tip)/2)/geo.htail.CT;
    %
    if (geo.htail.span_inboard == 0.0)
        ox = geo.htail.xQC + geo.htail.twc/2*tan(geo.htail.lambdaQC_outboard) - geo.htail.CR/4;
        oy = 0;
        oz = geo.htail.zLE;
        REFx = ox + alpha(1,1)*geo.htail.CR_kink;
        REFy = oy;
        REFz = oz;
    else
        ox   = geo.htail.QC( 1,end );
        oy   = geo.htail.QC( 2,end );
        oz   = geo.htail.QC( 3,end );
        REFx = geo.htail.C2( 1,end );
        REFy = geo.htail.C2( 2,end );
        REFz = geo.htail.C2( 3,end );
    end
    %
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.htail.bS_outboard = norm(qc(:,2) - qc(:,1));
    geo.htail.WING = [geo.htail.WING wing];
    geo.htail.QC   = [geo.htail.QC qc];
    geo.htail.C2   = [geo.htail.C2 c2];
    geo.htail.PANE = [geo.htail.PANE pane];
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.htail.lambdaC2_outboard = atan( (geo.htail.C2(1,end)-geo.htail.C2(1,end-1))/(geo.htail.C2(2,end)-geo.htail.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at outboard section
    geo.htail.SELL_outboard = 0.5*(geo.htail.CR_kink+geo.htail.CT)*geo.htail.span_outboard;
    % Step in outboard
    geo.htail.dy_outboard = geo.htail.bS_outboard/pdcylin.guess.hori.outboard;
    % Node discretization in outboard
    geo.htail.y_outboard = (0:geo.htail.dy_outboard:geo.htail.bS_outboard)';
    % Aerodynamic wing chord in outboard measured along X axis
    geo.htail.r_outboard = geo.htail.CR_kink-geo.htail.y_outboard./geo.htail.bS_outboard*(geo.htail.CR_kink-geo.htail.CT);
    % Structural wing chord in outboard measured along X axis
    geo.htail.rs_outboard = geo.htail.CSR_kink-geo.htail.y_outboard./geo.htail.bS_outboard*(geo.htail.CSR_kink-geo.htail.CST);
    geo.htail.x_outboard= linspace( geo.htail.C2(1,end-1),geo.htail.C2(1,end),pdcylin.guess.hori.outboard+1)';
    geo.htail.spar_frac_outboard = linspace(alpha(1,1),alpha(2,1),length(geo.htail.rs_outboard))';
    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.htail.Swet_outboard = 0.5 *(geo.htail.r_outboard(1:end-1)+geo.htail.r_outboard(2:end)) .*(geo.htail.span_outboard./pdcylin.guess.hori.outboard);
    %$$$$$$$$$$$$$$$$$$$$$$$
    
    % Aerodynamic wing chord in outboard  measured perpendicular to structural chord
    %     geo.htail.Z_outboard = geo.htail.r_outboard.*cos(geo.htail.lambdaQC_outboard);
    geo.htail.Z_outboard = geo.htail.r_outboard.*cos(geo.htail.lambdaC2_outboard);
    
    % Structural wing chord in outboard  measured perpendicular to structural chord
    %     geo.htail.Zs_outboard = geo.htail.rs_outboard.*cos(geo.htail.lambdaQC_outboard);
    geo.htail.Zs_outboard = geo.htail.rs_outboard.*cos(geo.htail.lambdaC2_outboard);
    geo.htail.Rt_tip = eq_wbox_t(aircraft.Horizontal_tail.airfoilTip,[pdcylin.spar.HT.Aft_HT_spar_loc_tip, pdcylin.spar.HT.Fore_HT_spar_loc_tip]);
    t_tip = geo.htail.CT *geo.htail.Rt_tip;
    geo.htail.tbs_outboard = t_kink - geo.htail.y_outboard./geo.htail.bS_outboard*(t_kink - t_tip);
    %
    % CAERO1 parameters
    geo.htail.CAERO1.span     = [geo.htail.CAERO1.span    ; geo.htail.span_outboard];
    geo.htail.CAERO1.bS       = [geo.htail.CAERO1.bS      ; geo.htail.bS_outboard];
    geo.htail.CAERO1.SELL     = [geo.htail.CAERO1.SELL    ; geo.htail.SELL_outboard];
    geo.htail.CAERO1.dy       = [geo.htail.CAERO1.dy      ; geo.htail.dy_outboard];
    geo.htail.CAERO1.dihedral = [geo.htail.CAERO1.dihedral; aircraft.Horizontal_tail.dihedral_outboard];
    geo.htail.CAERO1.sweepLE  = [geo.htail.CAERO1.sweepLE ; aircraft.Horizontal_tail.LE_sweep_outboard];
    geo.htail.CAERO1.sweepQC  = [geo.htail.CAERO1.sweepQC ; aircraft.Horizontal_tail.quarter_chord_sweep_outboard];
    geo.htail.CAERO1.sweepC2  = [geo.htail.CAERO1.sweepC2 ; geo.htail.lambdaC2_outboard *180/pi];
    geo.htail.CAERO1.n        = [geo.htail.CAERO1.n       ; pdcylin.stick.nhtail_outboard];
    geo.htail.CAERO1.n_coarse = [geo.htail.CAERO1.n_coarse       ; pdcylin.stick.nhtail_outboard_coarse];
    geo.htail.CAERO1.airfoil  = [geo.htail.CAERO1.airfoil; {aircraft.Horizontal_tail.airfoilKink};...
        {aircraft.Horizontal_tail.airfoilTip}];
    
    % Fraction chord and name of control surface
    if aircraft.Horizontal_tail.Elevator.present == 1
        elev2lab = elevlab;
        elev2lab(elev2lab=='1') = '2';
        frc = aircraft.Horizontal_tail.Elevator.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; frc];
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; frc];
        if (geo.htail.span_inboard <= MIN)
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme; elevlab];
        else
            geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme; elev2lab];
        end
        
        % AELINK card: set to -1 (flap)
        geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ; -1];
    else
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; 0];
        geo.htail.CAERO1.sup_control.frc = [geo.htail.CAERO1.sup_control.frc; 0];
        geo.htail.CAERO1.sup_control.nme = [geo.htail.CAERO1.sup_control.nme; '    none'];
        
        % AELINK card: set to -1 (flap)
        geo.htail.CAERO1.sup_control.typ = [geo.htail.CAERO1.sup_control.typ; 0];
    end
        
        
    %
else
    
    geo.htail.span_outboard = 0.0;
    % Angles
    geo.htail.lambdaQC_outboard = 0.0; %aircraft.Horizontal_tail.quarter_chord_sweep_outboard*(pi/180);
    geo.htail.lambdaLE_outboard = 0.0; %aircraft.Horizontal_tail.LE_sweep_outboard*(pi/180);
    geo.htail.dihedral_outboard = 0.0; %aircraft.Horizontal_tail.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.htail.dihedral_outboard  = 0;
    geo.htail.incidence_outboard = [];
    geo.htail.bS_outboard = 0.0;
    % Aerodynamic exposed area at outboard section
    geo.htail.SELL_outboard = 0.0;
    
end
%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% Save inboard and outboard parameters refering to correspondent parameter
%---------------------------------------------------------------------------------------------------------------------------

% From inboard
if (geo.htail.span_inboard > MIN)
    
    geo.htail.y         = geo.htail.y_inboard;
    geo.htail.r         = geo.htail.r_inboard;
    geo.htail.rs         = geo.htail.rs_inboard;
    geo.htail.Z         = geo.htail.Z_inboard;
    geo.htail.Zs         = geo.htail.Zs_inboard;
    geo.htail.tbs       = geo.htail.tbs_inboard;
    geo.htail.incidence = geo.htail.incidence_inboard;
    geo.htail.dy        = geo.htail.dy_inboard .*ones( pdcylin.guess.hori.inboard+1, 1);
    geo.htail.Swet = geo.htail.Swet_inboard;
    geo.htail.x         = geo.htail.x_inboard;
    geo.htail.spar_frac = geo.htail.spar_frac_inboard;

end

% From outboard
if (geo.htail.span_outboard > MIN)
    
    if (geo.htail.span_inboard > MIN)
        
        geo.htail.y         = [geo.htail.y        ; geo.htail.y(end)+geo.htail.y_outboard( 2:end )];
        geo.htail.r         = [geo.htail.r        ; geo.htail.r_outboard( 2:end )];
        geo.htail.rs         = [geo.htail.rs        ; geo.htail.rs_outboard( 2:end )];
        geo.htail.Z         = [geo.htail.Z        ; geo.htail.Z_outboard( 2:end )];
        geo.htail.Zs         = [geo.htail.Zs        ; geo.htail.Zs_outboard( 2:end )];
        geo.htail.tbs       = [geo.htail.tbs      ; geo.htail.tbs_outboard( 2:end )];
        geo.htail.incidence = [geo.htail.incidence; geo.htail.incidence_outboard( 2:end )];
        geo.htail.dy        = [geo.htail.dy       ; geo.htail.dy_outboard .*ones( pdcylin.guess.hori.outboard, 1)];
        geo.htail.Swet      = [geo.htail.Swet; geo.htail.Swet_outboard];
        geo.htail.x         = [geo.htail.x        ; geo.htail.x_outboard( 2:end )];
        geo.htail.spar_frac = [geo.htail.spar_frac; geo.htail.spar_frac_outboard( 2:end )];

    else
        
        geo.htail.y         = geo.htail.y_outboard;
        geo.htail.r         = geo.htail.r_outboard;
        geo.htail.rs         = geo.htail.rs_outboard;
        geo.htail.Z         = geo.htail.Z_outboard;
        geo.htail.Zs         = geo.htail.Zs_outboard;
        geo.htail.tbs       = geo.htail.tbs_outboard;
        geo.htail.incidence = geo.htail.incidence_outboard;
        geo.htail.dy        = geo.htail.dy_outboard .*ones( pdcylin.guess.hori.outboard+1, 1);
        geo.htail.Swet      = geo.htail.Swet_outboard;
        geo.htail.x         = geo.htail.x_outboard;
        geo.htail.spar_frac = geo.htail.spar_frac_outboard;
    end
    
end

% Wetted surface vector [n_bar x 1]: account for upper and lower surface
geo.htail.Swet = 2*geo.htail.Swet;

% Convert incidence angle
geo.htail.incidence = geo.htail.incidence .*(pi/180);
% Exposed area for total semi-wing
geo.htail.SELL = geo.htail.SELL_inboard + geo.htail.SELL_outboard;
% Number of nodes used to discretize structural semi-wing
geo.htail.leny = length(geo.htail.y);
% Carrythrough thickness
geo.htail.tcs = max(geo.htail.tbs);
% Length of structural line as summation of each single sector
geo.htail.bS = geo.htail.bS_inboard + geo.htail.bS_outboard;
%
%---------------------------------------------------------------------------------------------------------------------------

%---------------------------------------------------------------------------------------------------------------------------
% CAERO1 quantities: chord, taper, incidence
%
geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord; geo.htail.CR];
geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence; aircraft.Horizontal_tail.root_incidence];
%
if (geo.htail.span_outboard > MIN) && (geo.htail.span_inboard > MIN)
    geo.htail.CAERO1.chord = [geo.htail.CAERO1.chord; geo.htail.CR_kink];
    geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence; aircraft.Horizontal_tail.kink_incidence];
end
%
geo.htail.CAERO1.chord     = [geo.htail.CAERO1.chord; geo.htail.CT];
geo.htail.CAERO1.incidence = [geo.htail.CAERO1.incidence; aircraft.Horizontal_tail.tip_incidence];
geo.htail.CAERO1.taper     = (geo.htail.CAERO1.chord(2:end)./geo.htail.CAERO1.chord(1:end-1));

%*****************************************
if isequal(pdcylin.stick.model.symmXZ, 1)
    nrsp = length(geo.htail.CAERO1.taper);
    for i = 1:nrsp
        % copy the name
        geo.htail.CAERO1.sup_control.nme(i+nrsp,:) = geo.htail.CAERO1.sup_control.nme(i,:);
        % change the last letter
        geo.htail.CAERO1.sup_control.nme(i+nrsp,end) = 'l';
    end
end
%*****************************************

%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% Horizontal tail structural concept
%---------------------------------------------------------------------------------------------------------------------------

% Load txt-input file
wcoef = load('strwingcoef.txt');

if pdcylin.htail.kcon <=6 

    geo.htail.ep  = wcoef( pdcylin.htail.kcon, 1 );
    geo.htail.e   = wcoef( pdcylin.htail.kcon, 2 );
    geo.htail.epc = wcoef( pdcylin.htail.kcon, 3 );
    geo.htail.ec  = wcoef( pdcylin.htail.kcon, 4 );
    geo.htail.epw = wcoef( pdcylin.htail.kcon, 5 );
    geo.htail.Kgc = wcoef( pdcylin.htail.kcon, 6 );
    geo.htail.Kgw = wcoef( pdcylin.htail.kcon, 7 );

else
    geo.htail.ep  = 0;
    geo.htail.e   = 0;
    geo.htail.epc = 0;
    geo.htail.ec  = 0;
    geo.htail.epw = 0;
    geo.htail.Kgc = 0;
    geo.htail.Kgw = 0;
    
end
%
%---------------------------------------------------------------------------------------------------------------------------
