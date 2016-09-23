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
%                 1.4     Travaglini       Adding controls on distribution
%                                          of points and on span geometry
%     100216      1.4     A. De Gaspari    Unified labels
%
%*******************************************************************************
function [geo,pdcylin] = Geo_Canard(pdcylin, aircraft, geo)
%---------------------------------------------------------------------------------------------------------------------------
% Initialize structure
%---------------------------------------------------------------------------------------------------------------------------
geo.canard.index = [];               % index recognising end of sectors, coincident with kinks
%
geo.canard.zLE = [];                 % LE location along z-coordinate [m], scalar
geo.canard.twc = [];                 % fuselage diam or vertical tail thick depending on stabilizer location [m]  , scalar
geo.canard.SP = [];                  % planform area of vertical tail                                        [m2] , scalar
geo.canard.b = [];                   % span of vertical tail                                                 [m]  , scalar
geo.canard.span_inboard = [];        % span of inboard section, measured parallel to Y-global coordinate     [m]  , scalar
geo.canard.span_outboard = [];       % span of outboard section, measured parallel to Y-global coordinate    [m]  , scalar
geo.canard.CRp = [];                 % hor. tail chord at symmetry plane                                     [m]  , scalar
geo.canard.xLE = [];                 % LE location along x-coordinate                                        [m], scalar
geo.canard.CT = [];                  % tip chord                                                             [m]  , scalar
geo.canard.Rt_root = [];             % thickness ratio at wing root                                          [-]  , scalar
geo.canard.Rt_kink = [];             % thickness ratio at kink                                               [-]  , scalar
geo.canard.Rt_tip = [];              % thickness ratio at wing tip                                           [-]  , scalar
geo.canard.xQC = [];                 % QC location along x-coordinate                                        [m], scalar
geo.canard.xTE = [];                 % TE location along x-coordinate                                        [m], scalar
geo.canard.CR = [];                  % root vert. tail chord                                                 [m]  , scalar
geo.canard.CR_kink = [];             % vert. tail chord at kink                                              [m]  , scalar
%
geo.canard.lambdaQC_inboard = [];    % QC sweep angle at inboard section                                     [rad], scalar
geo.canard.lambdaQC_outboard = [];   % QC sweep angle at outboard section                                    [rad], scalar
%
geo.canard.lambdaC2_inboard = [];    % C2 sweep angle at inboard section                                     [rad], scalar
geo.canard.lambdaC2_outboard = [];   % C2 sweep angle at outboard section                                    [rad], scalar
% 
geo.canard.lambdaLE_inboard = [];    % LE sweep angle at inboard section                                     [rad], scalar
geo.canard.lambdaLE_outboard = [];   % LE sweep angle at outboard section                                    [rad], scalar
% 
geo.canard.dihedral_inboard  = [];   % dihedral angle at inboard section                                     [rad], scalar
geo.canard.dihedral_outboard = [];   % dihedral angle at inboard section                                     [rad], scalar
%
geo.canard.dinc_inboard = [];
geo.canard.dinc_outboard = [];
geo.canard.incidence_inboard = [];
geo.canard.incidence_outboard = [];
geo.canard.incidence = [];
%
geo.canard.bS_inboard = [];          % sempispan on quarter-chord line at inboard section                    [m]  , scalar
geo.canard.bS_outboard = [];         % sempispan on quarter-chord line at outboard section                   [m]  , scalar
geo.canard.bS = [];                  % sempispan on quarter-chord line                                       [m]  , scalar
%
geo.canard.SELL_inboard = [];        % exposed area at inboard section                                       [m2] , scalar
geo.canard.SELL_outboard = [];       % exposed area at outboard section                                      [m2] , scalar
geo.canard.SELL = [];                % exposed area for semiwing                                             [m2] , scalar
geo.canard.Swet_inboard = [];        % exposed area at inboard section                                       [m2] , vector
geo.canard.Swet_outboard = [];       % exposed area at outboard section                                      [m2] , vector
geo.canard.Swet = [];                % exposed area for semiwing                                             [m2] , vector
%
geo.canard.dy_inboard = [];          % discretization in inboard section                                     [m]  , scalar
geo.canard.dy_outboard = [];         % discretization in outboard section                                    [m]  , scalar
geo.canard.dy = [];
geo.canard.y_inboard = [];           % discretization along inboard section                                  [m]  , vector
geo.canard.x_inboard = [];
geo.canard.y_outboard = [];          % discretization along outboard section                                 [m]  , vector
geo.canard.x_outboard = []; 
geo.canard.y = [];                   % discretization along structural vert. tail semispan                   [m]  , vector
geo.canard.leny = [];                % nodes along structural wing semispan                                  [-]  , scalar
%
geo.canard.r_inboard = [];           % total chord // to vehicle long. axis inboard                          [m]  , vector
geo.canard.r_outboard = [];          % total chord // to vehicle long. axis outboard                         [m]  , vector
geo.canard.r = [];                   % total chord // to vehicle long. axis                                  [m]  , vector
geo.canard.rs_inboard = [];
geo.canard.rs_outboard = [];
geo.canard.rs = [];         

geo.canard.Z_inboard = [];           % total chord |- to struct. semispan inboard                            [m]  , vector
geo.canard.Z_outboard = [];          % total chord |- to struct. semispan outboard                           [m]  , vector
geo.canard.Z = [];                   % total chord |- to struct. semispan                                    [m]  , vector
geo.canard.Zs_inboard = [];           % total chord |- to struct. semispan inboard                            [m]  , vector
geo.canard.Zs_outboard = [];          % total chord |- to struct. semispan outboard                           [m]  , vector
geo.canard.Zs = [];                   % total chord |- to struct. semispan                                    [m]  , vector
%
geo.canard.spar_frac_inboard = [];
geo.canard.spar_frac_outboard = [];
geo.canard.spar_frac = [];
%
geo.canard.Rt_inboard = [];          % thickness ratio inboard                                               [-]  , vector
geo.canard.Rt_outboard = [];         % thickness ratio outboard                                              [-]  , vector
geo.canard.Rt = [];                  % thickness ratio                                                       [-]  , vector
geo.canard.tbs_inboard = [];         % thickness vert. tail box inboard                                      [-]  , vector
geo.canard.tbs_outboard = [];        % thickness vert. tail box outboard                                     [-]  , vector
geo.canard.tbs = [];                 % thickness vert. tail box                                              [-]  , vector
geo.canard.tcs = [];                 % thickness in the carrythrough structure                               [m]  , scalar
%
geo.canard.epc = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.e = [];                   % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.ep = [];                  % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.ec = [];                  % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.epw = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.Kgc = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
geo.canard.Kgw = [];                 % vert. tail structural coefficients & exponents                        [-]  , scalar
%
geo.canard.n = [];                   % nr of elements for sectors used in the geo                            [-]  , vector
%
% For the defined sectors, quantities are stored in appropriate vectors
geo.canard.CAERO1.chord      = [];    % chord for each defined sector                                          [m]  , vector
geo.canard.CAERO1.span       = [];    % span for each defined sector                                           [m]  , vector
geo.canard.CAERO1.taper      = [];    % taper for each defined sector                                          [-]  , vector
geo.canard.CAERO1.bS         = [];    % contains bS for each defined sectors                                   [m]  , vector
geo.canard.CAERO1.SELL       = [];    % exposed aerodyn. area for each defined sector                          [m2] , vector
geo.canard.CAERO1.dy         = [];    % step for each defined sector                                           [m]  , scalar
geo.canard.CAERO1.dihedral   = [];    % dihedral for each defined sector                                       [deg], vector
geo.canard.CAERO1.sweepLE    = [];    % LE sweep angle for each defined sector                                 [deg], vector
geo.canard.CAERO1.sweepQC    = [];    % QC sweep angle for each defined sector                                 [deg], vector
geo.canard.CAERO1.sweepC2    = [];    % C2 sweep angle for each defined sector                                 [deg], vector
geo.canard.CAERO1.incidence  = [];    % incidence angle for each defined sector                                [deg], vector
geo.canard.CAERO1.n          = [];    % number of elements for each defined sector                             [-]  , vector
geo.canard.CAERO1.n_coarse   = [];    % number of elements for each defined sector                             [-]  , vector
geo.canard.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                [-]  , vector
geo.canard.CAERO1.sup_control.nme = [];% Name of control surface                                               [-]  , vector
geo.canard.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.canard.CAERO1.airfoil    = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
%
geo.canard.csi_root          = [];
geo.canard.ni_root           = [];
geo.canard.csi_kink          = [];
geo.canard.ni_kink           = [];
geo.canard.csi_tip           = [];
geo.canard.ni_tip            = [];
%
%**************************************************************************
geo.canard.WING              = [];   % points defining each panel (3x(4*nr_sector))
geo.canard.QC                = [];   % points defining quarter chord line (3x(2*nr_sector))
geo.canard.C2                = [];   % points defining elastic line (3x(2*nr_sector))
geo.canard.PANE              = [];   % points to write CAERO1 card
%**************************************************************************
geo.canard.x_nodes     = [];
geo.canard.x_nodes_1_2 = [];
%
geo.canard.V                 = []; 
geo.canard.cg                = [];
%
lab = TRIMlabels;
labels = lab(:, 1);
%---------------------------------------------------------------------------------------------------------------------------

geo.canard.intersect = 'none';
%---------------------------------------------------------------------------------------------------------------------------
% Basic calculation
%---------------------------------------------------------------------------------------------------------------------------

% T-tail configuration 
% if (aircraft.Canard.vertical_location > (aircraft.Vertical_tail.vertical_location + aircraft.Vertical_tail.span))
%     aircraft.Canard.vertical_location = (aircraft.Vertical_tail.vertical_location + aircraft.Vertical_tail.span);
% end

geo.canard.index = 1;
% LE location along z-coordinate
geo.canard.zLE = aircraft.Canard.vertical_location;
% LE location along x-coordinate
geo.canard.xLE = aircraft.Canard.longitudinal_location;
% planform area of horizontal tail
geo.canard.SP = aircraft.Canard.area;
% span of vertical tail (2 different way for the same number)
geo.canard.b = aircraft.Canard.span;
% Root chord
geo.canard.CRp = 2*geo.canard.SP/( geo.canard.b*((1 + aircraft.Canard.taper_kink)*aircraft.Canard.spanwise_kink +...
                (aircraft.Canard.taper_kink + aircraft.Canard.taper_tip)*(1 - aircraft.Canard.spanwise_kink)) );
% QC location along x-coordinate in symmetry plane
geo.canard.xQC = geo.canard.xLE + 1/4*geo.canard.CRp;

%****************************************************************************************
Zfus = interp1(geo.fus.xx,geo.fus.zz,geo.canard.xLE+geo.canard.CRp*0.5, 'linear', 'extrap');
Rfus = spline(geo.fus.x,geo.fus.r,geo.canard.xLE+geo.canard.CRp*0.5);
geo.canard.Zfus = Zfus;
geo.canard.Rfus = Rfus;
%
if (geo.canard.zLE <= Zfus+Rfus) || (geo.canard.zLE >= Zfus-Rfus)
    geo.canard.intersect = 'fuse';
    DZ = abs(geo.canard.zLE-Zfus);
    if abs(DZ)<=1e-6
        geo.canard.twc = Rfus*2;
    else
        theta = acos(DZ/Rfus);
        geo.canard.twc = Rfus*sin(theta)*2;
    end
else
    geo.canard.twc = 0;
end
%****************************************************************************************

%
% Span of inboard sector, measured parallel to global Y coordinate
geo.canard.span_inboard = aircraft.Canard.spanwise_kink *(geo.canard.b/2) - (geo.canard.twc/2);
% Span of outboard sector, measured parallel to global Y coordinate
geo.canard.span_outboard = (1 - aircraft.Canard.spanwise_kink) *(geo.canard.b/2);
%
MIN = 0.01*aircraft.Canard.span*0.5;
% Check on different span, want to obtain a uniform distribution of node
% both on guess model and smart model.

if ~isfield(pdcylin.guess,'check')
%
    Ltot = (aircraft.Canard.span- geo.canard.twc)*0.5;
    Ls = [geo.canard.span_inboard , geo.canard.span_outboard];
    index = find(Ls<MIN);
%
    n = ceil( Ls * (pdcylin.guess.canard.inboard + pdcylin.guess.canard.outboard )/Ltot );
    n(index)=0;
%    if n(1) == 0
%        n(n==0) = 1;
%        n(2) = n(2)-1;
%    end
%    if n(1)<0
%        n(2) = n(2)+n(1);
%        n(1) = 0;
%    end
    m = ceil( Ls * (pdcylin.stick.ncanard_inboard  + pdcylin.stick.ncanard_outboard )/Ltot );
    m(index)=0;
%    if m(1) == 0
%        m(m==0) = 1;
%        m(2) = m(2)-1;
%    end
%    if m(1)<0
%        m(2) = m(2)+m(1);
%        m(1) = 0;
%    end
    m2 = ceil( Ls * (pdcylin.stick.ncanard_inboard_coarse +  pdcylin.stick.ncanard_outboard_coarse )/Ltot );
    m2(index)=0;
%    if m2(1) == 0
%        m2(m2==0) = 1;
%        m2(2) = m2(2)-1;
%    end
%    if m2(1)<0
%        m2(2) = m2(2)+m2(1);
%        m2(1) = 0;
%    end
    ny = ceil( Ls * (pdcylin.stick.ny.canard_inboard +  pdcylin.stick.ny.canard_outboard )/Ltot );
    ny(index)=0;
%    if ny(1) == 0
%        ny(ny==0) = 1;
%        ny(2) = ny(2)-1;
%    end
%    if ny(1)<0
%        ny(2) = ny(2)+ny(1);
%        ny(1) = 0;
%    end
%
    fprintf('\n\t\t- Canard computational mesh: %d, %d elements.', n(1), n(2));
    fprintf('\n\t\t- Canard stick mesh: %d, %d elements.', m2(1), m2(2));
    fprintf('\n\t\t- Canard spanwise aerodynamic mesh: %d, %d elements.', ny(1), ny(2));
%
    pdcylin.guess.canard.inboard = n(1);
    pdcylin.guess.canard.outboard = n(2);
    
    pdcylin.stick.ncanard_inboard = m(1);
    pdcylin.stick.ncanard_outboard = m(2);
    
    pdcylin.stick.ncanard_inboard_coarse = m2(1);
    pdcylin.stick.ncanard_outboard_coarse = m2(2);
    
    pdcylin.stick.ny.canard_inboard = ny(1);
    pdcylin.stick.ny.canard_outboard = ny(2);
end

% Tip chord
geo.canard.CT = geo.canard.CRp *aircraft.Canard.taper_tip;
% Struct. chord at tip
geo.canard.CST = (pdcylin.spar.CA.Aft_CA_spar_loc_tip - pdcylin.spar.CA.Fore_CA_spar_loc_tip) *geo.canard.CT;
%
geo.canard.csi_tip = pdcylin.spar.CA.Fore_CA_spar_loc_tip *geo.canard.CT;
geo.canard.ni_tip = pdcylin.spar.CA.Aft_CA_spar_loc_tip *geo.canard.CT;
%
% thickness ratio at root
%geo.canard.Rt_root = aircraft.Canard.thickness_root;
geo.canard.Rt_root = eq_wbox_t(aircraft.Canard.airfoilRoot,[pdcylin.spar.CA.Aft_CA_spar_loc_root, pdcylin.spar.CA.Fore_CA_spar_loc_root]);
% Thickness ratio at kink1
%geo.canard.Rt_kink = aircraft.Canard.thickness_kink;
% thickness ratio at tip
%geo.canard.Rt_tip = aircraft.Canard.thickness_tip;
%t_tip = geo.canard.CT *geo.canard.Rt_tip;
% TE location along x-coordinate in symmetry plane
geo.canard.xTE = geo.canard.xLE + geo.canard.CRp;
%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% inboard
%---------------------------------------------------------------------------------------------------------------------------
canlab = str2_8ch_right(labels{22});

if (geo.canard.span_inboard > MIN)

    geo.canard.index = [geo.canard.index; geo.canard.index(end) + pdcylin.guess.canard.inboard];
    %
    % Aerodynamic wing chord at kink 
    geo.canard.CR_kink = geo.canard.CRp *aircraft.Canard.taper_kink;
    geo.canard.CSR_kink = (pdcylin.spar.CA.Aft_CA_spar_loc_kink - pdcylin.spar.CA.Fore_CA_spar_loc_kink) *geo.canard.CR_kink;
    % Angles
    geo.canard.lambdaQC_inboard = aircraft.Canard.quarter_chord_sweep_inboard*(pi/180);
    geo.canard.lambdaLE_inboard = aircraft.Canard.LE_sweep_inboard*(pi/180);
    geo.canard.dihedral_inboard = aircraft.Canard.dihedral_inboard*(pi/180);
    %
    % Angle of incidence [deg], at the end it's converted
    geo.canard.dinc_inboard = (aircraft.Canard.kink_incidence-aircraft.Canard.root_incidence)/pdcylin.guess.canard.inboard;
    %
    if (geo.canard.dinc_inboard ~= 0)
        geo.canard.incidence_inboard = (aircraft.Canard.root_incidence : geo.canard.dinc_inboard : aircraft.Canard.kink_incidence)';
    else
        geo.canard.incidence_inboard = aircraft.Canard.root_incidence*ones(pdcylin.guess.canard.inboard+1, 1);
    end        
    %
    % Wing chord at wing-fuselage connection
	geo.canard.CR = geo.canard.CRp - (geo.canard.twc/2)/(aircraft.Canard.spanwise_kink *(geo.canard.b/2))*(geo.canard.CRp-geo.canard.CR_kink);
    geo.canard.CSR = (pdcylin.spar.CA.Aft_CA_spar_loc_root - pdcylin.spar.CA.Fore_CA_spar_loc_root) *geo.canard.CR;
    %
    geo.canard.csi_root = geo.canard.CR *pdcylin.spar.CA.Fore_CA_spar_loc_root;
    geo.canard.ni_root  = geo.canard.CR *pdcylin.spar.CA.Aft_CA_spar_loc_root;
    geo.canard.csi_kink = geo.canard.CR_kink *pdcylin.spar.CA.Fore_CA_spar_loc_kink;
    geo.canard.ni_kink  = geo.canard.CR_kink *pdcylin.spar.CA.Aft_CA_spar_loc_kink;
    
    %**********************************************************************************************
    c = geo.canard.CR;
    b = geo.canard.span_inboard;
    T = geo.canard.CR_kink/c;
    TW(1,1) = aircraft.Canard.root_incidence*pi/180;
    TW(2,1) = aircraft.Canard.kink_incidence*pi/180;
    SW = geo.canard.lambdaQC_inboard;
    dihed = geo.canard.dihedral_inboard;
    ox = geo.canard.xQC + geo.canard.twc/2*tan(geo.canard.lambdaQC_inboard) - geo.canard.CR/4;
    oy = 0;
    oz = geo.canard.zLE;
    alpha(1,1) = (geo.canard.csi_root+(geo.canard.ni_root-geo.canard.csi_root)/2)/geo.canard.CR;
    alpha(2,1) = (geo.canard.csi_kink+(geo.canard.ni_kink-geo.canard.csi_kink)/2)/geo.canard.CR_kink;
    REFx = ox + alpha(1,1)*geo.canard.CR;
    REFy = oy;
    REFz = oz;
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.canard.bS_inboard = norm(qc(:,2) - qc(:,1));
    geo.canard.WING = [geo.canard.WING wing];
    geo.canard.QC   = [geo.canard.QC qc];
    geo.canard.C2   = [geo.canard.C2 c2];
    geo.canard.PANE = [geo.canard.PANE pane];
    %**********************************************************************************************
    
    %**********************************************************************
    % Extract sweep angle - structural line
    geo.canard.lambdaC2_inboard = atan( (geo.canard.C2(1,end)-geo.canard.C2(1,end-1))/(geo.canard.C2(2,end)-geo.canard.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at inboard section
    geo.canard.SELL_inboard = 0.5*(geo.canard.CR+geo.canard.CR_kink)*geo.canard.span_inboard;
    % Step in inboard
    geo.canard.dy_inboard = geo.canard.bS_inboard/pdcylin.guess.canard.inboard;
    % Node discretization in inboard
    geo.canard.y_inboard = (0:geo.canard.dy_inboard:geo.canard.bS_inboard)';
    % Aerodynamic wing chord in inboard measured along X axis
    geo.canard.r_inboard = geo.canard.CR-geo.canard.y_inboard./geo.canard.bS_inboard*(geo.canard.CR-geo.canard.CR_kink);
    % Structural wing chord in inboard measured along X axis
    geo.canard.rs_inboard = geo.canard.CSR-geo.canard.y_inboard./geo.canard.bS_inboard*(geo.canard.CSR-geo.canard.CSR_kink);
    %
    geo.canard.Swet_inboard = 0.5 *(geo.canard.r_inboard(1:end-1)+geo.canard.r_inboard(2:end)) .*(geo.canard.span_inboard./pdcylin.guess.canard.inboard);
    %
    geo.canard.x_inboard = linspace(geo.canard.C2(1,end-1),geo.canard.C2(1,end),pdcylin.guess.canard.inboard+1)';
    geo.canard.spar_frac_inboard = linspace(alpha(1,1),alpha(2,1),length(geo.canard.rs_inboard))';
    % Aerodynamic wing chord in inboard measured perpendicular to structural chord 
%     geo.canard.Z_inboard = geo.canard.r_inboard.*cos(geo.canard.lambdaQC_inboard);
    geo.canard.Z_inboard = geo.canard.r_inboard.*cos(geo.canard.lambdaC2_inboard);
    
    % Structural wing chord in inboard measured perpendicular to structural chord 
%     geo.canard.Zs_inboard = geo.canard.rs_inboard.*cos(geo.canard.lambdaQC_inboard);
    geo.canard.Zs_inboard = geo.canard.rs_inboard.*cos(geo.canard.lambdaC2_inboard);
    
    
    % Thickness
    t_root = geo.canard.Rt_root *geo.canard.CR;
    geo.canard.Rt_kink = eq_wbox_t(aircraft.Canard.airfoilKink,[pdcylin.spar.CA.Aft_CA_spar_loc_kink, pdcylin.spar.CA.Fore_CA_spar_loc_kink]);
    t_kink = geo.canard.Rt_kink *geo.canard.CR_kink;
    geo.canard.tbs_inboard = t_root - geo.canard.y_inboard./geo.canard.bS_inboard*(t_root - t_kink);

    % CAERO1 parameters        
    geo.canard.CAERO1.span     = [geo.canard.CAERO1.span    ; geo.canard.span_inboard];
    geo.canard.CAERO1.bS       = [geo.canard.CAERO1.bS      ; geo.canard.bS_inboard];
    geo.canard.CAERO1.SELL     = [geo.canard.CAERO1.SELL    ; geo.canard.SELL_inboard];
    geo.canard.CAERO1.dy       = [geo.canard.CAERO1.dy      ; geo.canard.dy_inboard];
    geo.canard.CAERO1.dihedral = [geo.canard.CAERO1.dihedral; aircraft.Canard.dihedral_inboard];
    geo.canard.CAERO1.sweepLE  = [geo.canard.CAERO1.sweepLE ; aircraft.Canard.LE_sweep_inboard];
    geo.canard.CAERO1.sweepQC  = [geo.canard.CAERO1.sweepQC ; aircraft.Canard.quarter_chord_sweep_inboard];
    geo.canard.CAERO1.sweepC2  = [geo.canard.CAERO1.sweepC2 ; geo.canard.lambdaC2_inboard *180/pi];
    geo.canard.CAERO1.n        = [geo.canard.CAERO1.n       ; pdcylin.stick.ncanard_inboard];
    geo.canard.CAERO1.n_coarse = [geo.canard.CAERO1.n_coarse       ; pdcylin.stick.ncanard_inboard_coarse];
    geo.canard.CAERO1.airfoil  = [geo.canard.CAERO1.airfoil; {aircraft.Canard.airfoilRoot};...
                                                           {aircraft.Canard.airfoilKink}];
    
    % Fraction chord and name of control surface
    if aircraft.Canard.Elevator.present ==1
        frc = aircraft.Canard.Elevator.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; frc];
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; frc];
        geo.canard.CAERO1.sup_control.nme = [geo.canard.CAERO1.sup_control.nme; canlab];
        
        % AELINK card: set to -1 (flap)
        geo.canard.CAERO1.sup_control.typ = [geo.canard.CAERO1.sup_control.typ; -1];
    else
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; 0];
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; 0];
        geo.canard.CAERO1.sup_control.nme = [geo.canard.CAERO1.sup_control.nme; '    none'];
        geo.canard.CAERO1.sup_control.typ = [geo.canard.CAERO1.sup_control.typ; 0];
    end

else
    
    geo.canard.span_inboard = 0.0;
    geo.canard.CR_kink = geo.canard.CRp *1; % updated in the next section
    geo.canard.CR      = geo.canard.CRp;    % updated in the next section
    geo.canard.lambdaQC_inboard = 0.0; %aircraft.Canard.quarter_chord_sweep_inboard*(pi/180);
    geo.canard.lambdaLE_inboard = 0.0; %aircraft.Canard.LE_sweep_inboard*(pi/180);
    geo.canard.dihedral_inboard = 0.0; %aircraft.Canard.dihedral_inboard*(pi/180);
    geo.canard.dihedral_inboard  = 0.0;
    geo.canard.incidence_inboard = 0.0;
    geo.canard.bS_inboard = 0.0;
    geo.canard.SELL_inboard = 0.0;

end
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% outboard
%---------------------------------------------------------------------------------------------------------------------------

if (geo.canard.span_outboard > MIN)
    
    if (geo.canard.span_inboard <= MIN)
        
        % Update outboard span and move inboard sector to the intersection
        geo.canard.span_outboard = (geo.canard.b - geo.canard.twc)/2;
        geo.canard.CR = geo.canard.CRp - (geo.canard.twc/2)/geo.canard.span_outboard*(geo.canard.CR_kink-geo.canard.CT);
        geo.canard.CSR = geo.canard.CR *(pdcylin.spar.CA.Aft_CA_spar_loc_kink - pdcylin.spar.CA.Fore_CA_spar_loc_kink);
        geo.canard.CR_kink = geo.canard.CR;
        geo.canard.CSR_kink = geo.canard.CR_kink *(pdcylin.spar.CA.Aft_CA_spar_loc_root - pdcylin.spar.CA.Fore_CA_spar_loc_root);
        t_kink = geo.canard.Rt_kink *geo.canard.CR_kink;
        %
        geo.canard.csi_kink = geo.canard.CR_kink *pdcylin.spar.CA.Fore_CA_spar_loc_kink;
        geo.canard.ni_kink  = geo.canard.CR_kink *pdcylin.spar.CA.Aft_CA_spar_loc_kink;
        %
        
    end

    geo.canard.index = [geo.canard.index; geo.canard.index(end) + pdcylin.guess.canard.outboard];
    % Angles
    geo.canard.lambdaQC_outboard = aircraft.Canard.quarter_chord_sweep_outboard*(pi/180);
    geo.canard.lambdaLE_outboard = aircraft.Canard.LE_sweep_outboard*(pi/180);
    geo.canard.dihedral_outboard = aircraft.Canard.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.canard.dinc_outboard = (aircraft.Canard.tip_incidence-aircraft.Canard.kink_incidence)/pdcylin.guess.canard.outboard;
    %
    if (geo.canard.dinc_outboard ~= 0)
        geo.canard.incidence_outboard = (aircraft.Canard.kink_incidence:geo.canard.dinc_outboard:aircraft.Canard.tip_incidence)';
    else
        geo.canard.incidence_outboard = aircraft.Canard.kink_incidence*ones(pdcylin.guess.canard.outboard+1, 1);
    end    
    
    %**********************************************************************************************
    c = geo.canard.CR_kink;
    b = geo.canard.span_outboard;
    T = geo.canard.CT/c;
    TW(1,1) = aircraft.Canard.kink_incidence *(pi/180);
    TW(2,1) = aircraft.Canard.tip_incidence *(pi/180);
    SW = geo.canard.lambdaQC_outboard;
    dihed = geo.canard.dihedral_outboard;
    alpha(1,1) = (geo.canard.csi_kink + (geo.canard.ni_kink - geo.canard.csi_kink)/2)/geo.canard.CR_kink;
    alpha(2,1) = (geo.canard.csi_tip + (geo.canard.ni_tip - geo.canard.csi_tip)/2)/geo.canard.CT;
    %
    if (geo.canard.span_inboard == 0.0)
        ox = geo.canard.xQC + geo.canard.twc/2*tan(geo.canard.lambdaQC_outboard) - geo.canard.CR/4;
        oy = 0;
        oz = geo.canard.zLE;
        REFx = ox + alpha(1,1)*geo.canard.CR_kink;
        REFy = oy;
        REFz = oz;
    else
        ox   = geo.canard.QC( 1,end );
        oy   = geo.canard.QC( 2,end );
        oz   = geo.canard.QC( 3,end );
        REFx = geo.canard.C2( 1,end );
        REFy = geo.canard.C2( 2,end );
        REFz = geo.canard.C2( 3,end );
    end
    %
    [wing, qc, c2, pane] = aero_geo(c, b, T, TW, SW, dihed, ox, oy, oz, alpha, REFx, REFy, REFz);
    geo.canard.bS_outboard = norm(qc(:,2) - qc(:,1));
    geo.canard.WING = [geo.canard.WING wing];
    geo.canard.QC   = [geo.canard.QC qc];
    geo.canard.C2   = [geo.canard.C2 c2];
    geo.canard.PANE = [geo.canard.PANE pane];
    %**********************************************************************************************

    %**********************************************************************
    % Extract sweep angle - structural line
    geo.canard.lambdaC2_outboard = atan( (geo.canard.C2(1,end)-geo.canard.C2(1,end-1))/(geo.canard.C2(2,end)-geo.canard.C2(2,end-1)) );
    %**********************************************************************
    
    % Aerodynamic exposed area at outboard section
    geo.canard.SELL_outboard = 0.5*(geo.canard.CR_kink+geo.canard.CT)*geo.canard.span_outboard;
    % Step in outboard    
    geo.canard.dy_outboard = geo.canard.bS_outboard/pdcylin.guess.canard.outboard;
    % Node discretization in outboard
    geo.canard.y_outboard = (0:geo.canard.dy_outboard:geo.canard.bS_outboard)';
    % Aerodynamic wing chord in outboard measured along X axis
    geo.canard.r_outboard = geo.canard.CR_kink-geo.canard.y_outboard./geo.canard.bS_outboard*(geo.canard.CR_kink-geo.canard.CT);
    % Structural wing chord in outboard measured along X axis
    geo.canard.rs_outboard = geo.canard.CSR_kink-geo.canard.y_outboard./geo.canard.bS_outboard*(geo.canard.CSR_kink-geo.canard.CST);
    
    %$$$$$$$$$$$$$$$$$$$$$$$
    geo.canard.Swet_outboard = 0.5 *(geo.canard.r_outboard(1:end-1)+geo.canard.r_outboard(2:end)) .*(geo.canard.span_outboard./pdcylin.guess.canard.outboard);
    %$$$$$$$$$$$$$$$$$$$$$$$    
    geo.canard.x_outboard = linspace(geo.canard.C2(1,end-1),geo.canard.C2(1,end),pdcylin.guess.canard.outboard+1)';
    geo.canard.spar_frac_outboard = linspace(alpha(1,1),alpha(2,1),length(geo.canard.rs_outboard))';
    % Aerodynamic wing chord in outboard  measured perpendicular to structural chord
%     geo.canard.Z_outboard = geo.canard.r_outboard.*cos(geo.canard.lambdaQC_outboard);
    geo.canard.Z_outboard = geo.canard.r_outboard.*cos(geo.canard.lambdaC2_outboard);
    
    % Structural wing chord in outboard  measured perpendicular to structural chord
%     geo.canard.Zs_outboard = geo.canard.rs_outboard.*cos(geo.canard.lambdaQC_outboard);
    geo.canard.Zs_outboard = geo.canard.rs_outboard.*cos(geo.canard.lambdaC2_outboard);  
    geo.canard.Rt_tip = eq_wbox_t(aircraft.Canard.airfoilTip,[pdcylin.spar.CA.Aft_CA_spar_loc_tip, pdcylin.spar.CA.Fore_CA_spar_loc_tip]);
    t_tip = geo.canard.CT *geo.canard.Rt_tip;
    geo.canard.tbs_outboard = t_kink - geo.canard.y_outboard./geo.canard.bS_outboard*(t_kink - t_tip);
    %
    % CAERO1 parameters        
    geo.canard.CAERO1.span     = [geo.canard.CAERO1.span    ; geo.canard.span_outboard];    
    geo.canard.CAERO1.bS       = [geo.canard.CAERO1.bS      ; geo.canard.bS_outboard];    
    geo.canard.CAERO1.SELL     = [geo.canard.CAERO1.SELL    ; geo.canard.SELL_outboard];    
    geo.canard.CAERO1.dy       = [geo.canard.CAERO1.dy      ; geo.canard.dy_outboard];    
    geo.canard.CAERO1.dihedral = [geo.canard.CAERO1.dihedral; aircraft.Canard.dihedral_outboard];    
    geo.canard.CAERO1.sweepLE  = [geo.canard.CAERO1.sweepLE ; aircraft.Canard.LE_sweep_outboard];    
    geo.canard.CAERO1.sweepQC  = [geo.canard.CAERO1.sweepQC ; aircraft.Canard.quarter_chord_sweep_outboard];
    geo.canard.CAERO1.sweepC2  = [geo.canard.CAERO1.sweepC2 ; geo.canard.lambdaC2_outboard *180/pi];
    geo.canard.CAERO1.n        = [geo.canard.CAERO1.n       ; pdcylin.stick.ncanard_outboard];
    geo.canard.CAERO1.n_coarse = [geo.canard.CAERO1.n_coarse       ; pdcylin.stick.ncanard_outboard_coarse];   
    geo.canard.CAERO1.airfoil  = [geo.canard.CAERO1.airfoil; {aircraft.Canard.airfoilKink};...
                                                           {aircraft.Canard.airfoilTip}];
    
    % Fraction chord and name of control surface
    if aircraft.Canard.Elevator.present ==1
        can2lab = canlab;
        can2lab(can2lab=='1') = '2';
        frc = aircraft.Canard.Elevator.chord;
        if frc > 1
            frc = frc / 100;
        end
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; frc];
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; frc];
        geo.canard.CAERO1.sup_control.nme = [geo.canard.CAERO1.sup_control.nme; can2lab];
        % AELINK card: set to -1 (flap)
        geo.canard.CAERO1.sup_control.typ = [geo.canard.CAERO1.sup_control.typ; -1];
    else
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; 0];
        geo.canard.CAERO1.sup_control.frc = [geo.canard.CAERO1.sup_control.frc; 0];
        geo.canard.CAERO1.sup_control.nme = [geo.canard.CAERO1.sup_control.nme; '    none'];
        geo.canard.CAERO1.sup_control.typ = [geo.canard.CAERO1.sup_control.typ; 0];
    end
    
    %
else
    
    geo.canard.span_outboard = 0.0;
    % Angles
    geo.canard.lambdaQC_outboard = 0.0; %aircraft.Canard.quarter_chord_sweep_outboard*(pi/180);
    geo.canard.lambdaLE_outboard = 0.0; %aircraft.Canard.LE_sweep_outboard*(pi/180);
    geo.canard.dihedral_outboard = 0.0; %aircraft.Canard.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    geo.canard.dihedral_outboard  = 0;
    geo.canard.incidence_outboard = [];
    geo.canard.bS_outboard = 0.0;
    % Aerodynamic exposed area at outboard section
    geo.canard.SELL_outboard = 0.0;
    
end
% 
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% Save inboard and outboard parameters refering to correspondent parameter
%---------------------------------------------------------------------------------------------------------------------------

% From inboard
if (geo.canard.span_inboard > MIN)
    
    geo.canard.y         = geo.canard.y_inboard;
    geo.canard.r         = geo.canard.r_inboard;
    geo.canard.rs         = geo.canard.rs_inboard;
    geo.canard.Z         = geo.canard.Z_inboard;
    geo.canard.Zs         = geo.canard.Zs_inboard;
    geo.canard.tbs       = geo.canard.tbs_inboard;
    geo.canard.incidence = geo.canard.incidence_inboard;    
    geo.canard.dy        = geo.canard.dy_inboard .*ones( pdcylin.guess.canard.inboard+1, 1);
    geo.canard.Swet = geo.canard.Swet_inboard;
    geo.canard.x         = geo.canard.x_inboard;
    geo.canard.spar_frac = geo.canard.spar_frac_inboard;
end

% From outboard
if (geo.canard.span_outboard > MIN)
    
    if (geo.canard.span_inboard > MIN)
        
        geo.canard.y         = [geo.canard.y        ; geo.canard.y(end)+geo.canard.y_outboard( 2:end )];
        geo.canard.r         = [geo.canard.r        ; geo.canard.r_outboard( 2:end )];
        geo.canard.rs         = [geo.canard.rs        ; geo.canard.rs_outboard( 2:end )];
        geo.canard.Z         = [geo.canard.Z        ; geo.canard.Z_outboard( 2:end )];
        geo.canard.Zs         = [geo.canard.Zs        ; geo.canard.Zs_outboard( 2:end )];
        geo.canard.tbs       = [geo.canard.tbs      ; geo.canard.tbs_outboard( 2:end )];
        geo.canard.incidence = [geo.canard.incidence; geo.canard.incidence_outboard( 2:end )];        
        geo.canard.dy        = [geo.canard.dy       ; geo.canard.dy_outboard .*ones( pdcylin.guess.canard.outboard, 1)];
        geo.canard.Swet      = [geo.canard.Swet; geo.canard.Swet_outboard];
        geo.canard.x         = [geo.canard.x        ; geo.canard.x_outboard( 2:end )];
        geo.canard.spar_frac = [geo.canard.spar_frac; geo.canard.spar_frac_outboard( 2:end )];

    else
        
        geo.canard.y         = geo.canard.y_outboard;
        geo.canard.r         = geo.canard.r_outboard;
        geo.canard.rs         = geo.canard.rs_outboard;
        geo.canard.Z         = geo.canard.Z_outboard;
        geo.canard.Zs         = geo.canard.Zs_outboard;
        geo.canard.tbs       = geo.canard.tbs_outboard;
        geo.canard.incidence = geo.canard.incidence_outboard;        
        geo.canard.dy        = geo.canard.dy_outboard .*ones( pdcylin.guess.canard.outboard+1, 1);
        geo.canard.Swet      = geo.canard.Swet_outboard;
        geo.canard.x         = geo.canard.x_outboard; 
        geo.canard.spar_frac = geo.canard.spar_frac_outboard; 
    end
    
end

% Wetted surface vector [n_bar x 1]: account for upper and lower surface
geo.canard.Swet = 2*geo.canard.Swet;
    
% Convert incidence angle
geo.canard.incidence = geo.canard.incidence .*(pi/180);
% Exposed area for total semi-wing
geo.canard.SELL = geo.canard.SELL_inboard + geo.canard.SELL_outboard;
% Number of nodes used to discretize structural semi-wing
geo.canard.leny = length(geo.canard.y);
% Carrythrough thickness
geo.canard.tcs = max(geo.canard.tbs);
% Length of structural line as summation of each single sector
geo.canard.bS = geo.canard.bS_inboard + geo.canard.bS_outboard;
%
%---------------------------------------------------------------------------------------------------------------------------

%---------------------------------------------------------------------------------------------------------------------------
% CAERO1 quantities: chord, taper, incidence
%
geo.canard.CAERO1.chord = [geo.canard.CAERO1.chord; geo.canard.CR];
geo.canard.CAERO1.incidence = [geo.canard.CAERO1.incidence; aircraft.Canard.root_incidence];
%
if (geo.canard.span_outboard > MIN) && (geo.canard.span_inboard > MIN)
    geo.canard.CAERO1.chord = [geo.canard.CAERO1.chord; geo.canard.CR_kink];
    geo.canard.CAERO1.incidence = [geo.canard.CAERO1.incidence; aircraft.Canard.kink_incidence];
end
%
geo.canard.CAERO1.chord     = [geo.canard.CAERO1.chord; geo.canard.CT];
geo.canard.CAERO1.incidence = [geo.canard.CAERO1.incidence; aircraft.Canard.tip_incidence];
geo.canard.CAERO1.taper     = (geo.canard.CAERO1.chord(2:end)./geo.canard.CAERO1.chord(1:end-1));

%*****************************************
if isequal(pdcylin.stick.model.symmXZ, 1)
    nrsp = length(geo.canard.CAERO1.taper);
    for i = 1:nrsp
        % copy the name
        geo.canard.CAERO1.sup_control.nme(i+nrsp,:) = geo.canard.CAERO1.sup_control.nme(i,:);
        % change the last letter
        geo.canard.CAERO1.sup_control.nme(i+nrsp,end) = 'l';
    end
end
%*****************************************

%
%---------------------------------------------------------------------------------------------------------------------------


%---------------------------------------------------------------------------------------------------------------------------
% Canard structural concept
%---------------------------------------------------------------------------------------------------------------------------
% Load txt-input file
wcoef = load('strwingcoef.txt');

if pdcylin.canard.kcon <=6 

    geo.canard.ep  = wcoef( pdcylin.canard.kcon, 1 );
    geo.canard.e   = wcoef( pdcylin.canard.kcon, 2 );
    geo.canard.epc = wcoef( pdcylin.canard.kcon, 3 );
    geo.canard.ec  = wcoef( pdcylin.canard.kcon, 4 );
    geo.canard.epw = wcoef( pdcylin.canard.kcon, 5 );
    geo.canard.Kgc = wcoef( pdcylin.canard.kcon, 6 );
    geo.canard.Kgw = wcoef( pdcylin.canard.kcon, 7 );

else
    geo.canard.ep  = 0;
    geo.canard.e   = 0;
    geo.canard.epc = 0;
    geo.canard.ec  = 0;
    geo.canard.epw = 0;
    geo.canard.Kgc = 0;
    geo.canard.Kgw = 0;
    
end



%
%---------------------------------------------------------------------------------------------------------------------------
