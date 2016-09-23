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
geo.wing.b                  = [];    % wing span                                                              [m]  , scalar
geo.wing.CRp                = [];    % wing chord at symmetry plane                                           [m]  , scalar
geo.wing.CR                 = [];    % wing chord at wing-fuselage connection                                 [m]  , scalar
geo.wing.CR_kink1           = [];    % wing chord at kink 1                                                   [m]  , scalar
geo.wing.CR_kink2           = [];    % wing chord at kink 2                                                   [m]  , scalar
geo.wing.CT                 = [];    % wing chord at tip                                                      [m]  , scalar
%
% For the defined sectors, quantities are stored in appropriate vectors
geo.wing.CAERO1.dihedral    = [];    % dihedral for each defined sector                                       [deg], vector
geo.wing.CAERO1.sup_control.frc = [];% Fraction chord for the control surface                                 [-]  , vector
geo.wing.CAERO1.sup_control.frs = [];% Span fraction for the control surface                                  [-]  , vector
geo.wing.CAERO1.sup_control.nme = [];% Name of control surface                                                [-]  , vector
geo.wing.CAERO1.sup_control.typ = [];% Type used for the AELINK card, gets value -1,0,1                       [-]  , vector
geo.wing.CAERO1.airfoil     = {};    % airfoil designation for CAERO surfaces                                 [-]  , cell
geo.wing.CAERO1.sup_control.position = [1];
geo.wing.CAERO1.twist = [];
geo.wing.PANE               = [];    % points to write CAERO1 card
%**************************************************************************
%
lab = TRIMlabels;
labels = lab(:, 1);
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
% 
geo.wing.index = 1;
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% inboard
%--------------------------------------------------------------------------------------------------------------------------
flap1lab = str2_8ch_right(labels{17});

if (geo.wing.span_inboard > MIN)

    % Aerodynamic wing chord at kink 1
    geo.wing.CR_kink1 = geo.wing.CRp *aircraft.wing1.taper_kink1;
    % Angles in [rad]
    geo.wing.lambdaQC_inboard = aircraft.wing1.quarter_chord_sweep_inboard *(pi/180);
    geo.wing.dihedral_inboard = aircraft.wing1.dihedral_inboard *(pi/180);
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
    [fore_spar, aft_spar] = root_spar(geo.wing.CRp, geo.wing.CR_kink1, geo.wing.CR, ...
                    aircraft.fuel.Fore_wing_spar_loc_root, aircraft.fuel.Aft_wing_spar_loc_root, ...
                    aircraft.fuel.Fore_wing_spar_loc_kik1, aircraft.fuel.Aft_wing_spar_loc_kin1, ...
                    geo.fus.R, aircraft.wing1.spanwise_kink1*geo.wing.b/2, geo.wing.lambdaQC_inboard);
    pane = aero_geo(c, b, T, SW, dihed, ox, oy, oz);
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************
    %
    % CAERO1 parameters
    geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist; TW(1,1)];
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_inboard];
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
else

    geo.wing.span_inboard = 0.0;
    
end
%--------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------
% midboard
%--------------------------------------------------------------------------------------------------------------------------
flap2lab = str2_8ch_right(labels{18});

if (geo.wing.span_midboard > MIN)
    % Aerodynamic wing chord at kink 2
    geo.wing.CR_kink2 = geo.wing.CRp *aircraft.wing1.taper_kink2;
    
    if (geo.wing.span_inboard <= MIN)
        % Update outboard span and move inboard sector to the intersection
        geo.wing.span_midboard = aircraft.wing1.spanwise_kink2 *(geo.wing.b/2) - geo.fus.R;
    end   
    
    % Angles
    geo.wing.lambdaQC_midboard = aircraft.wing1.quarter_chord_sweep_midboard *(pi/180);
    geo.wing.dihedral_midboard = aircraft.wing1.dihedral_midboard *(pi/180);   
    
    %**********************************************************************************************
    c = geo.wing.CR_kink1;
    b = geo.wing.span_midboard;
    T = geo.wing.CR_kink2/c;
    TW(1,1) = aircraft.wing1.kink1_incidence*pi/180;
    TW(2,1) = aircraft.wing1.kink2_incidence*pi/180;
    SW = geo.wing.lambdaQC_midboard;
    dihed = geo.wing.dihedral_midboard;
    if (geo.wing.span_inboard <= MIN)
        ox = geo.wing.xLE + geo.fus.R*tan(geo.wing.lambdaQC_midboard); %% ????
        oy   = 0;
        oz   = aircraft.wing1.vertical_location;
    else
        ox   = geo.wing.PANE(1,end-2);
        oy   = geo.wing.PANE(2,end-2);
        oz   = geo.wing.PANE(3,end-2);
    end

    pane = aero_geo(c, b, T, SW, dihed, ox, oy, oz);
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************
    geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist; TW(1,1)];
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_midboard];   
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

else
    
    geo.wing.span_midboard = 0.0;
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
        end
    end
    
    % Angles
    geo.wing.lambdaQC_outboard = aircraft.wing1.quarter_chord_sweep_outboard*(pi/180);
    geo.wing.lambdaLE_outboard = aircraft.wing1.LE_sweep_outboard*(pi/180);
    geo.wing.dihedral_outboard = aircraft.wing1.dihedral_outboard*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    %**********************************************************************************************
    c = geo.wing.CR_kink2;
    b = geo.wing.span_outboard;
    T = geo.wing.CT/c;
    TW(1,1) = aircraft.wing1.kink2_incidence*pi/180;
    TW(2,1) = aircraft.wing1.tip_incidence*pi/180;
    SW = geo.wing.lambdaQC_outboard;
    dihed = geo.wing.dihedral_outboard;
    if (geo.wing.span_inboard <= MIN) && (geo.wing.span_midboard <= MIN)
        ox = geo.wing.xLE + geo.fus.R*tan(geo.wing.lambdaQC_outboard); %% ????
        oy = 0;
        oz = aircraft.wing1.vertical_location;
    else
        ox   = geo.wing.PANE(1,end-2);
        oy   = geo.wing.PANE(2,end-2);
        oz   = geo.wing.PANE(3,end-2);
    end
    
    pane = aero_geo(c, b, T, SW, dihed, ox, oy, oz);
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************   
    geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist; TW(1,1)];
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.wing1.dihedral_outboard]; 
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
        %geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; frc];
        % Span fraction
        geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; aircraft.wing1.aileron.Span];
        geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; aileronlab];
        
        % AELINK card: set to 1 (aileron)
        geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 1];
        geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;aircraft.wing1.aileron.position];
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

end
geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist; aircraft.wing1.tip_incidence*pi/180];
%--------------------------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------------------------
% winglet
%--------------------------------------------------------------------------------------------------------------------------
if (aircraft.winglet.present == 1) && aircraft.winglet.Span>0
    
    
    
    geo.wing.index = [geo.wing.index; geo.wing.index(end)];
    % Angles
    geo.wing.lambdaQC_winglet = atan2(aircraft.winglet.Span*tan(aircraft.winglet.LE_sweep*pi/180) + 0.25*(aircraft.winglet.taper_ratio-1)*geo.wing.CT , aircraft.winglet.Span);
    geo.wing.dihedral_winglet = aircraft.winglet.Cant_angle*(pi/180);
    % Angle of incidence [deg], at the end it's converted
    
    %**********************************************************************************************
    c = geo.wing.CT;
    b = aircraft.winglet.Span;
    T = aircraft.winglet.taper_ratio;
    TW(1,1) = aircraft.winglet.root_incidence*pi/180;
    TW(2,1) = aircraft.winglet.tip_incidence*pi/180;
    SW = geo.wing.lambdaQC_winglet;
    dihed = geo.wing.dihedral_winglet;
    ox = geo.wing.PANE( 1,end-2 );
    oy = geo.wing.PANE( 2,end-2 );
    oz = geo.wing.PANE( 3,end-2);
    pane = aero_geo(c, b, T, SW, dihed, ox, oy, oz);
    geo.wing.PANE = [geo.wing.PANE pane];
    %**********************************************************************************************    
    geo.wing.CAERO1.twist = [geo.wing.CAERO1.twist; TW(1,1)];
    geo.wing.CAERO1.dihedral = [geo.wing.CAERO1.dihedral; aircraft.winglet.Cant_angle]; 
    geo.wing.CAERO1.airfoil  = [geo.wing.CAERO1.airfoil; {aircraft.wing1.airfoilTip};
                                                         {aircraft.wing1.airfoilTip}];
    
    geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    %geo.wing.CAERO1.sup_control.frc = [geo.wing.CAERO1.sup_control.frc; 0];
    % Span fraction
    geo.wing.CAERO1.sup_control.frs = [geo.wing.CAERO1.sup_control.frs; 1];
    geo.wing.CAERO1.sup_control.nme = [geo.wing.CAERO1.sup_control.nme; '    none'];
    
    % AELINK card: set to 1 (aileron)
    geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
    
  
    geo.wing.CAERO1.sup_control.typ = [geo.wing.CAERO1.sup_control.typ; 0];
    geo.wing.CAERO1.sup_control.position = [geo.wing.CAERO1.sup_control.position;1];


end

%
%--------------------------------------------------------------------------------------------------------------------------

end
%
%--------------------------------------------------------------------------------------------------------------------------
function pane = aero_geo(c, b, T, SW, dihed, ox, oy, oz)

lem(1) =  0.25*c;
lem(2) =  0.25*T*c;
lem(3) = -0.75*T*c;
lem(4) = -0.75*c;
%
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
pane = [panex; paney; panez];

end