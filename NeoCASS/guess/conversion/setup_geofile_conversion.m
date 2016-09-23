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
% Convert the inputs necessary to run GUESS
% 
% Called by:    guess.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080407      1.0     A. Da Ronch      Creation
%     091119      1.3.9   L. Travaglini    Modification
%
% There are one struct for canard and one for wing2 without confusing wing2
% with canard
%*******************************************************************************
function OUTPUT = setup_geofile_conversion(INPUT)

%--------------------------------------------------------------------------
% --Fuselage--
OUTPUT.fuselage = INPUT.Fuselage;
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Wing--
%
OUTPUT.wing1.present                      = INPUT.Wing1.present;
OUTPUT.wing1.configuration                = INPUT.Wing1.configuration;
OUTPUT.wing1.longitudinal_location        = INPUT.Wing1.longitudinal_location;
OUTPUT.wing1.vertical_location            = INPUT.Wing1.vertical_location;
OUTPUT.wing1.area                         = INPUT.Wing1.area;
OUTPUT.wing1.span                         = INPUT.Wing1.Span;
OUTPUT.wing1.spanwise_kink1               = INPUT.Wing1.spanwise_kink1;
OUTPUT.wing1.spanwise_kink2               = INPUT.Wing1.spanwise_kink2;
OUTPUT.wing1.taper_kink1                  = INPUT.Wing1.taper_kink1;
OUTPUT.wing1.taper_kink2                  = INPUT.Wing1.taper_kink2;
OUTPUT.wing1.taper_tip                    = INPUT.Wing1.taper_tip;
OUTPUT.wing1.thickness_root               = INPUT.Wing1.thickness_root;
OUTPUT.wing1.thickness_kink1              = INPUT.Wing1.thickness_kink1;
OUTPUT.wing1.thickness_kink2              = INPUT.Wing1.thickness_kink2;
OUTPUT.wing1.thickness_tip                = INPUT.Wing1.thickness_tip;
OUTPUT.wing1.quarter_chord_sweep_inboard  = INPUT.Wing1.quarter_chord_sweep_inboard;
OUTPUT.wing1.quarter_chord_sweep_midboard = INPUT.Wing1.quarter_chord_sweep_midboard;
OUTPUT.wing1.quarter_chord_sweep_outboard = INPUT.Wing1.quarter_chord_sweep_outboard;
OUTPUT.wing1.LE_sweep_inboard             = INPUT.Wing1.LE_sweep_inboard;
OUTPUT.wing1.LE_sweep_midboard            = INPUT.Wing1.LE_sweep_midboard;
OUTPUT.wing1.LE_sweep_outboard            = INPUT.Wing1.LE_sweep_outboard;
OUTPUT.wing1.dihedral_inboard             = INPUT.Wing1.dihedral_inboard;
OUTPUT.wing1.dihedral_midboard            = INPUT.Wing1.dihedral_midboard;
OUTPUT.wing1.dihedral_outboard            = INPUT.Wing1.dihedral_outboard;
OUTPUT.wing1.root_incidence               = INPUT.Wing1.root_incidence;
OUTPUT.wing1.kink1_incidence              = INPUT.Wing1.kink1_incidence;
OUTPUT.wing1.kink2_incidence              = INPUT.Wing1.kink2_incidence;
OUTPUT.wing1.tip_incidence                = INPUT.Wing1.tip_incidence;
OUTPUT.wing1.airfoilRoot                  = INPUT.Wing1.airfoilRoot;
OUTPUT.wing1.airfoilKink1                 = INPUT.Wing1.airfoilKink1;
OUTPUT.wing1.airfoilKink2                 = INPUT.Wing1.airfoilKink2;
OUTPUT.wing1.airfoilTip                   = INPUT.Wing1.airfoilTip;
OUTPUT.wing1.flap                         = INPUT.Wing1.flap;     % Struct
OUTPUT.wing1.aileron                      = INPUT.Wing1.aileron;  % Struct
OUTPUT.wing1.Span_matrix_partition_in_mid_outboard = INPUT.Wing1.Span_matrix_partition_in_mid_outboard;
OUTPUT.wing1.apex_locale                  = INPUT.Wing1.apex_locale;
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Vertical_tail--
%
if INPUT.Vertical_tail.present
    
    OUTPUT.Vertical_tail.present = INPUT.Vertical_tail.present;
    if isfield(INPUT.Vertical_tail, 'Twin_tail')
        OUTPUT.Vertical_tail.Twin_tail      = INPUT.Vertical_tail.Twin_tail;
        OUTPUT.Vertical_tail.Twin_tail_span = INPUT.Vertical_tail.Twin_tail_span;
    else
        OUTPUT.Vertical_tail.Twin_tail      = 0;
        OUTPUT.Vertical_tail.Twin_tail_span = 0;
    end
    OUTPUT.Vertical_tail.area                               = INPUT.Vertical_tail.area;
    OUTPUT.Vertical_tail.longitudinal_location              = INPUT.Vertical_tail.longitudinal_location;
    OUTPUT.Vertical_tail.vertical_location                  = INPUT.Vertical_tail.vertical_location;
    OUTPUT.Vertical_tail.span                               = INPUT.Vertical_tail.Span;
    OUTPUT.Vertical_tail.spanwise_kink                      = INPUT.Vertical_tail.spanwise_kink;
    OUTPUT.Vertical_tail.taper_kink                         = INPUT.Vertical_tail.taper_kink;
    OUTPUT.Vertical_tail.taper_tip                          = INPUT.Vertical_tail.taper_tip;
    OUTPUT.Vertical_tail.thickness_root                     = INPUT.Vertical_tail.thickness_root;
    OUTPUT.Vertical_tail.thickness_kink                     = INPUT.Vertical_tail.thickness_kink;
    OUTPUT.Vertical_tail.thickness_tip                      = INPUT.Vertical_tail.thickness_tip;
    OUTPUT.Vertical_tail.quarter_chord_sweep_inboard        = INPUT.Vertical_tail.quarter_chord_sweep_inboard;
    OUTPUT.Vertical_tail.quarter_chord_sweep_outboard       = INPUT.Vertical_tail.quarter_chord_sweep_outboard;
    OUTPUT.Vertical_tail.LE_sweep_inboard                   = INPUT.Vertical_tail.LE_sweep_inboard;
    OUTPUT.Vertical_tail.LE_sweep_outboard                  = INPUT.Vertical_tail.LE_sweep_outboard;
    OUTPUT.Vertical_tail.airfoilRoot                        = INPUT.Vertical_tail.airfoilRoot;
    OUTPUT.Vertical_tail.airfoilKink                        = INPUT.Vertical_tail.airfoilKink;
    OUTPUT.Vertical_tail.airfoilTip                         = INPUT.Vertical_tail.airfoilTip;
    OUTPUT.Vertical_tail.Rudder                             = INPUT.Vertical_tail.Rudder; % Struct
    OUTPUT.Vertical_tail.reference_wing_area                = INPUT.Vertical_tail.reference_wing_area;
    OUTPUT.Vertical_tail.reference_wing_taper_ratio         = INPUT.Vertical_tail.reference_wing_taper_ratio;
    OUTPUT.Vertical_tail.reference_wing_quarter_chord_sweep = INPUT.Vertical_tail.reference_wing_quarter_chord_sweep;
    OUTPUT.Vertical_tail.original_root_chord                = INPUT.Vertical_tail.original_root_chord;
    OUTPUT.Vertical_tail.reference_Y_bar_non_dim            = INPUT.Vertical_tail.reference_Y_bar_non_dim;
    OUTPUT.Vertical_tail.Moment_arm_to_VT                   = INPUT.Vertical_tail.Moment_arm_to_VT;
    OUTPUT.Vertical_tail.dihedral_inboard                   = INPUT.Vertical_tail.dihedral_inboard;
    OUTPUT.Vertical_tail.dihedral_outboard                  = INPUT.Vertical_tail.dihedral_outboard;
    
else
    OUTPUT.Vertical_tail.present = 0;
end
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Horizontal_tail--
%
if INPUT.Horizontal_tail.present

    OUTPUT.Horizontal_tail.present                         = INPUT.Horizontal_tail.present;
    OUTPUT.Horizontal_tail.vertical_location               = INPUT.Horizontal_tail.vertical_location;
    OUTPUT.Horizontal_tail.longitudinal_location           = INPUT.Horizontal_tail.longitudinal_location;
    OUTPUT.Horizontal_tail.area                            = INPUT.Horizontal_tail.area;
    OUTPUT.Horizontal_tail.span                            = INPUT.Horizontal_tail.Span;
    OUTPUT.Horizontal_tail.spanwise_kink                   = INPUT.Horizontal_tail.spanwise_kink;
    OUTPUT.Horizontal_tail.taper_kink                      = INPUT.Horizontal_tail.taper_kink;
    OUTPUT.Horizontal_tail.taper_tip                       = INPUT.Horizontal_tail.taper_tip;
    OUTPUT.Horizontal_tail.thickness_root                  = INPUT.Horizontal_tail.thickness_root;
    OUTPUT.Horizontal_tail.thickness_kink                  = INPUT.Horizontal_tail.thickness_kink;
    OUTPUT.Horizontal_tail.thickness_tip                   = INPUT.Horizontal_tail.thickness_tip;
    OUTPUT.Horizontal_tail.quarter_chord_sweep_inboard     = INPUT.Horizontal_tail.quarter_chord_sweep_inboard;
    OUTPUT.Horizontal_tail.quarter_chord_sweep_outboard    = INPUT.Horizontal_tail.quarter_chord_sweep_outboard;
    OUTPUT.Horizontal_tail.LE_sweep_inboard                = INPUT.Horizontal_tail.LE_sweep_inboard;
    OUTPUT.Horizontal_tail.LE_sweep_outboard               = INPUT.Horizontal_tail.LE_sweep_outboard;
    OUTPUT.Horizontal_tail.dihedral_inboard                = INPUT.Horizontal_tail.dihedral_inboard;
    OUTPUT.Horizontal_tail.dihedral_outboard               = INPUT.Horizontal_tail.dihedral_outboard;
    OUTPUT.Horizontal_tail.root_incidence                  = INPUT.Horizontal_tail.root_incidence;
    OUTPUT.Horizontal_tail.kink_incidence                  = INPUT.Horizontal_tail.kink_incidence;
    OUTPUT.Horizontal_tail.tip_incidence                   = INPUT.Horizontal_tail.tip_incidence;
    OUTPUT.Horizontal_tail.limit_tailplane_deflection_up   = INPUT.Horizontal_tail.limit_tailplane_deflection_up;
    OUTPUT.Horizontal_tail.limit_tailplane_deflection_down = INPUT.Horizontal_tail.limit_tailplane_deflection_down;
    OUTPUT.Horizontal_tail.airfoilRoot                     = INPUT.Horizontal_tail.airfoilRoot;
    OUTPUT.Horizontal_tail.airfoilKink                     = INPUT.Horizontal_tail.airfoilKink;
    OUTPUT.Horizontal_tail.airfoilTip                      = INPUT.Horizontal_tail.airfoilTip;
    OUTPUT.Horizontal_tail.Moment_arm_to_HT                = INPUT.Horizontal_tail.Moment_arm_to_HT;
    OUTPUT.Horizontal_tail.Elevator                        = INPUT.Horizontal_tail.Elevator; % Struct
    OUTPUT.Horizontal_tail.reference_wing_half_chord_sweep = INPUT.Horizontal_tail.reference_wing_half_chord_sweep;
    
else
    OUTPUT.Horizontal_tail.present = 0;
end
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Canard--
%
if INPUT.Canard.present
    
    OUTPUT.Canard.present                         = INPUT.Canard.present;
    OUTPUT.Canard.vertical_location               = INPUT.Canard.z;
    OUTPUT.Canard.longitudinal_location           = INPUT.Canard.x;
    OUTPUT.Canard.area                            = INPUT.Canard.area;
    OUTPUT.Canard.span                            = INPUT.Canard.Span;
    OUTPUT.Canard.spanwise_kink                   = INPUT.Canard.spanwise_kink;
    OUTPUT.Canard.taper_kink                      = INPUT.Canard.taper_kink;
    OUTPUT.Canard.taper_tip                       = INPUT.Canard.taper_tip;
    OUTPUT.Canard.thickness_root                  = INPUT.Canard.thickness_root;
    OUTPUT.Canard.thickness_kink                  = INPUT.Canard.thickness_kink;
    OUTPUT.Canard.thickness_tip                   = INPUT.Canard.thickness_tip;
    OUTPUT.Canard.quarter_chord_sweep_inboard     = INPUT.Canard.quarter_chord_sweep_inboard;
    OUTPUT.Canard.quarter_chord_sweep_outboard    = INPUT.Canard.quarter_chord_sweep_outboard;
    OUTPUT.Canard.LE_sweep_inboard                = INPUT.Canard.LE_sweep_inboard;
    OUTPUT.Canard.LE_sweep_outboard               = INPUT.Canard.LE_sweep_outboard;
    OUTPUT.Canard.dihedral_inboard                = INPUT.Canard.dihedral_inboard;
    OUTPUT.Canard.dihedral_outboard               = INPUT.Canard.dihedral_outboard;
    OUTPUT.Canard.root_incidence                  = INPUT.Canard.root_incidence;
    OUTPUT.Canard.kink_incidence                  = INPUT.Canard.kink_incidence;
    OUTPUT.Canard.tip_incidence                   = INPUT.Canard.tip_incidence;
    OUTPUT.Canard.limit_tailplane_deflection_up   = INPUT.Canard.limit_tailplane_deflection_up;
    OUTPUT.Canard.limit_tailplane_deflection_down = INPUT.Canard.limit_tailplane_deflection_down;
    OUTPUT.Canard.airfoilRoot                     = INPUT.Canard.airfoilRoot;
    OUTPUT.Canard.airfoilKink                     = INPUT.Canard.airfoilKink;
    OUTPUT.Canard.airfoilTip                      = INPUT.Canard.airfoilTip;
    OUTPUT.Canard.Moment_arm_to_CA                = INPUT.Canard.Moment_arm_to_CA;
    OUTPUT.Canard.Elevator                        = INPUT.Canard.Elevator; % Struct
    OUTPUT.Canard.reference_wing_half_chord_sweep = INPUT.Canard.reference_wing_half_chord_sweep;

else
    OUTPUT.Canard.present = 0;
end

%--------------------------------------------------------------------------
% --Wing 2--
%
if INPUT.Wing2.present
    
    OUTPUT.wing2.present                      = INPUT.Wing2.present;
    OUTPUT.wing2.configuration                = INPUT.Wing2.configuration;
    OUTPUT.wing2.longitudinal_location        = INPUT.Wing2.longitudinal_location;
    OUTPUT.wing2.vertical_location            = INPUT.Wing2.vertical_location;
    OUTPUT.wing2.area                         = INPUT.Wing2.area;
    OUTPUT.wing2.span                         = INPUT.Wing2.Span;
    OUTPUT.wing2.spanwise_kink1               = INPUT.Wing2.spanwise_kink1;
    OUTPUT.wing2.spanwise_kink2               = INPUT.Wing2.spanwise_kink2;
    OUTPUT.wing2.taper_kink1                  = INPUT.Wing2.taper_kink1;
    OUTPUT.wing2.taper_kink2                  = INPUT.Wing2.taper_kink2;
    OUTPUT.wing2.taper_tip                    = INPUT.Wing2.taper_tip;
    OUTPUT.wing2.thickness_root               = INPUT.Wing2.thickness_root;
    OUTPUT.wing2.thickness_kink1              = INPUT.Wing2.thickness_kink1;
    OUTPUT.wing2.thickness_kink2              = INPUT.Wing2.thickness_kink2;
    OUTPUT.wing2.thickness_tip                = INPUT.Wing2.thickness_tip;
    OUTPUT.wing2.quarter_chord_sweep_inboard  = INPUT.Wing2.quarter_chord_sweep_inboard;
    OUTPUT.wing2.quarter_chord_sweep_midboard = INPUT.Wing2.quarter_chord_sweep_midboard;
    OUTPUT.wing2.quarter_chord_sweep_outboard = INPUT.Wing2.quarter_chord_sweep_outboard;
    OUTPUT.wing2.LE_sweep_inboard             = INPUT.Wing2.LE_sweep_inboard;
    OUTPUT.wing2.LE_sweep_midboard            = INPUT.Wing2.LE_sweep_midboard;
    OUTPUT.wing2.LE_sweep_outboard            = INPUT.Wing2.LE_sweep_outboard;
    OUTPUT.wing2.dihedral_inboard             = INPUT.Wing2.dihedral_inboard;
    OUTPUT.wing2.dihedral_midboard            = INPUT.Wing2.dihedral_midboard;
    OUTPUT.wing2.dihedral_outboard            = INPUT.Wing2.dihedral_outboard;
    OUTPUT.wing2.root_incidence               = INPUT.Wing2.root_incidence;
    OUTPUT.wing2.kink1_incidence              = INPUT.Wing2.kink1_incidence;
    OUTPUT.wing2.kink2_incidence              = INPUT.Wing2.kink2_incidence;
    OUTPUT.wing2.tip_incidence                = INPUT.Wing2.tip_incidence;
    OUTPUT.wing2.airfoilRoot                  = INPUT.Wing2.airfoilRoot;
    OUTPUT.wing2.airfoilKink1                 = INPUT.Wing2.airfoilKink1;
    OUTPUT.wing2.airfoilKink2                 = INPUT.Wing2.airfoilKink2;
    OUTPUT.wing2.airfoilTip                   = INPUT.Wing2.airfoilTip;
    OUTPUT.wing2.flap                         = INPUT.Wing2.flap;     % Struct
    OUTPUT.wing2.aileron                      = INPUT.Wing2.aileron;  % Struct
    
else
    OUTPUT.wing2.present = 0;
end
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Tailbooms--
%
if isfield(INPUT,'Tailbooms') && INPUT.Tailbooms.present
    
    OUTPUT.Tailbooms.present      = INPUT.Tailbooms.present;
    OUTPUT.Tailbooms.symmetry     = INPUT.Tailbooms.symmetry;
    OUTPUT.Tailbooms.total_length = INPUT.Tailbooms.total_length;
    OUTPUT.Tailbooms.diameter     = INPUT.Tailbooms.diameter;
    OUTPUT.Tailbooms.x            = INPUT.Tailbooms.x_location*OUTPUT.fuselage.Total_fuselage_length;
    OUTPUT.Tailbooms.y            = INPUT.Tailbooms.y_location*OUTPUT.wing1.span*0.5;
    OUTPUT.Tailbooms.z            = INPUT.Tailbooms.z;
    
else
    OUTPUT.Tailbooms.present = 0;
end
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Weight_balance--
OUTPUT.weight_balance = INPUT.weight_balance; % struct
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Cabin--
OUTPUT.cabin = INPUT.cabin; % struct
%
% --Baggage--
OUTPUT.Baggage = INPUT.Baggage;
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Refernce wing--
OUTPUT.Reference_wing = INPUT.Reference_wing; % struct
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Fuel--
OUTPUT.fuel = INPUT.fuel; % struct
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Engines1--
OUTPUT.engines1.Number_of_engines              = INPUT.Engines1.Number_of_engines;
OUTPUT.engines1.nacelle_length                 = INPUT.Engines1.Nacelle_length_array;
OUTPUT.engines1.Max_thrust                     = INPUT.Engines1.Max_thrust;
OUTPUT.engines1.Engine_spanwise_location       = INPUT.Engines1.Y_locale;
OUTPUT.engines1.d_max                          = INPUT.Engines1.d_max;
OUTPUT.engines1.Location_engines_nacelles_on_X = INPUT.Engines1.Nacelle1.longitudinal_location;
OUTPUT.engines1.Location_engines_nacelles_on_Y = INPUT.Engines1.Nacelle1.lateral_location;
OUTPUT.engines1.Location_engines_nacelles_on_Z = INPUT.Engines1.Nacelle1.vertical_location;
OUTPUT.engines1.Layout_and_config              = INPUT.Engines1.Layout_and_config; 
OUTPUT.engines1.Propulsion_type                = INPUT.Engines1.Propulsion_type; 
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Engines2--
OUTPUT.engines2.Number_of_engines              = INPUT.Engines2.Number_of_engines;
OUTPUT.engines2.nacelle_length                 = INPUT.Engines1.Nacelle_length_array;
OUTPUT.engines2.Max_thrust                     = INPUT.Engines2.Max_thrust;
OUTPUT.engines2.Engine_spanwise_location       = INPUT.Engines2.Y_locale;
OUTPUT.engines2.d_max                          = INPUT.Engines2.d_max;
OUTPUT.engines2.Location_engines_nacelles_on_X = INPUT.Engines2.Nacelle3.longitudinal_location;
OUTPUT.engines2.Location_engines_nacelles_on_Y = INPUT.Engines2.Nacelle3.lateral_location;
OUTPUT.engines2.Location_engines_nacelles_on_Z = INPUT.Engines2.Nacelle3.vertical_location;
OUTPUT.engines2.Layout_and_config              = INPUT.Engines2.Layout_and_config;
OUTPUT.engines2.Propulsion_type                = INPUT.Engines2.Propulsion_type; 
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% --Main landing gear--
if (isfield(INPUT.miscellaneous, 'main_landing_gear_on_fuselage'))
  OUTPUT.miscellaneous.main_landing_gear_on_fuselage = INPUT.miscellaneous.main_landing_gear_on_fuselage;
else
  OUTPUT.miscellaneous.main_landing_gear_on_fuselage = 0;
end
%
%--------------------------------------------------------------------------


