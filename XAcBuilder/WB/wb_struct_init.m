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

function aircraft = wb_struct_init(aircraft)
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080918      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function       aircraft = wb_struct_init(aircraft)
%
%
%   DESCRIPTION:  Initialize struct to correctly interface to WB module
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                aircraft       struct     input aircraft struct
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                aircraft       struct     updated aircraft
%         
%                
%    REFERENCES:
%
%**************************************************************************


%--------------------------------------------------------------------------
% PARAMETERS INPUT ONLY MODIFIED BY THE USER OR COMING FROM OTHER MODULES
% NEVER MODIFIED BY WB
%--------------------------------------------------------------------------

% Useful to set baggage volume scale factor
aircraft.miscellaneous.Design_classification = 1;

% 0 = under wings
aircraft.miscellaneous.Undercarriage_layout  = 0.;

%**************************************************************************
% LR 18/09/08
%
% Main and auxiliary landing gear cg position
aircraft.miscellaneous.main_landing_gear_x_cg  = 0.;
aircraft.miscellaneous.main_landing_gear_z_cg  = 0.;
aircraft.miscellaneous.aux_landing_gear_x_cg  = 0.;
aircraft.miscellaneous.aux_landing_gear_z_cg  = 0.;

% Box elastic axis position (chord percentage)
aircraft.fuel.box_ea_loc_root  = 0.;
aircraft.fuel.box_ea_loc_kink1 = 0.;
aircraft.fuel.box_ea_loc_kink2 = 0.;
aircraft.fuel.box_ea_loc_tip   = 0.;

% Wing box semi span (chord percentage)
aircraft.fuel.box_semispan_root  = 0.;
aircraft.fuel.box_semispan_kink1 = 0.;
aircraft.fuel.box_semispan_kink2 = 0.;
aircraft.fuel.box_semispan_tip   = 0.;
%
%**************************************************************************

% Aerodynamic parameter
aircraft.wing.Fractional_change_vortex_induced_drag_factor = 0.;

% stability parameter
aircraft.stability.All_up_weight = 0.;


%**************************************************************************
% LR 18/03/09
%
% Flight envelope
aircraft.weight_balance.flight_envelope_prediction.VD_Flight_envelope_dive =  365;
aircraft.weight_balance.flight_envelope_prediction.VMO_Flight_envelope     =  285;
%
%**************************************************************************


%--------------------------------------------------------------------------
% PARAMETERS INPUT ONLY MODIFIED BY THE USER / NEVER MODIFIED BY WB
%--------------------------------------------------------------------------
aircraft.weight_balance.Fuel.Maximum_fuel_in_wings           = 0.;
aircraft.weight_balance.Fuel.Maximum_fuel_in_auxiliary       = 0.;
aircraft.weight_balance.Fuel.Maximum_fuel_in_central_wingbox = 0.;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% WEIGHT & BALANCE MODULE PARAMETERS definition and assignement 
%--------------------------------------------------------------------------

aircraft.designation = 1;

%--------------------------------------------------------------------------
% Modified if null value is given
%--------------------------------------------------------------------------
aircraft.weight_balance.System.Compl_allowance_plus_paint = 0;
aircraft.weight_balance.System.Other_operating_items      = 0;
aircraft.weight_balance.MFW_decrement_to_MTOW             = 1.;
aircraft.weight_balance.Year_advan_techn_multip           = 1993;
% Systems

% LR 18/09/08 added auxiliary landing gear weight
aircraft.weight_balance.System.Aux_Landing_gear = 0.;

aircraft.weight_balance.System.Fuel_system_weight         = 0; 
aircraft.weight_balance.System.Flight_controls_weight     = 0; 
aircraft.weight_balance.System.APU_weight                 = 0.; 
aircraft.weight_balance.System.Instruments_weight         = 0; 
aircraft.weight_balance.System.Avionics_weight            = 0; 
aircraft.weight_balance.System.Hydraulic_Pneumatic_weight = 0; 
aircraft.weight_balance.System.Electrical_weight          = 0.; 
aircraft.weight_balance.System.ECS_anti_icing_grp_weight  = 0.;
aircraft.weight_balance.System.Furnishings_Green          = 0.;
aircraft.weight_balance.System.Miscellaneous              = 0;
aircraft.weight_balance.Struct1_x_cg                      = 0.;
aircraft.weight_balance.Struct1_y_cg                      = 0.;
aircraft.weight_balance.Struct1_z_cg                      = 0.;
aircraft.weight_balance.Struct.Horizontal_tail_x_cg       = 0.;
aircraft.weight_balance.Struct.Horizontal_tail_y_cg       = 0.;
aircraft.weight_balance.Struct.Horizontal_tail_z_cg       = 0.;
aircraft.weight_balance.Struct.Vertical_tail_x_cg         = 0.;
aircraft.weight_balance.Struct.Vertical_tail_y_cg         = 0.;
aircraft.weight_balance.Struct.Vertical_tail_z_cg         = 0.;
aircraft.weight_balance.Struct.Fuselage_structure_x_cg    = 0.;
aircraft.weight_balance.Struct.Fuselage_structure_y_cg    = 0.;
aircraft.weight_balance.Struct.Fuselage_structure_z_cg    = 0.;
% Powerplant CGs
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_x_cg   = 0.;
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_y_cg   = 0.;
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_z_cg   = 0.;
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_x_cg   = 0.;
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_y_cg   = 0.;
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_z_cg   = 0.;
% Systems CGs
aircraft.weight_balance.System.Total_systems_or_miscellaneous_x_cg = 0.;
aircraft.weight_balance.System.Total_systems_or_miscellaneous_y_cg = 0.;
aircraft.weight_balance.System.Total_systems_or_miscellaneous_z_cg = 0.;

% Fuel CGs (specified if known)
aircraft.weight_balance.Fuel.Fuel_tank_wing_x_cg                   = 0.;
aircraft.weight_balance.Fuel.Fuel_tank_wing_y_cg                   = 0.;
aircraft.weight_balance.Fuel.Fuel_tank_wing_z_cg                   = 0.;
aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_x_cg          = 0.;
aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_y_cg          = 0.;
aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_z_cg          = 0.;
aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_x_cg              = 0.;
aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_y_cg              = 0.;
aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_z_cg              = 0.;
% Left for compatibility
aircraft.weight_balance.Fuel.Fuel_in_wing_x_cg       = 0.;
aircraft.weight_balance.Fuel.Fuel_in_wing_y_cg       = 0.;
aircraft.weight_balance.Fuel.Fuel_in_wing_z_cg       = 0.;  
aircraft.weight_balance.Fuel.Fuel_in_fairings_x_cg   = 0.; 
aircraft.weight_balance.Fuel.Fuel_in_fairings_y_cg   = 0.; 
aircraft.weight_balance.Fuel.Fuel_in_fairings_z_cg   = 0.; 

% Payload CGs
aircraft.weight_balance.Payload.Interiors_completion_x_cg          = 0.;
aircraft.weight_balance.Payload.Interiors_completion_y_cg          = 0.;
aircraft.weight_balance.Payload.Interiors_completion_z_cg          = 0.;
aircraft.weight_balance.Payload.Passengers_x_cg                    = 0.;
aircraft.weight_balance.Payload.Passengers_y_cg                    = 0.;
aircraft.weight_balance.Payload.Passengers_z_cg                    = 0.;
aircraft.weight_balance.Payload.Baggage_and_cargo_x_cg             = 0.;
aircraft.weight_balance.Payload.Baggage_and_cargo_y_cg             = 0.;
aircraft.weight_balance.Payload.Baggage_and_cargo_z_cg             = 0.;
aircraft.weight_balance.Payload.Passenger_weight_coefficient       = 0; 
aircraft.weight_balance.Payload.Weight_per_passenger               = 0; 
aircraft.weight_balance.Payload.Weight_increment_per_passenger     = 0; 
% Crew CGs
aircraft.weight_balance.Crew.Pilots_x_cg = 0.;
aircraft.weight_balance.Crew.Pilots_y_cg = 0.;
aircraft.weight_balance.Crew.Pilots_z_cg = 0.;

%--------------------------------------------------------------------------
% Always computed independently by its value
%--------------------------------------------------------------------------

aircraft.weight_balance.System.Landing_gear = 0;
aircraft.weight_balance.Struct.Wings = 0;
aircraft.weight_balance.Struct.Winglet_and_span_load_penalty = 0;
aircraft.weight_balance.Struct.Horizontal_tail_and_elevator = 0;
aircraft.weight_balance.Struct.Vertical_tail_rudder_and_dorsal = 0;
aircraft.weight_balance.Struct.Ventral_fins = 0;
aircraft.weight_balance.Struct.Fuselage = 0;
% Powerplant data always computed (unless saiplane is designed)
aircraft.weight_balance.Powerplant.Pylons_and_or_propellers = 0;
aircraft.weight_balance.Powerplant.Engines_acc_propeller_gearbox = 0;
aircraft.weight_balance.Powerplant.Nacelles = 0;
% Crew data always computed
aircraft.weight_balance.Crew.Crew_and_carry_on = 0;
aircraft.weight_balance.Crew.Cabin_attendant_weight = 0;
aircraft.weight_balance.Crew.Flight_crew_weight = 0;
% Fuel data always computed
aircraft.weight_balance.Fuel.Maximum_fuel_weight = 0;
aircraft.weight_balance.Fuel.Fuel_to_MTOW_at_maximum_payload = 0;
% Aircraft general weights
aircraft.weight_balance.MTOW_Maximum_takeoff_weight = 0;
aircraft.weight_balance.OEW_Operational_empty_weight = 0;
aircraft.weight_balance.Green_Manufacturer_empty_weight = 0;
aircraft.weight_balance.Maximum_payload_weight = 0;
aircraft.weight_balance.Maximum_fuel_weight = 0;
aircraft.weight_balance.MRW_Maximum_ramp_weight = 0; 
aircraft.weight_balance.MZFW_Maximum_zero_fuel_weight = 0; 
% Merit
aircraft.weight_balance.Merit.Maximum_payload_per_passenger     = 0;
aircraft.weight_balance.Merit.Payload_to_MTOW_at_MFW            = 0;
aircraft.weight_balance.Merit.Load_factor_to_MTOW_at_MFW        = 0; 
aircraft.weight_balance.Merit.Merit_function_OEW_over_MTOW      = 0; 
aircraft.weight_balance.Merit.Merit_function_W_over_S_gross     = 0; 
aircraft.weight_balance.Merit.Merit_function_thrust_to_weight   = 0; 
% This variables is actually never used in QCARD but it is declared and set
% to null value.
aircraft.weight_balance.Merit.Merit_function_load_parameter = 0;
% Inertia matrix (0 force inertia calculation with coarse approximation
% while -1 means refined calculation)
aircraft.weight_balance.Imat = zeros(3, 3);
aircraft.weight_balance.IXXINER = 0.;
aircraft.weight_balance.IYYINER = 0.;
aircraft.weight_balance.IZZINER = 0.;
aircraft.weight_balance.IXZINER = 0.;
aircraft.weight_balance.IYZINER = 0.;
aircraft.weight_balance.IXYINER = 0.;
% CGs
aircraft.weight_balance.MEW_longitudinal_CoG = 0; 
aircraft.weight_balance.MEW_lateral_CoG      = 0; 
aircraft.weight_balance.MEW_vertical_CoG     = 0;
aircraft.weight_balance.Maximum_payload_at_MTOW_longitudinal_CoG = 0;
aircraft.weight_balance.Maximum_payload_at_MTOW_lateral_CoG = 0;
aircraft.weight_balance.Maximum_payload_at_MTOW_vertical_CoG = 0;
aircraft.weight_balance.Computed_longitudinal_CoG = 0;
aircraft.weight_balance.Computed_lateral_CoG = 0;
aircraft.weight_balance.Computed_vertical_CoG = 0;
% Lr 16/02/2010 - Added Canard structural CoG initialization
aircraft.weight_balance.Struct.Canard_x_cg = 0.;
aircraft.weight_balance.Struct.Canard_y_cg = 0.;
aircraft.weight_balance.Struct.Canard_z_cg = 0.;
% SR 02/06/2010 - Added tailbooms structural CoG initialization
aircraft.weight_balance.Struct.Tailbooms_x_cg = 0.;
aircraft.weight_balance.Struct.Tailbooms_y_cg = 0.;
aircraft.weight_balance.Struct.Tailbooms_z_cg = 0.;
%--------------------------------------------------------------------------
