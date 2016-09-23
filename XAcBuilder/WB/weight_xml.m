function out = weight_xml(in)
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
%     080101      2.1     L.Riccobene      Creation
%
%**************************************************************************
%
% function         out = weight_xml(in)
%
%
%   DESCRIPTION:   Weight and Balance main function (Run it with NEW
%                  version of XML file)
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                in             struct     aircraft struct loaded with
%                                          xml_load from XML file and
%                                          processed by Geo module.
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                out            struct     updated aircraft struct after
%                                          W&B computations
%         
%                
%    REFERENCES:
%
%**************************************************************************

% Read input struct
aircraft = in;

% Aircraft designation
n = 1;
acftidn(1, n) = n; %aircraft.designation; % = n
TOTLWET(1, n) = real(aircraft.Wetted_areas.Total_wetted_area);

global PLOTCGS

%% Add on by Adrien
global STACLAU STACLAD STACLRU STACTPD STACLEU STACLED STACLTU STACLTD
global STACAUW
% % % global FOTEZLF STACSPT STACIFL STACISA
% % % FOTEZLF(1)=0;
% % % STACSPT(1)=0;
% % % STACIFL(1)=0;
% % % STACISA(1)=0;
global IXXINER IYYINER IZZINER IXZINER WINGWEI VTALWEI
global HTALWEI PYLNWEI POWPWEI NACEWEI DMFWWEI PMFWWEI CALCOGX CALCOGZ RAMPWEI
% IXXINER(1)=0;
% IYYINER(1)=0;
% IZZINER(1)=0;
% IXZINER(1)=0;
%%

PLOTCGS = zeros(30, 4, 15);  % initialise the cg locations
%--------------------------------------------------------------------------
% Parameters defined for internal usage
kdrang = 2*pi/360;      % conversion from deg. to rad.
KTAS2MS = 0.5144;       % conversion from KTAS to m/s.
DEG2RAD = 2*pi/360;     % conversion from deg to rad               
PSI2KPA = 6.894757;     % conversion from PSI to KPa
SEA_LEVEL_P = 101.325;  % sea level pressure condition (kPa)
SEA_LEVEL_D = 1.225;    % sea level density (kg/m^3)
NEWTON2LB = 4.45;       % conversion from Newton to lb
g = 9.81;               % gravity acceleration (m/s^2)
DRWSELN = n;            % indicate column radio button on/off (aircraft choice)
CONSELN = 0;            % set to zero consol number

%--------------------------------------------------------------------------
% Call the modified fuel calculation routine to update fuel CoGs
% LR 25/09/08
aircraft = qfucalc_mod(aircraft);

% Check user specified fuel input consistency
aircraft = check_max_fuel_weight(aircraft);

%--------------------------------------------------------------------------
% Miscellaneous
opercei(1, n) = aircraft.miscellaneous.Target_operating_ceiling;
destype(1, n) = aircraft.miscellaneous.Design_classification;
spoiler(1, n) = aircraft.miscellaneous.Spoiler_effectivity;
undrloc(1, n) = aircraft.miscellaneous.Undercarriage_layout;
%--------------------------------------------------------------------------
% Weight & Balance
%--------------------------------------------------------------------------
% System weights
COPAWEI(1, n) = aircraft.weight_balance.System.Compl_allowance_plus_paint;
OPSIWEI(1, n) = aircraft.weight_balance.System.Other_operating_items;
LNDGWEI(1, n) = aircraft.weight_balance.System.Landing_gear;
% LR 18/09/08
ALNDGWEI(1, n) = aircraft.weight_balance.System.Aux_Landing_gear;
FSYSWEI(1, n) = aircraft.weight_balance.System.Fuel_system_weight; 
FCTLWEI(1, n) = aircraft.weight_balance.System.Flight_controls_weight; 
APUSWEI(1, n) = aircraft.weight_balance.System.APU_weight; 
INSTWEI(1, n) = aircraft.weight_balance.System.Instruments_weight; 
AVIOWEI(1, n) = aircraft.weight_balance.System.Avionics_weight; 
HYPNWEI(1, n) = aircraft.weight_balance.System.Hydraulic_Pneumatic_weight; 
ELECWEI(1, n) = aircraft.weight_balance.System.Electrical_weight; 
ECSAWEI(1, n) = aircraft.weight_balance.System.ECS_anti_icing_grp_weight; 
FURNWEI(1, n) = aircraft.weight_balance.System.Furnishings_Green; 
MISCWEI(1, n) = aircraft.weight_balance.System.Miscellaneous;
%-------------------------------------------------------------------------------
% Struct weights
WINGWEI(1, n) = aircraft.weight_balance.Struct.Wings;
WLETWEI(1, n) = aircraft.weight_balance.Struct.Winglet_and_span_load_penalty;
HTALWEI(1, n) = aircraft.weight_balance.Struct.Horizontal_tail_and_elevator;
VTALWEI(1, n) = aircraft.weight_balance.Struct.Vertical_tail_rudder_and_dorsal;
VENTWEI(1, n) = aircraft.weight_balance.Struct.Ventral_fins;
FUSEWEI(1, n) = aircraft.weight_balance.Struct.Fuselage;
%-------------------------------------------------------------------------------
% Power plant weights
PYLNWEI(1, n) = aircraft.weight_balance.Powerplant.Pylons_and_or_propellers;
POWPWEI(1, n) = aircraft.weight_balance.Powerplant.Engines_acc_propeller_gearbox;
NACEWEI(1, n) = aircraft.weight_balance.Powerplant.Nacelles;
%-------------------------------------------------------------------------------
% Crew weight
CREWWEI(1, n) = aircraft.weight_balance.Crew.Crew_and_carry_on;
%-------------------------------------------------------------------------------
% Fuel weights
DMFWWEI(1, n) = aircraft.weight_balance.Fuel.Maximum_fuel_weight;
FMPYWEI(1, n) = aircraft.weight_balance.Fuel.Fuel_to_MTOW_at_maximum_payload;
MFUWWEI(1, n) = aircraft.weight_balance.Fuel.Maximum_fuel_in_wings;
MFAWWEI(1, n) = aircraft.weight_balance.Fuel.Maximum_fuel_in_auxiliary;
MFCWWEI(1, n) = aircraft.weight_balance.Fuel.Maximum_fuel_in_central_wingbox;
%-------------------------------------------------------------------------------
% Aircraft general weights
MTOWWEI(1, n) = aircraft.weight_balance.MTOW_Maximum_takeoff_weight;
DOEWWEI(1, n) = aircraft.weight_balance.OEW_Operational_empty_weight;
GMEWWEI(1, n) = aircraft.weight_balance.Green_Manufacturer_empty_weight;
MPAYWEI(1, n) = aircraft.weight_balance.Maximum_payload_weight;
MZFWWEI(1, n) = aircraft.weight_balance.Maximum_fuel_weight;
RAMPWEI(1, n) = aircraft.weight_balance.MRW_Maximum_ramp_weight; 
MZFWWEI(1, n) = aircraft.weight_balance.MZFW_Maximum_zero_fuel_weight; 
%-------------------------------------------------------------------------------
% Merit
MPAPWEI(1, n) = aircraft.weight_balance.Merit.Maximum_payload_per_passenger;
PMFWWEI(1, n) = aircraft.weight_balance.Merit.Payload_to_MTOW_at_MFW;
LMFWWEI(1, n) = aircraft.weight_balance.Merit.Load_factor_to_MTOW_at_MFW; 
MBOMWEI(1, n) = aircraft.weight_balance.Merit.Merit_function_OEW_over_MTOW; 
MWOSWEI(1, n) = aircraft.weight_balance.Merit.Merit_function_W_over_S_gross; 
MTTWWEI(1, n) = aircraft.weight_balance.Merit.Merit_function_thrust_to_weight; 
MLDBMFN(1, n) = aircraft.weight_balance.Merit.Merit_function_load_parameter; 
%-------------------------------------------------------------------------------
% Inertia matrix
Imat = aircraft.weight_balance.Imat;
IXXINER(1, n) = aircraft.weight_balance.IXXINER; % Moment_of_inertia_Ixx
IYYINER(1, n) = aircraft.weight_balance.IYYINER; % Moment_of_inertia_Iyy;
IZZINER(1, n) = aircraft.weight_balance.IZZINER; % Moment_of_inertia_Izz;
IXZINER(1, n) = aircraft.weight_balance.IXZINER; % Moment_of_inertia_Ixz;
IYZINER(1, n) = aircraft.weight_balance.IYZINER; % Moment_of_inertia_Iyz;
IXYINER(1, n) = aircraft.weight_balance.IXYINER; % Moment_of_inertia_Ixy;
%-------------------------------------------------------------------------------
% CGs
COECOGX(1, n) = aircraft.weight_balance.MEW_longitudinal_CoG; 
COECOGY(1, n) = aircraft.weight_balance.MEW_lateral_CoG; 
COECOGZ(1, n) = aircraft.weight_balance.MEW_vertical_CoG;
CMPCOGX(1, n) = aircraft.weight_balance.Maximum_payload_at_MTOW_longitudinal_CoG;
CMPCOGY(1, n) = aircraft.weight_balance.Maximum_payload_at_MTOW_lateral_CoG;
CMPCOGZ(1, n) = aircraft.weight_balance.Maximum_payload_at_MTOW_vertical_CoG;
% CMFCOGX(1, n) = aircraft.weight_balance.Maximum_fuel_at_MTOW_longitudinal_CoG;
% CMFCOGY(1, n) = aircraft.weight_balance.Maximum_fuel_at_MTOW_lateral_CoG;
% CMFCOGZ(1, n) = aircraft.weight_balance.Maximum_fuel_at_MTOW_vertical_CoG;
CALCOGX(1, n) = aircraft.weight_balance.Computed_longitudinal_CoG;
CALCOGY(1, n) = aircraft.weight_balance.Computed_lateral_CoG;
CALCOGZ(1, n) = aircraft.weight_balance.Computed_vertical_CoG;
%-------------------------------------------------------------------------------
% Fuel cgs
xcgwing(1, n) = aircraft.weight_balance.Fuel.Fuel_in_wing_x_cg; 
ycgwing(1, n) = aircraft.weight_balance.Fuel.Fuel_in_wing_y_cg; 
zcgwing(1, n) = aircraft.weight_balance.Fuel.Fuel_in_wing_z_cg;
xcgfair(1, n) = aircraft.weight_balance.Fuel.Fuel_in_fairings_x_cg; 
ycgfair(1, n) = aircraft.weight_balance.Fuel.Fuel_in_fairings_y_cg; 
zcgfair(1, n) = aircraft.weight_balance.Fuel.Fuel_in_fairings_z_cg; 
xcgtaux(1, n) = aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_x_cg; 
ycgtaux(1, n) = aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_y_cg; 
zcgtaux(1, n) = aircraft.weight_balance.Fuel.Fuel_in_auxiliary_tanks_z_cg;
%-------------------------------------------------------------------------------
% Weight arms
WINGBAX(1, n) = aircraft.weight_balance.Struct1_x_cg;
WINGBAY(1, n) = aircraft.weight_balance.Struct1_y_cg;
WINGBAZ(1, n) = aircraft.weight_balance.Struct1_z_cg;
HTALBAX(1, n) = aircraft.weight_balance.Struct.Horizontal_tail_x_cg;
HTALBAY(1, n) = aircraft.weight_balance.Struct.Horizontal_tail_y_cg;
HTALBAZ(1, n) = aircraft.weight_balance.Struct.Horizontal_tail_z_cg;
VTALBAX(1, n) = aircraft.weight_balance.Struct.Vertical_tail_x_cg;
VTALBAY(1, n) = aircraft.weight_balance.Struct.Vertical_tail_y_cg;
VTALBAZ(1, n) = aircraft.weight_balance.Struct.Vertical_tail_z_cg;
FUSEBAX(1, n) = aircraft.weight_balance.Struct.Fuselage_structure_x_cg;
FUSEBAY(1, n) = aircraft.weight_balance.Struct.Fuselage_structure_y_cg;
FUSEBAZ(1, n) = aircraft.weight_balance.Struct.Fuselage_structure_z_cg;
POWPBAX(1, n) = aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_x_cg;
POWPBAX(2, n) = aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_x_cg;
POWPBAY(1, n) = aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_y_cg;
POWPBAY(2, n) = aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_y_cg;
POWPBAZ(1, n) = aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_z_cg;
POWPBAZ(2, n) = aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_z_cg;
FURNBAX(1, n) = aircraft.weight_balance.System.Total_systems_or_miscellaneous_x_cg;
FURNBAY(1, n) = aircraft.weight_balance.System.Total_systems_or_miscellaneous_y_cg;
FURNBAZ(1, n) = aircraft.weight_balance.System.Total_systems_or_miscellaneous_z_cg;

FUELBAX(1, n) = xcgwing(1, n);
FUELBAY(1, n) = ycgwing(1, n);
FUELBAZ(1, n) = zcgwing(1, n);
FCENBAX(1, n) = aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_x_cg;
FCENBAY(1, n) = aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_y_cg;
FCENBAZ(1, n) = aircraft.weight_balance.Fuel.Fuel_centre_plus_confor_z_cg;
FAUXBAX(1, n) = aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_x_cg;
FAUXBAY(1, n) = aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_y_cg;
FAUXBAZ(1, n) = aircraft.weight_balance.Fuel.Fuel_tank_auxiliary_z_cg;

INTEBAX(1, n) = aircraft.weight_balance.Payload.Interiors_completion_x_cg;
INTEBAY(1, n) = aircraft.weight_balance.Payload.Interiors_completion_y_cg;
INTEBAZ(1, n) = aircraft.weight_balance.Payload.Interiors_completion_z_cg;
PLOTBAX(1, n) = aircraft.weight_balance.Crew.Pilots_x_cg;
PLOTBAY(1, n) = aircraft.weight_balance.Crew.Pilots_y_cg;
PLOTBAZ(1, n) = aircraft.weight_balance.Crew.Pilots_z_cg;
PASSBAX(1, n) = aircraft.weight_balance.Payload.Passengers_x_cg;
PASSBAY(1, n) = aircraft.weight_balance.Payload.Passengers_y_cg;
PASSBAZ(1, n) = aircraft.weight_balance.Payload.Passengers_z_cg;
BAGGBAX(1, n) = aircraft.weight_balance.Payload.Baggage_and_cargo_x_cg;
BAGGBAY(1, n) = aircraft.weight_balance.Payload.Baggage_and_cargo_y_cg;
BAGGBAZ(1, n) = aircraft.weight_balance.Payload.Baggage_and_cargo_z_cg;

% Added to allow principal landing gear positioning into x-z plane
% LR 18/09/08
LNDGBAX(1, n) = aircraft.miscellaneous.main_landing_gear_x_cg;
LNDGBAZ(1, n) = aircraft.miscellaneous.main_landing_gear_z_cg;
ALNDGBAX(1, n) = aircraft.miscellaneous.aux_landing_gear_x_cg;
ALNDGBAZ(1, n) = aircraft.miscellaneous.aux_landing_gear_z_cg;

%-------------------------------------------------------------------------------
paxcwei(1, n) = aircraft.weight_balance.Payload.Passenger_weight_coefficient; 
paxwwei(1, n) = aircraft.weight_balance.Payload.Weight_per_passenger; 
paxiwei(1, n) = aircraft.weight_balance.Payload.Weight_increment_per_passenger; 
rampinc(1, n) = aircraft.weight_balance.Ramp_increment; 
contwei(1, n) = aircraft.weight_balance.Weight_cont_allow_perc_of_MEW; 
mantwei(1, n) = aircraft.weight_balance.Manufacturer_weights_tolerance; 
mfwdwei(1, n) = aircraft.weight_balance.MFW_decrement_to_MTOW;
advnmat(1, n) = aircraft.weight_balance.Year_advan_techn_multip;
cabawei(1, n) = aircraft.weight_balance.Crew.Cabin_attendant_weight;
cabfwei(1, n) = aircraft.weight_balance.Crew.Flight_crew_weight;
%-------------------------------------------------------------------------------
% Fuselage, Cabin, Baggages
fuselgt(1, n) = aircraft.Fuselage.Total_fuselage_length;
fusevma(1, n) = aircraft.Fuselage.Aftfuse_X_sect_vertical_diameter;
fusevmi(1, n) = aircraft.Fuselage.Aftfuse_Xs_distortion_coefficient;
fusehma(1, n) = aircraft.Fuselage.Aftfuse_X_sect_horizontal_diameter;
forelgt(1, n) = aircraft.Fuselage.Nose_length;
aftslgt(1, n) = aircraft.Fuselage.Tail_length;

% Added for cabin floor width estimation
% LR 06/06/08
CROSXCF(1) = aircraft.Fuselage.a0_aft;
CROSXCF(3) = aircraft.Fuselage.a1_aft;
CROSXCF(2) = aircraft.Fuselage.b1_aft;
%-------------------------------------------------------------------------------
cabnpas(1, n) = aircraft.cabin.Passenger_accomodation;
cabnsab(1, n) = aircraft.cabin.Seats_abreast_in_fuselage; 
cabnspt(1, n) = aircraft.cabin.Seat_pitch; 
cabnlgt(1, n) = aircraft.cabin.Cabin_length_to_aft_cab;

% Added for cabin floor width estimation
% LR 16/11/08
% Infer cabin length from geometry if no value has been specified and set
% it if user input exceeds it
computed_cabnlgt = fuselgt - forelgt - aftslgt;
if ~cabnlgt(1, n)
    cabnlgt(1, n) = computed_cabnlgt;
else
    warning('Cabin length inserted %6.2f [m] doesn''t correspond to computed value of %6.2f [m]',...
        cabnlgt(1, n), computed_cabnlgt);
% elseif cabnlgt(1, n) > computed_cabnlgt
%     cabnlgt(1, n) = computed_cabnlgt;
end

cabnhei(1, n) = aircraft.cabin.Cabin_max_internal_height; 
cabnwid(1, n) = aircraft.cabin.Cabin_max_internal_width;
cabnfwd(1, n) = aircraft.cabin.Cabin_floor_width;
cabnvol(1, n) = aircraft.cabin.Cabin_volume;
baggtyp(1, n) = aircraft.Baggage.installation_type;      
baggapx(1, n) = aircraft.Baggage.Baggage_apex_per_fuselgt;
baggvol(1, n) = aircraft.Baggage.gross_volume;                    
bagglgt(1, n) = aircraft.Baggage.Baggage_combined_length;
cabncei(1, n) = aircraft.cabin.Maximum_cabin_altitude;
cabnpdf(1, n) = aircraft.cabin.Max_pressure_differential;
cabnatt(1, n) = aircraft.cabin.Cabin_attendant_number;
cabnfcr(1, n) = aircraft.cabin.Flight_crew_number;
XSECDWF(1, n) = aircraft.Fuselage.X_sect_chord_at_fuse_wing;
%-------------------------------------------------------------------------------
% Wing 1
wingare(1, n) = aircraft.Wing1.area;
wingspn(1, n) = aircraft.Wing1.Span;
wingcfg(1, n) = aircraft.Wing1.configuration;
wingplc(1, n) = aircraft.Wing1.placement;
wingapx(1, n) = aircraft.Wing1.apex_locale;
wingkln(1, n) = aircraft.Wing1.spanwise_kink1;
wingkln(2, n) = aircraft.Wing1.spanwise_kink2;
winglsw(1, n) = aircraft.Wing1.LE_sweep_inboard;
winglsw(2, n) = aircraft.Wing1.LE_sweep_midboard;
winglsw(3, n) = aircraft.Wing1.LE_sweep_outboard;
wingdih(1, n) = aircraft.Wing1.dihedral_inboard;
wingdih(2, n) = aircraft.Wing1.dihedral_midboard;
wingdih(3, n) = aircraft.Wing1.dihedral_outboard;
wingthk(1, n) = aircraft.Wing1.thickness_root;
wingthk(2, n) = aircraft.Wing1.thickness_kink1;
wingthk(3, n) = aircraft.Wing1.thickness_kink2;
wingthk(4, n) = aircraft.Wing1.thickness_tip;

try
    wletiwt(1, n) = aircraft.Wetted_areas.increment_winglet; 
catch
    wletiwt(1, n) = aircraft.Wetted_areas.Winglet; 
end

wingtap(1, n) = aircraft.Wing1.taper_kink1;
wingtap(2, n) = aircraft.Wing1.taper_kink2;
wingtap(3, n) = aircraft.Wing1.taper_tip;

%-------------------------------------------------------------------------------
wletspn(1, n) = aircraft.Wing1.winglet.Span;
WSPNMTX(1:3, n) = aircraft.Wing1.Span_matrix_partition_in_mid_outboard;
WCHDROT(1, n) = aircraft.Reference_wing.Orig_root_chrd_at_ac_CL;
REFWYBR(1, n) = aircraft.Reference_wing.non_dim_MAC_y_bar;
WINWYBR(1, n) = REFWYBR(1, n);                                              % apparently they are the same
REFWMAC(1, n) = aircraft.Reference_wing.MAC;
REFWLSW(1, n) = aircraft.Reference_wing.LE_sweep;
REFWAPX(1, n) = aircraft.Reference_wing.relative_apex;
WINWGAR(1, n)  = aircraft.Reference_wing.Weighted_aspect_ratio;
WINWHSQ(1, n)  = aircraft.Reference_wing.Half_chord_sweep;
WINWARE(1, n)  = aircraft.Reference_wing.Weighted_area;
REFWARE(1, n)  = WINWARE(1, n);                                             % apparently the same after qgeotry call
WINWTHB(1, n)  = aircraft.Reference_wing.mean_thickness;
WINWTAP(1, n)  = aircraft.Wing1.Weighted_taper_ratio;
WINWQSW(1, n)  = aircraft.Reference_wing.Quarter_chord_sweep;
wletwfc(1, n) = aircraft.Reference_wing.Wing_area_for_weight_balance_analysis;
try
    wgevorx(1, n) = aircraft.Wing1.Fractional_change_vortex_induced_drag_factor;
catch
    wgevorx(1, n) = 0.;
end

%*************************************************************************      ADDED 2008-05-23 
% Added for Mitchell code compatibility
REFWTAP(1, n) = aircraft.Reference_wing.taper_ratio;
REFWTHB = WINWTHB; 
%*************************************************************************
%*************************************************************************      ADDED 2008-05-23 
% Added for Mitchell code compatibility
VTAWARE(1, n) = aircraft.Vertical_tail.reference_wing_area;
VTAWTAP(1, n) = aircraft.Vertical_tail.reference_wing_taper_ratio;
VTAWLSW(1, n) = aircraft.Vertical_tail.reference_wing_LE_sweep;
%-------------------------------------------------------------------------------
% Wing 2
wi2gcfg(1, n) = aircraft.Wing2.configuration;
wi2gplc(1, n) = aircraft.Wing2.placement;
wi2gare(1, n) = aircraft.Wing2.area;
%-------------------------------------------------------------------------------
% if wi2gare(n) > 0.001
WI2WGAR(1, n) = aircraft.Wing2.Weighted_reference_aspect_ratio;
WI2WARE(1, n) = aircraft.Wing2.Weighted_reference_wing_area;
WI2WTAP(1, n) = aircraft.Wing2.Weighted_taper_ratio;
WI2WQSW(1, n) = aircraft.Wing2.Reference_quarter_chord_sweep;
WI2WTHB(1, n) = aircraft.Wing2.Wing_mean_thickness;
% end
%-------------------------------------------------------------------------------
% Horizontal Tail
taillay(1, n) = aircraft.Horizontal_tail.empennage_layout;
htalare(1, n) = aircraft.Horizontal_tail.area;
htalgar(1, n) = aircraft.Horizontal_tail.AR;
htallsw(1, n) = aircraft.Horizontal_tail.LE_sweep_inboard;
htallsw(3, n) = aircraft.Horizontal_tail.LE_sweep_outboard;
htaldih(1, n) = aircraft.Horizontal_tail.dihedral_inboard;
htaldih(2, n) = aircraft.Horizontal_tail.dihedral_outboard;
htalver(1, n) = aircraft.Horizontal_tail.vertical_locale;
htalapx(1, n) = aircraft.Horizontal_tail.apex_locale;
htalspn(1, n) = aircraft.Horizontal_tail.Span;
htalkln(1, n) = aircraft.Horizontal_tail.spanwise_kink;

% LR 18/03/2009
% Not needed anymore since control surface definition is changed
% htalkln(2, n) = aircraft.Horizontal_tail.Elevator.Span;

htaltap(1, n) = aircraft.Horizontal_tail.taper_kink;
htaltap(3, n) = aircraft.Horizontal_tail.taper_tip;
%-------------------------------------------------------------------------------
% AP 2009.04.20 - LR 16/02/2010 modified check
ht_flag(1, n) = aircraft.Horizontal_tail.present;

if ht_flag
    
    HTAPEXX = aircraft.Horizontal_tail.longitudinal_location;
    HTAPEXZ = aircraft.Horizontal_tail.vertical_location;
    HTALYBR(1, n) = aircraft.Horizontal_tail.reference_wing_Y_bar_non_dim;
    HTALTHB(1, n) = aircraft.Horizontal_tail.reference_wing_mean.thickness;
    HTAWQSW(1, n) = aircraft.Horizontal_tail.reference_wing_quarter_chord_sweep;
    HTALMOA(1, n) = aircraft.Horizontal_tail.Moment_arm_to_HT;
    HSPNMTX(1:3, n) = aircraft.Horizontal_tail.Span_matrix_partition_in_mid_outboard;
    HCHDROT(1, n) = aircraft.Horizontal_tail.original_root_chord;
    % Added for Mitchell code compatibility
    HTAWARE(1, n) = aircraft.Horizontal_tail.reference_wing_area;
    HTAWTAP(1, n) = aircraft.Horizontal_tail.reference_wing_taper_ratio;
    HTAWLSW(1, n) = aircraft.Horizontal_tail.reference_wing_LE_chord_sweep;
    
else
    
    HTALYBR(1, n) = 0;
    HTALTHB(1, n) = 0;
    HTAWQSW(1, n) = 0;
    HTALMOA(1, n) = 0;
    HSPNMTX(1:3, n) = 0;
    HCHDROT(1, n) = 0;
    % Added for Mitchell code compatibility
    HTAWARE(1, n) = 0;
    HTAWTAP(1, n) = 0;
    HTAWLSW(1, n) = 0;
    HTAPEXX = 0.;
    HTAPEXZ = 0.;
    
end
%-------------------------------------------------------------------------------
% Vertical Tail
vtalare(1, n) = aircraft.Vertical_tail.area;
vtalgar(1, n) = aircraft.Vertical_tail.AR;
vtalver(1, n) = aircraft.Vertical_tail.vertical_locale;
vtalapx(1, n) = aircraft.Vertical_tail.apex_locale;
vtalspn(1, n) = aircraft.Vertical_tail.Span;
vtaltap(1, n) = aircraft.Vertical_tail.taper_kink;
vtaltap(3, n) = aircraft.Vertical_tail.taper_tip;
vtallsw(1, n) = aircraft.Vertical_tail.LE_sweep_inboard;
vtallsw(3, n) = aircraft.Vertical_tail.LE_sweep_outboard;
vtalkln(1, n) = aircraft.Vertical_tail.spanwise_kink;

% LR 18/03/2009
% Not needed anymore since control surface definition is changed
% vtalkln(2, n) = aircraft.Vertical_tail.Rudder.Span;

try
    vtaldlc(1, n) = aircraft.Vertical_tail.Dorsal_location;
catch
    vtaldlc(1, n) = 0.;
end

% LR 11/05/2009

% Added twin vertical tail modelling capability
try aircraft.Vertical_tail.Twin_tail;
    Twin_tail = aircraft.Vertical_tail.Twin_tail;      % Flag 0/1
    Twin_tail_span = aircraft.Vertical_tail.Twin_tail_span; % Offset in % wing span

catch
    Twin_tail = 0;
    Twin_tail_span = 0;
end

%--------------------------------------------------------------------------
VTALYBR(1, n) = aircraft.Vertical_tail.reference_Y_bar_non_dim;
VTALTHB(1, n) = aircraft.Vertical_tail.reference_wing_mean_thickness;
VTAWQSW(1, n) = aircraft.Vertical_tail.reference_wing_quarter_chord_sweep;
VTALWET(1, n) = aircraft.Wetted_areas.Vertical_tail;
VTALMOA(1, n) = aircraft.Vertical_tail.Moment_arm_to_VT;
VSPNMTX(1:3, n) = aircraft.Vertical_tail.Span_matrix_partition_in_mid_outboard;
dorswet(1, n) = aircraft.Wetted_areas.Dorsal_fin;
VCHDROT(1, n) = aircraft.Vertical_tail.original_root_chord;
%--------------------------------------------------------------------------
% LR 16/02/2010 - Added Canard
canard_flag(1, n) = aircraft.Canard.present;

if canard_flag
    
    candare(1, n)   = aircraft.Canard.area;
    candgar(1, n)   = aircraft.Canard.AR;
    CANDTHB(1, n)   = aircraft.Canard.reference_wing_mean.thickness;
    CANDWQSW(1, n)  = aircraft.Canard.reference_wing_quarter_chord_sweep;
    candver(1, n)   = aircraft.Canard.vertical_locale;
    candapx(1, n)   = aircraft.Canard.apex_locale;
    CANDYBR(1, n)   = aircraft.Canard.reference_wing_Y_bar_non_dim;
    CANDBAX(1, n)   = aircraft.weight_balance.Struct.Canard_x_cg;
    CANDBAY(1, n)   = aircraft.weight_balance.Struct.Canard_y_cg;
    CANDBAZ(1, n)   = aircraft.weight_balance.Struct.Canard_z_cg;
    candspn(1, n)   = aircraft.Canard.Span;
    candtap(1, n)   = aircraft.Canard.taper_kink;
    candtap(3, n)   = aircraft.Canard.taper_tip;
    candlsw(1, n)   = aircraft.Canard.LE_sweep_inboard;
    candlsw(3, n)   = aircraft.Canard.LE_sweep_outboard;
    candkln(1, n)   = aircraft.Canard.spanwise_kink;
    canddih(1, n)   = aircraft.Canard.dihedral_inboard;
    canddih(2, n)   = aircraft.Canard.dihedral_outboard;
    CSPNMTX(1:3, :) = aircraft.Canard.Span_matrix_partition_in_mid_outboard;
    CCHDROT(1, n)   = aircraft.Canard.original_root_chord;
    
else
 
    CANDBAX(1, n)   = 0.;
    CANDBAY(1, n)   = 0.;
    CANDBAZ(1, n)   = 0.;
    CANDTHB(1, n)   = 0.;
    CANDWQSW(1, n)  = 0;
    CANDYBR(1, n)   = 0.;
    CSPNMTX(1:3, :) = 0.;
    CCHDROT(1, n)   = 0.;
    CANDBAX(1, n)   = 0.;
    CANDBAY(1, n)   = 0.;
    CANDBAZ(1, n)   = 0.;
    
end

%--------------------------------------------------------------------------
% SR 31/05/2010 - Added Tailbooms

tailbooms_flag(1, n) = aircraft.Tailbooms.present;

if tailbooms_flag

    tlbmsys(1, n) = aircraft.Tailbooms.symmetry;
    tlbmdim(1, n) = aircraft.Tailbooms.diameter;
    tlbmlen(1, n) = aircraft.Tailbooms.total_length;
    tlbmlox(1, n) = aircraft.Tailbooms.x_location;
    tlbmloy(1, n) = aircraft.Tailbooms.y_location;
    tlbmloz(1, n) = aircraft.Tailbooms.z_location;

end

%--------------------------------------------------------------------------

% Propulsion
engenum(1, n) = aircraft.Engines1.Number_of_engines;
engeloc(1, n) = aircraft.Engines1.Layout_and_config;
engenum(2, n) = aircraft.Engines2.Number_of_engines;
engeloc(2, n) = aircraft.Engines2.Layout_and_config;
propdia(1, n) = aircraft.Engines1.Propeller_diameter;
propdia(2, n) = aircraft.Engines2.Propeller_diameter;
nacetyp(1, n) = aircraft.Engines1.Nacelle_body_type;
nacetyp(2, n) = aircraft.Engines2.Nacelle_body_type;
nacefin(1, n) = aircraft.Engines1.fineness_ratio;
nacefin(2, n) = aircraft.Engines2.fineness_ratio;
nacemdi(1, n) = aircraft.Engines1.d_max;
nacemdi(2, n) = aircraft.Engines2.d_max;
maxistc(1, n) = aircraft.Engines1.Max_thrust;
maxistc(2, n) = aircraft.Engines2.Max_thrust;
engeylc(1, n) = aircraft.Engines1.Y_locale;
engeylc(2, n) = aircraft.Engines2.Y_locale;
insttyp(1, n) = aircraft.Engines1.Propulsion_type;
insttyp(2, n) = aircraft.Engines2.Propulsion_type;
%-------------------------------------------------------------------------------
thrtrev(1, n) = aircraft.Engines1.Thrust_reverser_effectivness;
thrtrev(2, n) = aircraft.Engines2.Thrust_reverser_effectivness;
fixdthw(1, n) = aircraft.Engines1.Thrust_to_weight_ratio;
bypasem(1, n) = aircraft.Engines1.Bypass_ratio_to_emulate;
bypasem(2, n) = aircraft.Engines2.Bypass_ratio_to_emulate;

% 17/03/2009 L.Riccobene - AcBuilder has a different format for Nacelles,
% added a few lines to mantain backward compatibility
% 19/02/2010 L.Riccobene - Updated format since AcBuilder is now a standard
if aircraft.Engines1.present
    
    NACENOX(1, n) = aircraft.Engines1.Nacelle1.longitudinal_location;
    NACENOY(1, n) = aircraft.Engines1.Nacelle1.lateral_location;
    NACENOZ(1, n) = aircraft.Engines1.Nacelle1.vertical_location;
    NACELGT(1, n) = aircraft.Engines1.Nacelle_length_array; 
    
else
    
    NACENOX(1, n) = 0.;
    NACENOY(1, n) = 0.;
    NACENOZ(1, n) = 0.;
    NACELGT(1, n) = 0.;
    
end

if aircraft.Engines2.present
    
    NACENOX(2, n) = aircraft.Engines2.Nacelle3.longitudinal_location;
    NACENOY(2, n) = aircraft.Engines2.Nacelle3.lateral_location;
    NACENOZ(2, n) = aircraft.Engines2.Nacelle3.vertical_location;
    NACELGT(2, n) = aircraft.Engines2.Nacelle_length_array;
    
else
    
    NACENOX(2, n) = 0.;
    NACENOY(2, n) = 0.;
    NACENOZ(2, n) = 0.;
    NACELGT(2, n) = 0.;
    
end

%-------------------------------------------------------------------------------
% Fuel
wingspf(1, n) = aircraft.fuel.Fore_wing_spar_loc_root;
wingspf(2, n) = aircraft.fuel.Fore_wing_spar_loc_kik1;
wingspf(3, n) = aircraft.fuel.Fore_wing_spar_loc_kin2;
wingspf(4, n) = aircraft.fuel.Fore_wing_spar_loc_tip;
wingspa(1, n) = aircraft.fuel.Aft_wing_spar_loc_root;
wingspa(2, n) = aircraft.fuel.Aft_wing_spar_loc_kin1;
wingspa(3, n) = aircraft.fuel.Aft_wing_spar_loc_kin2;
wingspa(4, n) = aircraft.fuel.Aft_wing_spar_loc_tip;
fuelusu(1, n) = aircraft.fuel.Unusable_fuel_option;
%-------------------------------------------------------------------------------
% Flight envelope
FENVVDF(1, n) = aircraft.weight_balance.flight_envelope_prediction.VD_Flight_envelope_dive;
FENVVMO(1, n) = aircraft.weight_balance.flight_envelope_prediction.VMO_Flight_envelope;

%-------------------------------------------------------------------------------
% Input for Stability and Control Computations
% To be added to Adrien's file
STACAUW(1, n) = aircraft.stability.All_up_weight;

%--------------------------------------------------------------------------
% Run WB subroutines
%--------------------------------------------------------------------------
% First
rcogs
%--------------------------------------------------------------------------
% Second
wb_weight
%--------------------------------------------------------------------------
% Third
rweig
%--------------------------------------------------------------------------
% Fourth
riner

%--------------------------------------------------------------------------
% Set W&B version
% aircraft.weight_balance.version = wb_version;

%--------------------------------------------------------------------------
% Export all variables and overwrite / copy their values
% Weight and balance output struct

% WB Fuel
aircraft.weight_balance.Fuel.Fuel_in_wing_x_cg = FUELBAX(1, n);
aircraft.weight_balance.Fuel.Fuel_in_wing_y_cg = FUELBAY(1, n);
aircraft.weight_balance.Fuel.Fuel_in_wing_z_cg = FUELBAZ(1, n);

% WB generic
aircraft.weight_balance.MTOW_Maximum_takeoff_weight = MTOWWEI(1, n);
aircraft.weight_balance.OEW_Operational_empty_weight = DOEWWEI(1, n);
aircraft.weight_balance.Green_Manufacturer_empty_weight = GMEWWEI(1, n);
aircraft.weight_balance.Maximum_payload_weight = MPAYWEI(1, n);
aircraft.weight_balance.Fuel.Maximum_fuel_weight = DMFWWEI(1, n);
aircraft.weight_balance.Maximum_fuel_weight = DMFWWEI(1, n);
aircraft.weight_balance.MRW_Maximum_ramp_weight = RAMPWEI(1, n);
aircraft.weight_balance.MZFW_Maximum_zero_fuel_weight = MZFWWEI(1, n);
aircraft.weight_balance.MEW_longitudinal_CoG = COECOGX(1, n);
aircraft.weight_balance.MEW_lateral_CoG = COECOGY(1, n);
aircraft.weight_balance.MEW_vertical_CoG = COECOGZ(1, n);
aircraft.weight_balance.Maximum_payload_at_MTOW_longitudinal_CoG = CMPCOGX(1, n);
aircraft.weight_balance.Maximum_payload_at_MTOW_lateral_CoG = CMPCOGY(1, n);
aircraft.weight_balance.Maximum_payload_at_MTOW_vertical_CoG = CMPCOGZ(1, n);
aircraft.weight_balance.Computed_longitudinal_CoG = CALCOGX(1, n);
aircraft.weight_balance.Computed_lateral_CoG = CALCOGY(1, n);
aircraft.weight_balance.Computed_vertical_CoG = CALCOGZ(1, n);
aircraft.weight_balance.Ramp_increment = rampinc(1, n);
aircraft.weight_balance.Weight_cont_allow_perc_of_MEW = contwei(1, n);
aircraft.weight_balance.Manufacturer_weights_tolerance = mantwei(1, n);
aircraft.weight_balance.MFW_decrement_to_MTOW = mfwdwei(1, n);
aircraft.weight_balance.Year_advan_techn_multip = advnmat(1, n);
aircraft.weight_balance.Crew.Cabin_attendant_weight = cabawei(1, n);
aircraft.weight_balance.Crew.Flight_crew_weight = cabfwei(1, n);
aircraft.weight_balance.COG = PLOTCGS;

% WB Merit
aircraft.weight_balance.Merit.Maximum_payload_per_passenger = MPAPWEI(1, n);
aircraft.weight_balance.Merit.Payload_to_MTOW_at_MFW = PMFWWEI(1, n);
aircraft.weight_balance.Merit.Load_factor_to_MTOW_at_MFW = LMFWWEI(1, n);
aircraft.weight_balance.Merit.Merit_function_OEW_over_MTOW = MBOMWEI(1, n);
aircraft.weight_balance.Merit.Merit_function_W_over_S_gross = MWOSWEI(1, n);
aircraft.weight_balance.Merit.Merit_function_thrust_to_weight = MTTWWEI(1, n);
aircraft.weight_balance.Merit.Merit_function_load_parameter = MLDBMFN(1, n);

% WB inertia matrix
aircraft.weight_balance.IXXINER = IXXINER(1, n);
aircraft.weight_balance.IYYINER = IYYINER(1, n);
aircraft.weight_balance.IZZINER = IZZINER(1, n);
aircraft.weight_balance.IXZINER = IXZINER(1, n);
aircraft.weight_balance.IYZINER = IYZINER(1, n);
aircraft.weight_balance.IXYINER = IXYINER(1, n);
Imat = [IXXINER(1, n), IXYINER(1, n), IXZINER(1, n);...
    IXYINER(1, n), IYYINER(1, n), IYZINER(1, n);...
    IXZINER(1, n), IYZINER(1, n), IZZINER(1, n)];
aircraft.weight_balance.Imat = Imat;

% WB structure
aircraft.weight_balance.Struct1_x_cg = WINGBAX(1, n);
aircraft.weight_balance.Struct1_y_cg = WINGBAY(1, n);
aircraft.weight_balance.Struct1_z_cg = WINGBAZ(1, n);
aircraft.weight_balance.Struct.Horizontal_tail_x_cg = HTALBAX(1, n);
aircraft.weight_balance.Struct.Horizontal_tail_y_cg = HTALBAY(1, n);
aircraft.weight_balance.Struct.Horizontal_tail_z_cg = HTALBAZ(1, n);
aircraft.weight_balance.Struct.Vertical_tail_x_cg = VTALBAX(1, n);
aircraft.weight_balance.Struct.Vertical_tail_y_cg = VTALBAY(1, n);
aircraft.weight_balance.Struct.Vertical_tail_z_cg = VTALBAZ(1, n);
aircraft.weight_balance.Struct.Fuselage_structure_x_cg = FUSEBAX(1, n);
aircraft.weight_balance.Struct.Fuselage_structure_y_cg = FUSEBAY(1, n);
aircraft.weight_balance.Struct.Fuselage_structure_z_cg = FUSEBAZ(1, n);
aircraft.weight_balance.Struct.Wings = WINGWEI(1, n);
aircraft.weight_balance.Struct.Winglet_and_span_load_penalty = WLETWEI(1, n);
aircraft.weight_balance.Struct.Horizontal_tail_and_elevator = HTALWEI(1, n);
aircraft.weight_balance.Struct.Vertical_tail_rudder_and_dorsal = VTALWEI(1, n);
aircraft.weight_balance.Struct.Ventral_fins = VENTWEI(1, n);
aircraft.weight_balance.Struct.Fuselage = FUSEWEI(1, n);
% LR 16/02/2010 - added Canard
aircraft.weight_balance.Struct.Canard_x_cg = CANDBAX(1, n);
aircraft.weight_balance.Struct.Canard_y_cg = CANDBAY(1, n);
aircraft.weight_balance.Struct.Canard_z_cg = CANDBAZ(1, n);
% SR 02/06/2010 - added Tailbooms
aircraft.weight_balance.Struct.Tailbooms_x_cg = TLBMBAX(1, n);
aircraft.weight_balance.Struct.Tailbooms_y_cg = TLBMBAY(1, n);
aircraft.weight_balance.Struct.Tailbooms_z_cg = TLBMBAZ(1, n);

    
% WB powerplant
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_x_cg = POWPBAX(1, n);
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_x_cg = POWPBAX(2, n);
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_y_cg = POWPBAY(1, n);
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_y_cg = POWPBAY(2, n);
aircraft.weight_balance.Powerplant1_plus_nacelle_plus_pylon_z_cg = POWPBAZ(1, n);
aircraft.weight_balance.Powerplant2_plus_nacelle_plus_pylon_z_cg = POWPBAZ(2, n);
aircraft.weight_balance.Powerplant.Pylons_and_or_propellers = PYLNWEI(1, n);
aircraft.weight_balance.Powerplant.Engines_acc_propeller_gearbox = POWPWEI(1, n);
aircraft.weight_balance.Powerplant.Nacelles = NACEWEI(1, n);

% WB systems
aircraft.weight_balance.System.Compl_allowance_plus_paint = COPAWEI(1, n);
aircraft.weight_balance.System.Other_operating_items = OPSIWEI(1, n);
aircraft.weight_balance.System.Landing_gear = LNDGWEI(1, n);
aircraft.weight_balance.System.Aux_Landing_gear = ALNDGWEI(1, n);
aircraft.weight_balance.System.Fuel_system_weight = FSYSWEI(1, n);
aircraft.weight_balance.System.Flight_controls_weight = FCTLWEI(1, n);
aircraft.weight_balance.System.APU_weight = APUSWEI(1, n);
aircraft.weight_balance.System.Instruments_weight = INSTWEI(1, n);
aircraft.weight_balance.System.Avionics_weight = AVIOWEI(1, n);
aircraft.weight_balance.System.Hydraulic_Pneumatic_weight = HYPNWEI(1, n);
aircraft.weight_balance.System.Electrical_weight = ELECWEI(1, n);
aircraft.weight_balance.System.ECS_anti_icing_grp_weight = ECSAWEI(1, n);
aircraft.weight_balance.System.Furnishings_Green = FURNWEI(1, n);
aircraft.weight_balance.System.Miscellaneous = MISCWEI(1, n);
aircraft.weight_balance.System.Total_systems_or_miscellaneous_x_cg = FURNBAX(1, n);
aircraft.weight_balance.System.Total_systems_or_miscellaneous_y_cg = FURNBAY(1, n);
aircraft.weight_balance.System.Total_systems_or_miscellaneous_z_cg = FURNBAZ(1, n);

% WB payload
aircraft.weight_balance.Payload.Interiors_completion_x_cg = INTEBAX(1, n);
aircraft.weight_balance.Payload.Interiors_completion_y_cg = INTEBAY(1, n);
aircraft.weight_balance.Payload.Interiors_completion_z_cg = INTEBAZ(1, n);
aircraft.weight_balance.Payload.Passengers_x_cg = PASSBAX(1, n);
aircraft.weight_balance.Payload.Passengers_y_cg = PASSBAY(1, n);
aircraft.weight_balance.Payload.Passengers_z_cg = PASSBAZ(1, n);
aircraft.weight_balance.Payload.Baggage_and_cargo_x_cg = BAGGBAX(1, n);
aircraft.weight_balance.Payload.Baggage_and_cargo_y_cg = BAGGBAY(1, n);
aircraft.weight_balance.Payload.Baggage_and_cargo_z_cg = BAGGBAZ(1, n);
aircraft.weight_balance.Payload.Passenger_weight_coefficient = paxcwei(1, n);
aircraft.weight_balance.Payload.Weight_per_passenger = paxwwei(1, n);
aircraft.weight_balance.Payload.Weight_increment_per_passenger = paxiwei(1, n);

% WB crew
aircraft.weight_balance.Crew.Pilots_x_cg = PLOTBAX(1, n);
aircraft.weight_balance.Crew.Pilots_y_cg = PLOTBAY(1, n);
aircraft.weight_balance.Crew.Pilots_z_cg = PLOTBAZ(1, n);
aircraft.weight_balance.Crew.Crew_and_carry_on = CREWWEI(1, n);
%--------------------------------------------------------------------------
%
% Values used/modified by WB and saved in other 'aircraft' struct fields

% Miscellaneous
aircraft.miscellaneous.Target_operating_ceiling = opercei(1, n);
aircraft.miscellaneous.Design_classification = destype(1, n);
aircraft.miscellaneous.Spoiler_effectivity = spoiler(1, n);
aircraft.miscellaneous.Undercarriage_layout = undrloc(1, n);

% Added to allow principal landing gear positioning into x-z plane
% SR 11/01/12
aircraft.miscellaneous.main_landing_gear_x_cg = LNDGBAX(n);
aircraft.miscellaneous.main_landing_gear_z_cg = LNDGBAZ(n);
aircraft.miscellaneous.aux_landing_gear_x_cg = ALNDGBAX(n);
aircraft.miscellaneous.aux_landing_gear_z_cg = ALNDGBAZ(n);

% Cabin
aircraft.cabin.Passenger_accomodation = cabnpas(1, n);
aircraft.cabin.Seats_abreast_in_fuselage = cabnsab(1, n);
aircraft.cabin.Seat_pitch = cabnspt(1, n);
aircraft.cabin.Cabin_length_to_aft_cab = cabnlgt(1, n);
aircraft.cabin.Cabin_max_internal_height = cabnhei(1, n);
aircraft.cabin.Cabin_max_internal_width = cabnwid(1, n);
aircraft.cabin.Cabin_floor_width = cabnfwd(1, n);
aircraft.cabin.Cabin_volume = cabnvol(1, n);
aircraft.cabin.Maximum_cabin_altitude = cabncei(1, n);
aircraft.cabin.Max_pressure_differential = cabnpdf(1, n);
aircraft.cabin.Cabin_attendant_number = cabnatt(1, n);
aircraft.cabin.Flight_crew_number = cabnfcr(1, n);

% Baggage
aircraft.Baggage.Baggage_apex_per_fuselgt = baggapx(1, n);
aircraft.Baggage.Baggage_combined_length = bagglgt(1, n);
aircraft.Baggage.gross_volume = baggvol(1, n);

% Additional parameters (needed by W&B)
aircraft.weight_balance.flight_envelope_prediction.VD_Flight_envelope_dive = FENVVDF(1, n);
aircraft.weight_balance.flight_envelope_prediction.VMO_Flight_envelope = FENVVMO(1, n);
aircraft.stability.All_up_weight = STACAUW(1, n);
%

% Return updated aircraft struct
out = aircraft;

% end weight_xml


%--------------------------------------------------------------------------
% AUXILIARY FUNCTIONS
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Computes the local chord length at given span station
%          compute chord on actual planform geometry
%
function chord = qxcdcop(spanin, spanmtrx, rcrd, tapmtrx, z)

	if spanin == spanmtrx(1) || spanin == spanmtrx(1)+spanmtrx(2)
   		correct = -1;% correction to heavyside result
	else
   		correct = 0;% no correction to heavyside required
    end
    
	% identify the taper ratio for given wing segment
	taprs = tapmtrx(correct + 1 + qxheavy(spanin, spanmtrx(1), 1) + qxheavy(spanin, spanmtrx(1)+spanmtrx(2), 1), z);
	% identify the local span length for given wing segment           
	spanc = spanmtrx(correct + 1 + qxheavy(spanin, spanmtrx(1), 1) + qxheavy(spanin, spanmtrx(1)+spanmtrx(2), 1));
	% additional corrections before interpolation is executed 
    
	if spanin > spanmtrx(1) && spanin < spanmtrx(1)+spanmtrx(2)

   		spanin = spanin-spanmtrx(1);% correct input span to local datum
   		taprs = taprs/tapmtrx(1,z);% correct identified taper to local datum
   		rcrd = rcrd*tapmtrx(1,z);% correct for local chord datum   

	elseif spanin >= spanmtrx(1)+spanmtrx(2)   

   		spanin = spanin-spanmtrx(1)-spanmtrx(2);% correct input span to local datum
   		taprs = taprs/tapmtrx(2,z);% correct identified taper to local datum
   		rcrd = rcrd*tapmtrx(2,z);% correct for local chord datum   

	end
	
	chord = rcrd*(1-(1-taprs)*spanin/spanc);% computed chord result

return

function [gengewei, gnacewei, gpylnwei, gpropwei] = ewcomp(gmaxistc, gthrtrev, gengeloc, ginsttyp, id, z)

	kthrr = 1+0.18*gthrtrev(id,z)/100;% factor to increase weight due to thrust rev.
	gpropwei = 6.13*qxheavy(ginsttyp(id,z),1,1)*gmaxistc(id,z);% propeller 
	gengewei = 0.0117*(1+0.2*qxheavy(ginsttyp(id,z),1,1))*(gmaxistc(id, z)*1000)^1.0572;% dry engine weight
	gnacewei = 0.345*(1+qxheavy(gengeloc(id,z),4,1))*(1-0.53*qxheavy(ginsttyp(id, z),1,1))*kthrr*gengewei;% nacelle weight
	gpylnwei = 0.574*(1-qxheavy(gengeloc(id,z),4,1))*(1-qxheavy(ginsttyp(id, z),1,1))*gengewei^0.736;% pylon weight

return
%--------------------------------------------------------------------------
% Conversion from CAS to TAS for given ISA deviation and altitude
%
function ckspd = qxdctks(spd, alt, disa)

	ckspd = 1479.1 * (qxdthet(alt,disa) * ((((1/(qxdsigm(alt,disa)*qxdthet(alt,disa))*(((1+...
        ((spd/661.4786)^2)*0.2)^3.5)-1)+1)^(1/3.5))-1)))^0.5;

return
%--------------------------------------------------------------------------
% Conversion from TAS to Mach Number for given ISA deviation and altitude
%
function kmspd = qxdktms(spd, alt, disa)

	kmspd = spd * 0.5144/(340.3*qxdthet(alt,disa)^0.5);

return
%--------------------------------------------------------------------------
% Computes the density lapse ratio for given ISA deviation and flight level
%
function sigma = qxdsigm(alt, disa)
	
	if alt < 0.0001
   		alt = 0.0001; % change zero to something small
	end
	% density lapse ratio at ISA
	isigm = (qxdthet(alt,0)^4.2561)+qxheavy(alt,361,0)*(2.583-0.4398*(log(alt)));
	sigma = qxdthet(alt,0)*isigm/qxdthet(alt,disa);% density lapse ratio at dISA

return
%--------------------------------------------------------------------------
% Computes the temperature lapse ratio for given ISA deviation and flight level
%
function temper = qxdthet(alt, disa)
	
	if alt<0.0001

	   alt = 0.0001;% change zero to something small

	end

	temper = 1+(1454*qxheavy(alt,361,0)*(0.00069*alt-0.248)+5.046*disa-alt)/1454;

return
%--------------------------------------------------------------------------
% A heavyside step function to switch "on" or "off"
%
function comp1 = qxheavy(input, limit, type)
	
	if type < 1
		comp1 = 0.5 + 0.5*tanh(110*(input - limit));
	else
		comp1 = round(0.5+0.5*tanh(110*(input- limit)));
	end
	
return
