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

clear all
%fid = fopen('ms0313.dat')
%[A,count] = fread(fid,inf,'double');
file2read=input('File you want to rewrite: ','s' );

aircraft=neocass_xmlwrapper(strcat(file2read,'.xml'));

fusfvma(1,1)=aircraft.fuselage.Forefuse_X_sect_vertical_diameter;
fusfvmi(1,1)=aircraft.fuselage.Forefuse_Xs_distortion_coefficient;
fusfhma(1,1)=aircraft.fuselage.Forefuse_X_sect_horizontal_diameter;
foreslp(1,1)=aircraft.fuselage.omega_nose;
foredsw(1,1)=aircraft.fuselage.phi_nose;
forefin(1,1)=aircraft.fuselage.epsilon_nose;

ffsalgn(1,1)=aircraft.fuselage.shift_fore;
midfrat(1,1)=aircraft.fuselage.fraction_fore;
fuselgt(1,1)=aircraft.fuselage.Total_fuselage_length;
fusevma(1,1)=aircraft.fuselage.Aftfuse_X_sect_vertical_diameter;
fusevmi(1,1)=aircraft.fuselage.Aftfuse_Xs_distortion_coefficient;
fusehma(1,1)=aircraft.fuselage.Aftfuse_X_sect_horizontal_diameter;
btallow(1,1)=aircraft.fuselage.omega_tail;
aftfusw(1,1)=aircraft.fuselage.phi_tail;
aftsfin(1,1)=aircraft.fuselage.espilon_tail;

%% Sponson inputs
spsnxlc(1,1)=aircraft.sponson.Frg_xlcl;
spsnzlc(1,1)=aircraft.sponson.zlcl;
spsnlgt(1,1)=aircraft.sponson.length;
spsnxzs(1,1)=aircraft.sponson.XZ_slenderness;
spsnwid(1,1)=aircraft.sponson.width;
fuseiwt(1,1)=aircraft.fuselage.Wetted_increment_fuselage_fairing;
%% Wing 1 inputs
wingcfg(1,1)=aircraft.wing1.configuration;
wingplc(1,1)=aircraft.wing1.placement;
% wingprl(1,1)=aircraft.wing1.aerofoil_technology;
wingapx(1,1)=aircraft.wing1.apex_locale;
wingare(1,1)=aircraft.wing1.area;
winggar(1,1)=aircraft.wing1.AR;
wingspn(1,1)=aircraft.wing1.span;
wingkln(1,1)=aircraft.wing1.spanwise_kink1;
wingkln(2,1)=aircraft.wing1.spanwise_kink2;
wingtap(1,1)=aircraft.wing1.taper_kink1;
wingtap(2,1)=aircraft.wing1.taper_kink2;
wingtap(3,1)=aircraft.wing1.taper_tip;
winginc(1,1)=aircraft.wing1.root_incidence;
winginc(2,1)=aircraft.wing1.kink1_incidence;
winginc(3,1)=aircraft.wing1.kink2_incidence;
winginc(4,1)=aircraft.wing1.tip_incidence;
wingqsw(1,1)=aircraft.wing1.quarter_chord_sweep_inboard;
wingqsw(2,1)=aircraft.wing1.quarter_chord_sweep_midboard;
wingqsw(3,1)=aircraft.wing1.quarter_chord_sweep_outboard;
winglsw(1,1)=aircraft.wing1.LE_sweep_inboard;
winglsw(2,1)=aircraft.wing1.LE_sweep_midboard;
winglsw(3,1)=aircraft.wing1.LE_sweep_outboard;
wingdih(1,1)=aircraft.wing1.dihedral_inboard;
wingdih(2,1)=aircraft.wing1.dihedral_midboard;
wingdih(3,1)=aircraft.wing1.dihedral_outboard;
wletspn(1,1)=aircraft.wing1.winglet.span;
wlettap(1,1)=aircraft.wing1.winglet.taper_ratio;
wletlsw(1,1)=aircraft.wing1.winglet.LE_sweep;
wletvos(1,1)=aircraft.wing1.winglet.Cant_angle;
wletinc(1,1)=aircraft.wing1.winglet.root_incidence;
wletinc(3,1)=aircraft.wing1.winglet.tip_incidence;
flapCrd(1,1)=aircraft.wing1.flap.root_chord;
flapCrd(2,1)=aircraft.wing1.flap.kink1_chord;
flapCrd(3,1)=aircraft.wing1.flap.kink2_chord;
ailrPos(1,1)=aircraft.wing1.aileron.position;
ailrChr(1,1)=aircraft.wing1.aileron.chord;
ailrSpa(1,1)=aircraft.wing1.aileron.span;
slatCrd(1,1)=aircraft.wing1.slat.chord;
slatSpa(1,1)=aircraft.wing1.slat.root_position;
slatSpa(2,1)=aircraft.wing1.slat.tip_position;
wingthk(1,1)=aircraft.wing1.thickness_root;
wingthk(2,1)=aircraft.wing1.thickness_kink1;
wingthk(3,1)=aircraft.wing1.thickness_kink2;
wingthk(4,1)=aircraft.wing1.thickness_tip;
% STACLAU(1,1)=aircraft.wing1.aileron.limit_deflection_up;
% STACLAD(1,1)=aircraft.wing1.aileron.limit_deflection_down;
%% Fairing 1 input
fairfwd(1,1)=aircraft.fairing1.Forward_chord_fraction;
fairaft(1,1)=aircraft.fairing1.Aft_chord_fraction;
fairovh(1,1)=aircraft.fairing1.flushness;
WINREFC(1,1)=aircraft.Reference_wing.convention;
wingiwt(1,1)=aircraft.wing_results.Wetted_area_increment_wings;
wletiwt(1,1)=aircraft.wing_results.Wetted_area_increment_winglet;
%% Wing 2 inputs
wi2gcfg(1,1)=aircraft.wing2.configuration;
wi2gplc(1,1)=aircraft.wing2.placement;
wi2gapx(1,1)=aircraft.wing2.Apex_locale;
% wi2gprl(1,1)=aircraft.wing1.aerofoil_technology;
wi2gare(1,1)=aircraft.wing2.area;
wi2ggar(1,1)=aircraft.wing2.AR;
wi2gkln(1,1)=aircraft.wing2.spanwise_kink1;
wi2gkln(2,1)=aircraft.wing2.spanwise_kink2;
wi2gtap(1,1)=aircraft.wing2.taper_kink1;
wi2gtap(2,1)=aircraft.wing2.taper_kink2;
wi2gtap(3,1)=aircraft.wing2.taper_tip;
wi2ginc(1,1)=aircraft.wing2.root_incidence;
wi2ginc(2,1)=aircraft.wing2.kink1_incidence;
wi2ginc(3,1)=aircraft.wing2.kink2_incidence;
wi2ginc(4,1)=aircraft.wing2.tip_incidence;
wi2gqsw(1,1)=aircraft.wing2.quarter_chord_sweep_inboard;
wi2gqsw(2,1)=aircraft.wing2.quarter_chord_sweep_midboard;
wi2gqsw(3,1)=aircraft.wing2.quarter_chord_sweep_outboard;
wi2glsw(1,1)=aircraft.wing2.LE_sweep_inboard;
wi2glsw(2,1)=aircraft.wing2.LE_sweep_midboard;
wi2glsw(3,1)=aircraft.wing2.LE_sweep_outboard;
wi2gdih(1,1)=aircraft.wing2.dihedral_inboard;
wi2gdih(2,1)=aircraft.wing2.dihedral_midboard;
wi2gdih(3,1)=aircraft.wing2.dihedral_outboard;
wi2gspn(1,1)=aircraft.wing2.span;
fla2Crd(1,1)=aircraft.wing2.flap.root_chord;
fla2Crd(2,1)=aircraft.wing2.flap.kink1_chord;
fla2Crd(3,1)=aircraft.wing2.flap.kink2_chord;
ail2Pos(1,1)=aircraft.wing2.aileron.position;
ail2Chr(1,1)=aircraft.wing2.aileron.chord;
ail2Spa(1,1)=aircraft.wing2.aileron.span;
sla2Crd(1,1)=aircraft.wing2.slat.chord;
sla2Spa(1,1)=aircraft.wing2.slat.root_position;
sla2Spa(2,1)=aircraft.wing2.slat.tip_position;
%% Fairing 2 inputs
fa2rfwd(1,1)=aircraft.fairing2.Forward_chord_fraction;
fa2raft(1,1)=aircraft.fairing2.Aft_chord_fraction;
fa2rovh(1,1)=aircraft.fairing2.flushness;
%% Horizontal tail inputs
taillay(1,1)=aircraft.Horizontal_tail.empennage_layout;
htalare(1,1)=aircraft.Horizontal_tail.area;
htalgar(1,1)=aircraft.Horizontal_tail.AR;
htalspn(1,1)=aircraft.Horizontal_tail.span;
htalkln(1,1)=aircraft.Horizontal_tail.spanwise_kink;
htaltap(1,1)=aircraft.Horizontal_tail.taper_kink;
htaltap(3,1)=aircraft.Horizontal_tail.taper_tip;
htalinc(1,1)=aircraft.Horizontal_tail.root_incidence;
htalinc(2,1)=aircraft.Horizontal_tail.kink_incidence;
htalinc(4,1)=aircraft.Horizontal_tail.tip_incidence;
htalqsw(1,1)=aircraft.Horizontal_tail.quarter_chord_sweep_inboard;
htalqsw(3,1)=aircraft.Horizontal_tail.quarter_chord_sweep_outboard;
htallsw(1,1)=aircraft.Horizontal_tail.LE_sweep_inboard;
htallsw(3,1)=aircraft.Horizontal_tail.LE_sweep_outboard;
htaldih(1,1)=aircraft.Horizontal_tail.dihedral_inboard;
htaldih(3,1)=aircraft.Horizontal_tail.dihedral_outboard;
htalver(1,1)=aircraft.Horizontal_tail.vertical_locale;
htalapx(1,1)=aircraft.Horizontal_tail.apex_locale;
flHTCrd(1,1)=aircraft.Horizontal_tail.Elevator_chord;
htalkln(2,1)=aircraft.Horizontal_tail.Elevator_span;
htalthk(1,1)=aircraft.Horizontal_tail.thickness_root;
htalthk(2,1)=aircraft.Horizontal_tail.thickness_kink;
htalthk(4,1)=aircraft.Horizontal_tail.thickness_tip;
htaliwt(1,1)=aircraft.Horizontal_tail.Wetted_area_increment_Horizontal_tail;
% STACTPD(1,1)=aircraft.Horizontal_tail.tailplane_deflection;
% STACLTU(1,1)=aircraft.Horizontal_tail.limit_tailplane_deflection_up;
% STACLTD(1,1)=aircraft.Horizontal_tail.limit_tailplane_deflection_down;
% STACLEU(1,1)=aircraft.Horizontal_tail.limit_elevator_deflection_up;
% STACLED(1,1)=aircraft.Horizontal_tail.limit_elevator_deflection_down;
%% Vertical tail inputs
vtalare(1,1)=aircraft.Vertical_tail.area;
vtalgar(1,1)=aircraft.Vertical_tail.AR;
vtalspn(1,1)=aircraft.Vertical_tail.span;
vtalkln(1,1)=aircraft.Vertical_tail.spanwise_kink;
vtaltap(1,1)=aircraft.Vertical_tail.taper_kink;
vtaltap(3,1)=aircraft.Vertical_tail.taper_tip;
vtalqsw(1,1)=aircraft.Vertical_tail.quarter_chord_sweep_inboard;
vtalqsw(3,1)=aircraft.Vertical_tail.quarter_chord_sweep_outboard;
vtallsw(1,1)=aircraft.Vertical_tail.LE_sweep_inboard;
vtallsw(3,1)=aircraft.Vertical_tail.LE_sweep_outboard;
vtalver(1,1)=aircraft.Vertical_tail.vertical_locale;
vtalapx(1,1)=aircraft.Vertical_tail.apex_locale;
flVTCrd(1,1)=aircraft.Vertical_tail.Rudder_chord;
vtalkln(2,1)=aircraft.Vertical_tail.Rudder_span;
% STACLRU(1,1)=aircraft.Vertical_tail.Rudder_limit_deflection;
vtalthk(1,1)=aircraft.Vertical_tail.thickness_root;
vtalthk(2,1)=aircraft.Vertical_tail.thickness_kink;
vtalthk(4,1)=aircraft.Vertical_tail.thickness_tip;
vtaldlc(1,1)=aircraft.Vertical_tail.Dorsal_location;
vtaldsw(1,1)=aircraft.Vertical_tail.Dorsal_sweep;
vtaliwt(1,1)=aircraft.Vertical_tail.Wetted_area_increment_Vertical_tail;
vtaldih(1,1)=aircraft.Vertical_tail.dihedral_inboard;
vtaldih(3,1)=aircraft.Vertical_tail.dihedral_outboard;
vtalinc(1,1)=aircraft.Horizontal_tail.root_incidence;
vtalinc(2,1)=aircraft.Horizontal_tail.kink_incidence;
vtalinc(4,1)=aircraft.Horizontal_tail.tip_incidence;
vfinvlc(1,1)=aircraft.Vertical_tail.Ventral_location;
vfinhlc(1,1)=aircraft.Vertical_tail.Ventrl_height_location_per_fuse_height;
vfinvcd(1,1)=aircraft.Vertical_tail.Ventral_chord_at_midfuse;
vfinvsp(1,1)=aircraft.Vertical_tail.Ventral_span;
vfinvkl(2,1)=aircraft.Vertical_tail.Ventral_kink;
vfinvtp(2,1)=aircraft.Vertical_tail.Ventral_taper_ratio_at_kink ;
vfinvtp(3,1)=aircraft.Vertical_tail.Ventral_taper_ratio_at_tip;
vfinvls(2,1)=aircraft.Vertical_tail.Ventral_LE_sweep_inboard;
vfinvls(3,1)=aircraft.Vertical_tail.Ventral_LE_sweep_outboard;
vfinvdh(2,1)=aircraft.Vertical_tail.Ventral_cant_inbord;
vfinvdh(3,1)=aircraft.Vertical_tail.Ventral_cant_outboard;
% vfinvhg(3,1)=aircraft.Vertical_tail.Ventral_height_per_fuse_height;
%% Bullet fairing inputs
vtalblt(1,1)=aircraft.Vertical_tail.Bullet_more_vertical_tip_chord;
vtalbsl(1,1)=aircraft.Vertical_tail.Bullet_fairing_slenderness;
%% Main Engines inputs
engenum(1,1)=aircraft.engines1.Number_of_engines;
engeloc(1,1)=aircraft.engines1.Engines_layout_and_config;
insttyp(1,1)=aircraft.engines1.Propulsion_type;
engeylc(1,1)=aircraft.engines1.Engine_spanwise_location;
engexlc(1,1)=aircraft.engines1.Engine_longitdinal_location;
engezlc(1,1)=aircraft.engines1.Engine_vertical_location;
engetoe(1,1)=aircraft.engines1.toe_in;
engepit(1,1)=aircraft.engines1.pitch;
nacetyp(1,1)=aircraft.engines1.Nacelle_body_type;
nacefcl(1,1)=aircraft.engines1.Fan_cowl_length_ratio;
nacefin(1,1)=aircraft.engines1.fineness_ratio;
nacemdi(1,1)=aircraft.engines1.d_max;
propdia(1,1)=aircraft.engines1.Propeller_diameter;
maxistc(1,1)=aircraft.engines1.Max_thrust;
bypasem(1,1)=aircraft.engines1.Bypass_ratio_to_emulate;
%% Secondary engines inputs
engenum(2,1)=aircraft.engines2.Number_of_engines;
engeloc(2,1)=aircraft.engines2.Engines_layout_and_config;
insttyp(2,1)=aircraft.engines2.Propulsion_type;
engeylc(2,1)=aircraft.engines2.Engine_spanwise_location;
engexlc(2,1)=aircraft.engines2.Engine_longitdinal_location;
engezlc(2,1)=aircraft.engines2.Engine_vertical_location;
engetoe(2,1)=aircraft.engines2.toe_in;
engepit(2,1)=aircraft.engines2.pitch;
nacetyp(2,1)=aircraft.engines2.Nacelle_body_type;
nacefin(2,1)=aircraft.engines2.fineness_ratio;
nacefcl(2,1)=aircraft.engines2.Fan_cowl_length_ratio;
nacemdi(2,1)=aircraft.engines2.d_max;
propdia(2,1)=aircraft.engines2.Propeller_diameter;
maxistc(2,1)=aircraft.engines2.Max_thrust;
bypasem(2,1)=aircraft.engines2.Bypass_ratio_to_emulate;
pylniwt(1,1)=aircraft.engines_results.Wetted_area_increment_pylons;
pwrpiwt(1,1)=aircraft.engines_results.Wetted_area_increment_powerplant;
ANCIWET(1,1)=aircraft.Wetted_area_ancilry_final;
%% Fuel inputs
wingspf(1,1)=aircraft.fuel.Fore_wing_spar_loc_root;
wingspf(2,1)=aircraft.fuel.Fore_wing_spar_loc_kik1;
wingspf(3,1)=aircraft.fuel.Fore_wing_spar_loc_kin2;
wingspf(4,1)=aircraft.fuel.Fore_wing_spar_loc_tip;
wingsau(1,1)=aircraft.fuel.Aux_wing_spar_loc_root;
wingspa(1,1)=aircraft.fuel.Aft_wing_spar_loc_root;
wingspa(2,1)=aircraft.fuel.Aft_wing_spar_loc_kin1;
wingspa(3,1)=aircraft.fuel.Aft_wing_spar_loc_kin2;
wingspa(4,1)=aircraft.fuel.Aft_wing_spar_loc_tip;
fuelcut(1,1)=aircraft.fuel.Wing_fuel_tank_cutout_opt;
fuelobd(1,1)=aircraft.fuel.Outboard_fuel_tank_span;
fuelusu(1,1)=aircraft.fuel.Unusable_fuel_option;
fuelden(1,1)=aircraft.fuel.Assumed_fuel_density ;
fuelwic(1,1)=aircraft.fuel.Incr_weight_for_wing_tanks;
fuelcen(1,1)=aircraft.fuel.Centre_tank_portion_used;
fuelcic(1,1)=aircraft.fuel.Increment_for_centre_tank;
fuelaux(1,1)=aircraft.fuel.Fore_fairing_tank_length;
fuelaux(2,1)=aircraft.fuel.Aft_fairing_tank_length;
fuelafa(1,1)=aircraft.fuel.Aft_fuse_bladder_length;
fuelaic(1,1)=aircraft.fuel.Increment_for_aux_tanks;
%% Baggage and cabin input
baggtyp(1,1)=aircraft.Baggage.installation_type;
baggvol(1,1)=aircraft.Baggage.gross_volume;
bagglgt(1,1)=aircraft.Baggage.Baggage_combined_length;
baggapx(1,1)=aircraft.Baggage.Baggage_apex_per_fuselgt;
cabnlgt(1,1)=aircraft.cabin.Cabin_length_to_aft_cab;
cabnhei(1,1)=aircraft.cabin.Cabin_max_internal_height;
cabnwid(1,1)=aircraft.cabin.Cabin_max_internal_width;
cabnfwd(1,1)=aircraft.cabin.Cabin_floor_width;
cabnvol(1,1)=aircraft.cabin.Cabin_volume;
%% Additional inputs needed for weight and balance
 destype(1,1)=aircraft.miscellaneous.Design_classification ;
 spoiler(1,1)=aircraft.miscellaneous.Spoiler_effectivity ;

 rampinc(1,1)=aircraft.weight_balance.Ramp_increment ;
 contwei(1,1)=aircraft.weight_balance.Weight_cont_allow_perc_of_MEW ;
 mantwei(1,1)=aircraft.weight_balance.Manufacturer_weights_tolerance ;
 cabnatt(1,1)=aircraft.cabin.Cabin_attendant_number    ;
 cabnfcr(1,1)=aircraft.cabin.Flight_crew_number        ;
 cabnpas(1,1)=aircraft.cabin.Passenger_accomodation    ;
 cabnsab(1,1)=aircraft.cabin.Seats_abreast_in_fuselage ;
 cabnspt(1,1)=aircraft.cabin.Seat_pitch                ;
 
%%propulsion parameters
 thrtrev(1,1)=aircraft.engines1.Thrust_reverser_effectivness ;
fixdthw(1,1)=aircraft.engines1.Thrust_to_weight_ratio ;

 thrtrev(2,1)=aircraft.engines2.Thrust_reverser_effectivness ;

 FENVVDF(1,1)=aircraft.flight_envelope_prediction.VD_Flight_envelope_dive ;
 FENVVMO(1,1)=aircraft.flight_envelope_prediction.VMO_Flight_envelope ;
%%stability parameter

%--------------------------------------------------------------------------------------
% PARAMETERS INPUT from other modules which are overidden by WB if zero value is given 
%--------------------------------------------------------------------------------------
 opercei(1,1)=aircraft.miscellaneous.Target_operating_ceiling ;
 cabncei(1,1)=aircraft.cabin.Maximum_cabin_altitude    ;
 cabnpdf(1,1)=aircraft.cabin.Max_pressure_differential ;

 
%%
%%
%%
%%
%%
aircraft1.Fuselage.Forefuse_X_sect_vertical_diameter=fusfvma(1,1);
aircraft1.Fuselage.Forefuse_Xs_distortion_coefficient=fusfvmi(1,1);
aircraft1.Fuselage.Forefuse_X_sect_horizontal_diameter=fusfhma(1,1);
aircraft1.Fuselage.omega_nose=foreslp(1,1);
aircraft1.Fuselage.phi_nose=foredsw(1,1);
aircraft1.Fuselage.epsilon_nose=forefin(1,1);
%aircraft1.Fuselage.Nose_length=forelgt(1,1);
aircraft1.Fuselage.shift_fore=ffsalgn(1,1);
aircraft1.Fuselage.fraction_fore=midfrat(1,1);
aircraft1.Fuselage.Total_fuselage_length=fuselgt(1,1);
aircraft1.Fuselage.Aftfuse_X_sect_vertical_diameter=fusevma(1,1);
aircraft1.Fuselage.Aftfuse_Xs_distortion_coefficient=fusevmi(1,1);
aircraft1.Fuselage.Aftfuse_X_sect_horizontal_diameter=fusehma(1,1);
aircraft1.Fuselage.omega_tail=btallow(1,1);
aircraft1.Fuselage.phi_tail=aftfusw(1,1);
aircraft1.Fuselage.epsilon_tail=aftsfin(1,1);
%aircraft1.Fuselage.Tail_length=aftslgt(1,1);
% aircraft1.Fuselage.Wetted_increment_fuselage_fairing=fuseiwt(1,1);
%% Sponson inputs
aircraft1.Sponson.X_locale=spsnxlc(1,1);
aircraft1.Sponson.Z_locale=spsnzlc(1,1);
aircraft1.Sponson.length=spsnlgt(1,1);
aircraft1.Sponson.XZ_slenderness=spsnxzs(1,1);
aircraft1.Sponson.width=spsnwid(1,1);

%% Wing 1 inputs
aircraft1.Wing1.configuration=wingcfg(1,1);
aircraft1.Wing1.placement=wingplc(1,1);
aircraft1.Wing1.aerofoil_technology=0;%wingprl(1,1);
aircraft1.Wing1.apex_locale=wingapx(1,1);
aircraft1.Wing1.area=wingare(1,1);
aircraft1.Wing1.AR=winggar(1,1);
aircraft1.Wing1.Span=wingspn(1,1);
aircraft1.Wing1.airfoil=aircraft.wing1.airfoil;%'MS10313.dat';
aircraft1.Wing1.spanwise_kink1=wingkln(1,1);
aircraft1.Wing1.spanwise_kink2=wingkln(2,1);
aircraft1.Wing1.taper_kink1=wingtap(1,1);
aircraft1.Wing1.taper_kink2=wingtap(2,1);
aircraft1.Wing1.taper_tip=wingtap(3,1);
aircraft1.Wing1.root_incidence=winginc(1,1);
aircraft1.Wing1.kink1_incidence=winginc(2,1);
aircraft1.Wing1.kink2_incidence=winginc(3,1);
aircraft1.Wing1.tip_incidence=winginc(4,1);
aircraft1.Wing1.quarter_chord_sweep_inboard=wingqsw(1,1);
aircraft1.Wing1.quarter_chord_sweep_midboard=wingqsw(2,1);
aircraft1.Wing1.quarter_chord_sweep_outboard=wingqsw(3,1);
aircraft1.Wing1.LE_sweep_inboard=winglsw(1,1);
aircraft1.Wing1.LE_sweep_midboard=winglsw(2,1);
aircraft1.Wing1.LE_sweep_outboard=winglsw(3,1);
aircraft1.Wing1.dihedral_inboard=wingdih(1,1);
aircraft1.Wing1.dihedral_midboard=wingdih(2,1);
aircraft1.Wing1.dihedral_outboard=wingdih(3,1);
aircraft1.Wing1.thickness_root=wingthk(1,1);
aircraft1.Wing1.thickness_kink1=wingthk(2,1);
aircraft1.Wing1.thickness_kink2=wingthk(3,1);
aircraft1.Wing1.thickness_tip=wingthk(4,1);
aircraft1.Wing1.Fractional_change_vortex_induced_drag_factor = 0;
aircraft1.Wing1.winglet.Span=wletspn(1,1);
aircraft1.Wing1.winglet.taper_ratio=wlettap(1,1);
aircraft1.Wing1.winglet.LE_sweep=wletlsw(1,1);
aircraft1.Wing1.winglet.Cant_angle=wletvos(1,1);
aircraft1.Wing1.winglet.root_incidence=wletinc(1,1);
aircraft1.Wing1.winglet.tip_incidence=wletinc(3,1);
aircraft1.Wing1.flap.root_chord=flapCrd(1,1);
aircraft1.Wing1.flap.kink1_chord=flapCrd(2,1);
aircraft1.Wing1.flap.kink2_chord=flapCrd(3,1);
aircraft1.Wing1.aileron.chord=ailrChr(1,1);
aircraft1.Wing1.aileron.Span=ailrSpa(1,1);
aircraft1.Wing1.aileron.position=ailrPos(1,1);
aircraft1.Wing1.aileron.limit_deflection_up=0;%STACLAU(1,1);
aircraft1.Wing1.aileron.limit_deflection_down=0;%STACLAD(1,1);
aircraft1.Wing1.slat.chord=slatCrd(1,1);
aircraft1.Wing1.slat.root_position=slatSpa(1,1);
aircraft1.Wing1.slat.tip_position=slatSpa(2,1);
%% Fairing 1 input
aircraft1.Fairing1.Forward_chord_fraction=fairfwd(1,1);
aircraft1.Fairing1.Aft_chord_fraction=fairaft(1,1);
aircraft1.Fairing1.flushness=fairovh(1,1);
aircraft1.Reference_wing.convention=WINREFC(1,1);
% aircraft1.wing_results.Wetted_area_increment_wings=wingiwt(1,1);
% aircraft1.wing_results.Wetted_area_increment_winglet=wletiwt(1,1);
%% Wing 2 inputs
aircraft1.Wing2.configuration=wi2gcfg(1,1);
aircraft1.Wing2.placement=wi2gplc(1,1);
aircraft1.Wing2.Apex_locale=wi2gapx(1,1);
aircraft1.Wing2.aerofoil_technology=0;%wi2gprl(1,1);
aircraft1.Wing2.area=wi2gare(1,1);
aircraft1.Wing2.AR=wi2ggar(1,1);
aircraft1.Wing2.spanwise_kink1=wi2gkln(1,1);
aircraft1.Wing2.spanwise_kink2=wi2gkln(2,1);
aircraft1.Wing2.taper_kink1=wi2gtap(1,1);
aircraft1.Wing2.taper_kink2=wi2gtap(2,1);
aircraft1.Wing2.taper_tip=wi2gtap(3,1);
aircraft1.Wing2.root_incidence=wi2ginc(1,1);
aircraft1.Wing2.kink1_incidence=wi2ginc(2,1);
aircraft1.Wing2.kink2_incidence=wi2ginc(3,1);
aircraft1.Wing2.tip_incidence=wi2ginc(4,1);
aircraft1.Wing2.quarter_chord_sweep_inboard=wi2gqsw(1,1);
aircraft1.Wing2.quarter_chord_sweep_midboard=wi2gqsw(2,1);
aircraft1.Wing2.quarter_chord_sweep_outboard=wi2gqsw(3,1);
aircraft1.Wing2.LE_sweep_inboard=wi2glsw(1,1);
aircraft1.Wing2.LE_sweep_midboard=wi2glsw(2,1);
aircraft1.Wing2.LE_sweep_outboard=wi2glsw(3,1);
aircraft1.Wing2.dihedral_inboard=wi2gdih(1,1);
aircraft1.Wing2.dihedral_midboard=wi2gdih(2,1);
aircraft1.Wing2.dihedral_outboard=wi2gdih(3,1);
aircraft1.Wing2.Span=wi2gspn(1,1);
aircraft1.Wing2.flap.root_chord=fla2Crd(1,1);
aircraft1.Wing2.flap.kink1_chord=fla2Crd(2,1);
aircraft1.Wing2.flap.kink2_chord=fla2Crd(3,1);
aircraft1.Wing2.aileron.chord=ail2Chr(1,1);
aircraft1.Wing2.aileron.Span=ail2Spa(1,1);
aircraft1.Wing2.aileron.position=ail2Pos(1,1);
aircraft1.Wing2.slat.chord=sla2Crd(1,1);
aircraft1.Wing2.slat.root_position=sla2Spa(1,1);
aircraft1.Wing2.slat.tip_position=sla2Spa(2,1);
%% Fairing 2 inputs
aircraft1.Fairing2.Forward_chord_fraction=fa2rfwd(1,1);
aircraft1.Fairing2.Aft_chord_fraction=fa2raft(1,1);
aircraft1.Fairing2.flushness=fa2rovh(1,1);
%% Vertical tail inputs
aircraft1.Vertical_tail.area=vtalare(1,1);
aircraft1.Vertical_tail.AR=vtalgar(1,1);
aircraft1.Vertical_tail.Span=vtalspn(1,1);
aircraft1.Vertical_tail.spanwise_kink=vtalkln(1,1);
aircraft1.Vertical_tail.taper_kink=vtaltap(1,1);
aircraft1.Vertical_tail.taper_tip=vtaltap(3,1);
aircraft1.Vertical_tail.quarter_chord_sweep_inboard=vtalqsw(1,1);
aircraft1.Vertical_tail.quarter_chord_sweep_outboard=vtalqsw(3,1);
aircraft1.Vertical_tail.LE_sweep_inboard=vtallsw(1,1);
aircraft1.Vertical_tail.LE_sweep_outboard=vtallsw(3,1);
aircraft1.Vertical_tail.vertical_locale=vtalver(1,1);
aircraft1.Vertical_tail.apex_locale=vtalapx(1,1);
aircraft1.Vertical_tail.thickness_root=vtalthk(1,1);
aircraft1.Vertical_tail.thickness_kink=vtalthk(2,1);
aircraft1.Vertical_tail.thickness_tip=vtalthk(4,1);
aircraft1.Vertical_tail.Rudder.chord=flVTCrd(1,1);
aircraft1.Vertical_tail.Rudder.Span=vtalkln(2,1);
aircraft1.Vertical_tail.Rudder.limit_deflection=0;%STACLRU(1,1);

aircraft1.Vertical_tail.Dorsal_location=vtaldlc(1,1);
aircraft1.Vertical_tail.Dorsal_sweep=vtaldsw(1,1);
aircraft1.Vertical_tail.dihedral_inboard=vtaldih(1,1);
aircraft1.Vertical_tail.dihedral_outboard=vtaldih(3,1);
aircraft1.Vertical_tail.root_incidence=vtalinc(1,1);
aircraft1.Vertical_tail.kink_incidence=vtalinc(2,1);
aircraft1.Vertical_tail.tip_incidence=vtalinc(4,1);

aircraft1.Ventral_fin.X_locale=vfinvlc(1,1);
aircraft1.Ventral_fin.Z_locale=vfinhlc(1,1);
aircraft1.Ventral_fin.chord_fraction_at_midfuse=vfinvcd(1,1);
aircraft1.Ventral_fin.Span=vfinvsp(1,1);
aircraft1.Ventral_fin.spanwise_kink=vfinvkl(2,1);
aircraft1.Ventral_fin.taper_kink =vfinvtp(2,1);
aircraft1.Ventral_fin.taper_tip=vfinvtp(3,1);
aircraft1.Ventral_fin.LE_sweep_inboard=vfinvls(2,1);
aircraft1.Ventral_fin.LE_sweep_outboard=vfinvls(3,1);
aircraft1.Ventral_fin.cant_inbord=vfinvdh(2,1);
aircraft1.Ventral_fin.cant_outboard=vfinvdh(3,1);
%% Horizontal tail inputs
aircraft1.Horizontal_tail.empennage_layout=taillay(1,1);
aircraft1.Horizontal_tail.area=htalare(1,1);
aircraft1.Horizontal_tail.AR=htalgar(1,1);
aircraft1.Horizontal_tail.Span=htalspn(1,1);
aircraft1.Horizontal_tail.spanwise_kink=htalkln(1,1);
aircraft1.Horizontal_tail.taper_kink=htaltap(1,1);
aircraft1.Horizontal_tail.taper_tip=htaltap(3,1);
aircraft1.Horizontal_tail.root_incidence=htalinc(1,1);
aircraft1.Horizontal_tail.kink_incidence=htalinc(2,1);
aircraft1.Horizontal_tail.tip_incidence=htalinc(4,1);
aircraft1.Horizontal_tail.quarter_chord_sweep_inboard=htalqsw(1,1);
aircraft1.Horizontal_tail.quarter_chord_sweep_outboard=htalqsw(3,1);
aircraft1.Horizontal_tail.LE_sweep_inboard=htallsw(1,1);
aircraft1.Horizontal_tail.LE_sweep_outboard=htallsw(3,1);
aircraft1.Horizontal_tail.dihedral_inboard=htaldih(1,1);
aircraft1.Horizontal_tail.dihedral_outboard=htaldih(3,1);
aircraft1.Horizontal_tail.vertical_locale=htalver(1,1);
aircraft1.Horizontal_tail.apex_locale=htalapx(1,1);
aircraft1.Horizontal_tail.Elevator.chord=flHTCrd(1,1);
aircraft1.Horizontal_tail.Elevator.Span=htalkln(2,1);
aircraft1.Horizontal_tail.thickness_root=htalthk(1,1);
aircraft1.Horizontal_tail.thickness_kink=htalthk(2,1);
aircraft1.Horizontal_tail.thickness_tip=htalthk(4,1);
aircraft1.Horizontal_tail.tailplane_deflection=0;%STACTPD(1,1);
aircraft1.Horizontal_tail.limit_tailplane_deflection_up=0;%STACLTU(1,1);
aircraft1.Horizontal_tail.limit_tailplane_deflection_down=0;%STACLTD(1,1);
aircraft1.Horizontal_tail.Elevator.limit_deflection_up=0;%STACLEU(1,1);
aircraft1.Horizontal_tail.Elevator.limit_deflection_down=0;%STACLED(1,1);

%% Bullet fairing inputs
aircraft1.Vertical_tail.Bullet_more_vertical_tip_chord=vtalblt(1,1);
aircraft1.Vertical_tail.Bullet_fairing_slenderness=vtalbsl(1,1);
%% Main Engines inputs
aircraft1.Engines1.Number_of_engines=engenum(1,1);
aircraft1.Engines1.Layout_and_config=engeloc(1,1);
aircraft1.Engines1.Propulsion_type=insttyp(1,1);
aircraft1.Engines1.Y_locale=engeylc(1,1);
aircraft1.Engines1.X_locale=engexlc(1,1);
aircraft1.Engines1.Z_locale=engezlc(1,1);
aircraft1.Engines1.toe_in=engetoe(1,1);
aircraft1.Engines1.pitch=engepit(1,1);
aircraft1.Engines1.Nacelle_body_type=nacetyp(1,1);
aircraft1.Engines1.Fan_cowl_length_ratio=nacefcl(1,1);
aircraft1.Engines1.fineness_ratio=nacefin(1,1);
aircraft1.Engines1.d_max=nacemdi(1,1);
aircraft1.Engines1.Propeller_diameter=propdia(1,1);
aircraft1.Engines1.Max_thrust=maxistc(1,1);
aircraft1.Engines1.Bypass_ratio_to_emulate=bypasem(1,1);
%% Secondary engines inputs
aircraft1.Engines2.Number_of_engines=engenum(2,1);
aircraft1.Engines2.Layout_and_config=engeloc(2,1);
aircraft1.Engines2.Propulsion_type=insttyp(2,1);
aircraft1.Engines2.Y_locale=engeylc(2,1);
aircraft1.Engines2.X_locale=engexlc(2,1);
aircraft1.Engines2.Z_locale=engezlc(2,1);
aircraft1.Engines2.toe_in=engetoe(2,1);
aircraft1.Engines2.pitch=engepit(2,1);
aircraft1.Engines2.Nacelle_body_type=nacetyp(2,1);
aircraft1.Engines2.fineness_ratio=nacefin(2,1);
aircraft1.Engines2.Fan_cowl_length_ratio=nacefcl(2,1);
aircraft1.Engines2.d_max=nacemdi(2,1);
aircraft1.Engines2.Propeller_diameter=propdia(2,1);
aircraft1.Engines2.Max_thrust=maxistc(2,1);
aircraft1.Engines2.Bypass_ratio_to_emulate=bypasem(2,1);

%% Fuel inputs
aircraft1.fuel.Fore_wing_spar_loc_root=wingspf(1,1);
aircraft1.fuel.Fore_wing_spar_loc_kik1=wingspf(2,1);
aircraft1.fuel.Fore_wing_spar_loc_kin2=wingspf(3,1);
aircraft1.fuel.Fore_wing_spar_loc_tip=wingspf(4,1);
aircraft1.fuel.Aux_wing_spar_loc_root=wingsau(1,1);
aircraft1.fuel.Aft_wing_spar_loc_root=wingspa(1,1);
aircraft1.fuel.Aft_wing_spar_loc_kin1=wingspa(2,1);
aircraft1.fuel.Aft_wing_spar_loc_kin2=wingspa(3,1);
aircraft1.fuel.Aft_wing_spar_loc_tip=wingspa(4,1);
aircraft1.fuel.Wing_fuel_tank_cutout_opt=fuelcut(1,1);
aircraft1.fuel.Outboard_fuel_tank_span=fuelobd(1,1);
aircraft1.fuel.Unusable_fuel_option=fuelusu(1,1);
aircraft1.fuel.Assumed_fuel_density =fuelden(1,1);
aircraft1.fuel.Incr_weight_for_wing_tanks=fuelwic(1,1);
aircraft1.fuel.Centre_tank_portion_used=fuelcen(1,1);
aircraft1.fuel.Increment_for_centre_tank=fuelcic(1,1);
aircraft1.fuel.Fore_fairing_tank_length=fuelaux(1,1);
aircraft1.fuel.Aft_fairing_tank_length=fuelaux(2,1);
aircraft1.fuel.Aft_fuse_bladder_length=fuelafa(1,1);
aircraft1.fuel.Increment_for_aux_tanks=fuelaic(1,1);
%% Baggage and cabin input
aircraft1.Baggage.installation_type=baggtyp(1,1);
aircraft1.Baggage.gross_volume=baggvol(1,1);
aircraft1.Baggage.Baggage_combined_length=bagglgt(1,1);
aircraft1.Baggage.Baggage_apex_per_fuselgt=baggapx(1,1);
aircraft1.cabin.Cabin_length_to_aft_cab=cabnlgt(1,1);
aircraft1.cabin.Cabin_max_internal_height=cabnhei(1,1);
aircraft1.cabin.Cabin_max_internal_width=cabnwid(1,1);
aircraft1.cabin.Cabin_floor_width=cabnfwd(1,1);
aircraft1.cabin.Cabin_volume=cabnvol(1,1);

%% Additional inputs needed for weight and balance
aircraft1.miscellaneous.Design_classification = destype(1,1);
aircraft1.miscellaneous.Spoiler_effectivity = spoiler(1,1);
aircraft1.miscellaneous.Undercarriage_layout = 0;%undrloc(1,1);

aircraft1.weight_balance.Ramp_increment = rampinc(1,1); 
aircraft1.weight_balance.Weight_cont_allow_perc_of_MEW = contwei(1,1); 
aircraft1.weight_balance.Manufacturer_weights_tolerance = mantwei(1,1); 

aircraft1.cabin.Cabin_attendant_number    = cabnatt(1,1);
aircraft1.cabin.Flight_crew_number        = cabnfcr(1,1);
aircraft1.cabin.Passenger_accomodation    = cabnpas(1,1);
aircraft1.cabin.Seats_abreast_in_fuselage = cabnsab(1,1); 
aircraft1.cabin.Seat_pitch                = cabnspt(1,1); 


%%propulsion parameters
aircraft1.Engines1.Thrust_reverser_effectivness = thrtrev(1,1);
aircraft1.Engines1.Thrust_to_weight_ratio =fixdthw(1,1);
% aircraft1.Engines1.Location_engines_nacelles_on_X = 6.1786;
% aircraft1.Engines1.Location_engines_nacelles_on_Y = 0;
% aircraft1.Engines1.Location_engines_nacelles_on_Z = -1.742; 
% aircraft1.Engines1.Nacelle_length_array = 0; 

aircraft1.Engines2.Thrust_reverser_effectivness = thrtrev(2,1);
% aircraft1.Engines2.Location_engines_nacelles_on_X = 11.506;
% aircraft1.Engines2.Location_engines_nacelles_on_Y = 0;
% aircraft1.Engines2.Location_engines_nacelles_on_Z = 1.617;
% aircraft1.Engines2.Nacelle_length_array = 0;   

aircraft1.flight_envelope_prediction.VD_Flight_envelope_dive = FENVVDF(1,1);
aircraft1.flight_envelope_prediction.VMO_Flight_envelope = FENVVMO(1,1);
%%stability parameter
% aircraft1.stability.All_up_weight = 0;
% aircraft1.Wing_results.Wetted_area_increment_wings=wingiwt(1,n);
% aircraft1.Wing_results.Wetted_area_increment_winglet=wletiwt(1,n);
aircraft1.Wetted_areas.increment_fuselage_fairing=fuseiwt(1,1);
aircraft1.Wetted_areas.increment_wings=wingiwt(1,1);
aircraft1.Wetted_areas.increment_winglet=wletiwt(1,1); 
aircraft1.Wetted_areas.increment_Vertical_tail=vtaliwt(1,1);
aircraft1.Wetted_areas.increment_pylons=pylniwt(1,1);
aircraft1.Wetted_areas.increment_powerplant=pwrpiwt(1,1);
aircraft1.Wetted_areas.increment_Horizontal_tail=htaliwt(1,1);
aircraft1.Wetted_areas.anciliary_final=ANCIWET(1,1);
%--------------------------------------------------------------------------------------
% PARAMETERS INPUT from other modules which are overidden by WB if zero value is given 
%--------------------------------------------------------------------------------------

aircraft1.miscellaneous.Target_operating_ceiling = opercei(1,1);
aircraft1.cabin.Maximum_cabin_altitude    = cabncei(1,1);
aircraft1.cabin.Max_pressure_differential = cabnpdf(1,1);

%% Aditional data that are not int he structure
aircraft1.Horizontal_tail.airfoil=aircraft.Horizontal_tail.airfoil;%'NACA0012.dat';
aircraft1.Vertical_tail.airfoil=aircraft.Vertical_tail.airfoil;%'NACA0012.dat';
 
neocass_xmlunwrapper(strcat(file2read,'_new','.xml'),aircraft1);