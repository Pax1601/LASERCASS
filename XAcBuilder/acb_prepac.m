%
% acb_prepac.m
%
% Author: Martin Lahuta, (c) 2008,2009 VZLU (www.vzlu.cz)
% Developed within SimSAC project, www.simsacdesign.org
% Any usage without an explicit authorization may be persecuted.
%
%
% Modifications:
%	DATE		VERS	PROGRAMMER	DESCRIPTION
%	08.12.09	1.0	M. Lahuta	last update
% 
% 
% function checks whether data imported into 'ac' structure are compatible
% with AcBuilder (needed for old versions of xml files) and eventually uses
% default values
%
function acb_prepac

global ac

% wing1
if ~chkfield(ac,'Wing1.present')
    if chkfield(ac,'Wing1.area')
        if ac.Wing1.area~=0.0
            ac.Wing1.present=true;
        else
            ac.Wing1.present=false;
        end
    else
        ac.Wing1.present=false;
    end
end
if ac.Wing1.present
    % winglet1
    if ~chkfield(ac,'Wing1.winglet.present')
        if chkfield(ac,'Wing1.winglet.Span')
            if ac.Wing1.winglet.Span~=0
                ac.Wing1.winglet.present=true;
            else
                ac.Wing1.winglet.present=false;
            end
        else
            ac.Wing1.winglet.present=false;
        end
    end
    % flap
    if ~chkfield(ac,'Wing1.flap.present')
        ac.Wing1.flap.present=false;
    end
    % slat
    if ~chkfield(ac,'Wing1.slat.present')
        ac.Wing1.slat.present=false;
    end
    % aileron
    if ~chkfield(ac,'Wing1.aileron.present')
        ac.Wing1.aileron.present=false;
    end
    % airfoils
    if chkfield(ac,'Wing1.airfoilRoot')
        airfoil=chkaf(ac.Wing1.airfoilRoot);
        ac.Wing1.airfoilRoot=airfoil;
    end
    if chkfield(ac,'Wing1.airfoilKink1')
        airfoil=chkaf(ac.Wing1.airfoilKink1);
        ac.Wing1.airfoilKink1=airfoil;
    end
    if chkfield(ac,'Wing1.airfoilKink2')
        airfoil=chkaf(ac.Wing1.airfoilKink2);
        ac.Wing1.airfoilKink2=airfoil;
    end
    if chkfield(ac,'Wing1.airfoilTip')
        airfoil=chkaf(ac.Wing1.airfoilTip);
        ac.Wing1.airfoilTip=airfoil;
    end
    % reference_convention
    if ~chkfield(ac,'Wing1.reference_convention')
        if chkfield(ac,'Reference_wing.convention')
            ac.Wing1.reference_convention=ac.Reference_wing.convention;
        end
    end
    % fairing1
    if chkfield(ac,'Wing1.fairing.present')
        ac.Fairing1.present=true;
        ac.Fairing1.Forward_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Forward_chord_fraction'));
        ac.Fairing1.Aft_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Aft_chord_fraction'));
        ac.Fairing1.flushness=str2num(acb_loadvar('Wing1.fairing.flushness'));
    elseif chkfield(ac,'Fairing1.flushness')
        if ac.Fairing1.flushness~=0.0
            ac.Fairing1.present=true;
            ac.Wing1.fairing.present=true;
            ac.Wing1.fairing.flushness=ac.Fairing1.flushness;
            if chkfield(ac,'Fairing1.Forward_chord_fraction')
                ac.Wing1.fairing.Forward_chord_fraction=ac.Fairing1.Forward_chord_fraction;
            else
                ac.Wing1.fairing.Forward_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Forward_chord_fraction'));
            end
            if chkfield(ac,'Fairing1.Aft_chord_fraction')
                ac.Wing1.fairing.Aft_chord_fraction=ac.Fairing1.Aft_chord_fraction;
            else
                ac.Wing1.fairing.Aft_chord_fraction=str2num(acb_loadvar('Wing1.fairing.Aft_chord_fraction'));
            end
        else
            ac.Fairing1.present=false;
            ac.Wing1.fairing.present=false;
        end
    else
        ac.Fairing1.present=false;
        ac.Wing1.fairing.present=false;
    end
    %
    ac.Wing1.quarter_chord_sweep_inboard=0.0;
    ac.Wing1.quarter_chord_sweep_midboard=0.0;
    ac.Wing1.quarter_chord_sweep_outboard=0.0;

    ac.fuel.box_ea_loc_root=str2num(acb_loadvar('fuel.box_ea_loc_root'));
    ac.fuel.box_ea_loc_kink1=str2num(acb_loadvar('fuel.box_ea_loc_kink1'));
    ac.fuel.box_ea_loc_kink2=str2num(acb_loadvar('fuel.box_ea_loc_kink2'));
    ac.fuel.box_ea_loc_tip=str2num(acb_loadvar('fuel.box_ea_loc_tip'));
    ac.fuel.box_semispan_root=str2num(acb_loadvar('fuel.box_semispan_root'));
    ac.fuel.box_semispan_kink1=str2num(acb_loadvar('fuel.box_semispan_kink1'));
    ac.fuel.box_semispan_kink2=str2num(acb_loadvar('fuel.box_semispan_kink2'));
    ac.fuel.box_semispan_tip=str2num(acb_loadvar('fuel.box_semispan_tip'));
    ac.fuel.Fore_wing_spar_loc_root=ac.fuel.box_ea_loc_root-ac.fuel.box_semispan_root;
    ac.fuel.Fore_wing_spar_loc_kik1=ac.fuel.box_ea_loc_kink1-ac.fuel.box_semispan_kink1;
    ac.fuel.Fore_wing_spar_loc_kin2=ac.fuel.box_ea_loc_kink2-ac.fuel.box_semispan_kink2;
    ac.fuel.Fore_wing_spar_loc_tip=ac.fuel.box_ea_loc_tip-ac.fuel.box_semispan_tip;
    ac.fuel.Aft_wing_spar_loc_root=ac.fuel.box_ea_loc_root+ac.fuel.box_semispan_root;
    ac.fuel.Aft_wing_spar_loc_kin1=ac.fuel.box_ea_loc_kink1+ac.fuel.box_semispan_kink1;
    ac.fuel.Aft_wing_spar_loc_kin2=ac.fuel.box_ea_loc_kink2+ac.fuel.box_semispan_kink2;
    ac.fuel.Aft_wing_spar_loc_tip=ac.fuel.box_ea_loc_tip+ac.fuel.box_semispan_tip;
else
    ac.Fairing1.present=false;
end

% wing2
if ~chkfield(ac,'Wing2.present')
    if chkfield(ac,'Wing2.area')
        if ac.Wing2.area~=0.0
            ac.Wing2.present=true;
        else
            ac.Wing2.present=false;
        end
    else
        ac.Wing2.present=false;
    end
end
if ac.Wing2.present
    % winglet1
    if ~chkfield(ac,'Wing2.winglet.present')
        if chkfield(ac,'Wing2.winglet.Span')
            if ac.Wing2.winglet.Span~=0
                ac.Wing2.winglet.present=true;
            else
                ac.Wing2.winglet.present=false;
            end
        else
            ac.Wing2.winglet.present=false;
        end
    end
    % flap
    if ~chkfield(ac,'Wing2.flap.present')
        ac.Wing2.flap.present=false;
    end
    % slat
    if ~chkfield(ac,'Wing2.slat.present')
        ac.Wing2.slat.present=false;
    end
    % aileron
    if ~chkfield(ac,'Wing2.aileron.present')
        ac.Wing2.aileron.present=false;
    end
    % airfoils
    if chkfield(ac,'Wing2.airfoilRoot')
        airfoil=chkaf(ac.Wing2.airfoilRoot);
        ac.Wing2.airfoilRoot=airfoil;
    end
    if chkfield(ac,'Wing2.airfoilKink1')
        airfoil=chkaf(ac.Wing2.airfoilKink1);
        ac.Wing2.airfoilKink1=airfoil;
    end
    if chkfield(ac,'Wing2.airfoilKink2')
        airfoil=chkaf(ac.Wing2.airfoilKink2);
        ac.Wing2.airfoilKink2=airfoil;
    end
    if chkfield(ac,'Wing2.airfoilTip')
        airfoil=chkaf(ac.Wing2.airfoilTip);
        ac.Wing2.airfoilTip=airfoil;
    end
    % reference_convention
    if ~chkfield(ac,'Wing2.reference_convention')
        if chkfield(ac,'Reference_wing.convention')
            ac.Wing2.reference_convention=ac.Reference_wing.convention;
        end
    end
    % fairing2
    if chkfield(ac,'Wing2.fairing.present')
        ac.Fairing2.present=true;
        ac.Fairing2.Forward_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Forward_chord_fraction'));
        ac.Fairing2.Aft_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Aft_chord_fraction'));
        ac.Fairing2.flushness=str2num(acb_loadvar('Wing2.fairing.flushness'));
    elseif chkfield(ac,'Fairing2.flushness')
        if ac.Fairing2.flushness~=0.0
            ac.Fairing2.present=true;
            ac.Wing2.fairing.present=true;
            ac.Wing2.fairing.flushness=ac.Fairing2.flushness;
            if chkfield(ac,'Fairing2.Forward_chord_fraction')
                ac.Wing2.fairing.Forward_chord_fraction=ac.Fairing2.Forward_chord_fraction;
            else
                ac.Wing2.fairing.Forward_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Forward_chord_fraction'));
            end
            if chkfield(ac,'Fairing2.Aft_chord_fraction')
                ac.Wing2.fairing.Aft_chord_fraction=ac.Fairing2.Aft_chord_fraction;
            else
                ac.Wing2.fairing.Aft_chord_fraction=str2num(acb_loadvar('Wing2.fairing.Aft_chord_fraction'));
            end
        else
            ac.Fairing2.present=false;
            ac.Wing2.fairing.present=false;
        end
    else
        ac.Fairing2.present=false;
        ac.Wing2.fairing.present=false;
    end
    %
    ac.Wing2.quarter_chord_sweep_inboard=0.0;
    ac.Wing2.quarter_chord_sweep_midboard=0.0;
    ac.Wing2.quarter_chord_sweep_outboard=0.0;
else
    ac.Fairing2.present=false;
end

% horizontal tail
if ~chkfield(ac,'Horizontal_tail.present')
    if chkfield(ac,'Horizontal_tail.area')
        if ac.Horizontal_tail.area~=0.0
            ac.Horizontal_tail.present=true;
        else
            ac.Horizontal_tail.present=false;
        end
    else
        ac.Horizontal_tail.present=false;
    end
end
if ac.Horizontal_tail.present
    % elevator
    if ~chkfield(ac,'Horizontal_tail.Elevator.present')
        ac.Horizontal_tail.Elevator.present=false;
    end
    % airfoil
    if chkfield(ac,'Horizontal_tail.airfoilRoot')
        airfoil=chkaf(ac.Horizontal_tail.airfoilRoot);
        ac.Horizontal_tail.airfoilRoot=airfoil;
    end
    if chkfield(ac,'Horizontal_tail.airfoilKink')
        airfoil=chkaf(ac.Horizontal_tail.airfoilKink);
        ac.Horizontal_tail.airfoilKink=airfoil;
    end
    if chkfield(ac,'Horizontal_tail.airfoilTip')
        airfoil=chkaf(ac.Horizontal_tail.airfoilTip);
        ac.Horizontal_tail.airfoilTip=airfoil;
    end
    ac.Horizontal_tail.quarter_chord_sweep_inboard=0.0;
    ac.Horizontal_tail.quarter_chord_sweep_outboard=0.0;
end

% vertical tail
if ~chkfield(ac,'Vertical_tail.present')
    if chkfield(ac,'Vertical_tail.area')
        if ac.Vertical_tail.area~=0.0
            ac.Vertical_tail.present=true;
        else
            ac.Vertical_tail.present=false;
        end
    else
        ac.Vertical_tail.present=false;
    end
end

if ac.Vertical_tail.present
    % rudder
    if ~chkfield(ac,'Vertical_tail.Rudder.present')
        ac.Vertical_tail.Rudder.present=false;
    end
    % airfoil
    if chkfield(ac,'Vertical_tail.airfoilRoot')
        airfoil=chkaf(ac.Vertical_tail.airfoilRoot);
        ac.Vertical_tail.airfoilRoot=airfoil;
    end
    if chkfield(ac,'Vertical_tail.airfoilKink')
        airfoil=chkaf(ac.Vertical_tail.airfoilKink);
        ac.Vertical_tail.airfoilKink=airfoil;
    end
    if chkfield(ac,'Vertical_tail.airfoilTip')
        airfoil=chkaf(ac.Vertical_tail.airfoilTip);
        ac.Vertical_tail.airfoilTip=airfoil;
    end
    ac.Vertical_tail.Dorsal_location=0.0;
    ac.Vertical_tail.Dorsal_sweep=0.0;
    ac.Vertical_tail.Bullet_more_vertical_tip_chord=0.0;
    ac.Vertical_tail.Bullet_fairing_slenderness=0.0;
    ac.Vertical_tail.quarter_chord_sweep_inboard=0.0;
    ac.Vertical_tail.quarter_chord_sweep_outboard=0.0;
end

% canard
if ~chkfield(ac,'Canard.present')
    if chkfield(ac,'Canard.area')
        if ac.Canard.area~=0.0
            ac.Canard.present=true;
        else
            ac.Canard.present=false;
        end
    else
        ac.Canard.present=false;
    end
end
if ac.Canard.present
    % elevator
    if ~chkfield(ac,'Canard.Elevator.present')
        ac.Canard.Elevator.present=false;
    end
    % airfoil
    if chkfield(ac,'Canard.airfoilRoot')
        airfoil=chkaf(ac.Canard.airfoilRoot);
        ac.Canard.airfoilRoot=airfoil;
    end
    if chkfield(ac,'Canard.airfoilKink')
        airfoil=chkaf(ac.Canard.airfoilKink);
        ac.Canard.airfoilKink=airfoil;
    end
    if chkfield(ac,'Canard.airfoilTip')
        airfoil=chkaf(ac.Canard.airfoilTip);
        ac.Canard.airfoilTip=airfoil;
    end
    ac.Canard.quarter_chord_sweep_inboard=0.0;
    ac.Canard.quarter_chord_sweep_outboard=0.0;
end

% engines1
if chkfield(ac,'Engines1.present')
   if ac.Engines1.present
      if chkfield(ac,'Engines1.symmetry')
	  if ac.Engines1.symmetry
	      ac.Engines1.Number_of_engines=2;
	  else
	      ac.Engines1.Number_of_engines=1;
	  end
      else
	  ac.Engines1.Number_of_engines=2;
	  ac.Engines1.symmetry=true;
      end
   else
      ac.Engines1.Number_of_engines=0;
      ac.Engines1.symmetry=true;
   end
else
   if chkfield(ac,'Engines1.Number_of_engines')
      if ac.Engines1.Number_of_engines==2
	  ac.Engines1.present=true;
	  ac.Engines1.symmetry=true;
      elseif ac.Engines1.Number_of_engines==1
	  ac.Engines1.present=true;
	  ac.Engines1.symmetry=false;
      else
	  ac.Engines1.present=false;
	  ac.Engines1.Number_of_engines=0;
	  ac.Engines1.symmetry=true;
      end
   else
       ac.Engines1.present=false;
       ac.Engines1.Number_of_engines=0;
       ac.Engines1.symmetry=true;
   end
end

% engines2
if chkfield(ac,'Engines2.present')
   if ac.Engines2.present
      if chkfield(ac,'Engines2.symmetry')
	  if ac.Engines2.symmetry
	      ac.Engines2.Number_of_engines=2;
	  else
	      ac.Engines2.Number_of_engines=1;
	  end
      else
	  ac.Engines2.Number_of_engines=2;
	  ac.Engines2.symmetry=true;
      end
   else
      ac.Engines2.Number_of_engines=0;
      ac.Engines2.symmetry=true;
   end
else
   if chkfield(ac,'Engines2.Number_of_engines')
      if ac.Engines2.Number_of_engines==2
	  ac.Engines2.present=true;
	  ac.Engines2.symmetry=true;
      elseif ac.Engines2.Number_of_engines==1
	  ac.Engines2.present=true;
	  ac.Engines2.symmetry=false;
      else
	  ac.Engines2.present=false;
	  ac.Engines2.Number_of_engines=0;
	  ac.Engines2.symmetry=true;
      end
   else
       ac.Engines2.present=false;
       ac.Engines2.Number_of_engines=0;
       ac.Engines2.symmetry=true;
   end
end

% ventral fin
if ~chkfield(ac,'Ventral_fin.present')
    if chkfield(ac,'Ventral_fin.Span')
        if ac.Ventral_fin.Span~=0.0
            ac.Ventral_fin.present=true;
        else
            ac.Ventral_fin.present=false;
        end
    else
        ac.Ventral_fin.present=false;
    end
end

% tailbooms
if ~chkfield(ac,'Tailbooms.present')
   if chkfield(ac,'Tailbooms.total_length')
      if ac.Tailbooms.total_length~=0.0
	 ac.Tailbooms.present=true;
      else
	 ac.Tailbooms.present=false;
      end
   else
      ac.Tailbooms.present=false;
   end
end

ac.designation=1;

%
% check variables set in wb_struct_init.m
%

% Aerodynamic parameter
if ~chkfield(ac,'wing.Fractional_change_vortex_induced_drag_factor')
   ac.wing.Fractional_change_vortex_induced_drag_factor = 0.0;
end

% Flight envelope
if ~chkfield(ac,'weight_balance.flight_envelope_prediction.VD_Flight_envelope_dive')
   ac.weight_balance.flight_envelope_prediction.VD_Flight_envelope_dive =  365;
end
if ~chkfield(ac,'weight_balance.flight_envelope_prediction.VMO_Flight_envelope')
   ac.weight_balance.flight_envelope_prediction.VMO_Flight_envelope     =  285;
end

if ~chkfield(ac,'stability.All_up_weight')
   ac.stability.All_up_weight = 0.0;
end

ac.weight_balance.System.Landing_gear = 0.0;
ac.weight_balance.Struct.Wings = 0.0;
ac.weight_balance.Struct.Winglet_and_span_load_penalty = 0.0;
ac.weight_balance.Struct.Horizontal_tail_and_elevator = 0.0;
ac.weight_balance.Struct.Vertical_tail_rudder_and_dorsal = 0.0;
ac.weight_balance.Struct.Ventral_fins = 0.0;
ac.weight_balance.Struct.Fuselage = 0.0;
ac.weight_balance.Powerplant.Pylons_and_or_propellers = 0.0;
ac.weight_balance.Powerplant.Engines_acc_propeller_gearbox = 0.0;
ac.weight_balance.Powerplant.Nacelles = 0.0;
ac.weight_balance.Crew.Crew_and_carry_on = 0.0;
ac.weight_balance.Crew.Cabin_attendant_weight = 0.0;
ac.weight_balance.Crew.Flight_crew_weight = 0.0;
ac.weight_balance.Fuel.Maximum_fuel_weight = 0.0;
ac.weight_balance.Fuel.Fuel_to_MTOW_at_maximum_payload = 0.0;
ac.weight_balance.MTOW_Maximum_takeoff_weight = 0.0;
ac.weight_balance.OEW_Operational_empty_weight = 0.0;
ac.weight_balance.Green_Manufacturer_empty_weight = 0.0;
ac.weight_balance.Maximum_payload_weight = 0.0;
ac.weight_balance.Maximum_fuel_weight = 0.0;
ac.weight_balance.MRW_Maximum_ramp_weight = 0.0; 
ac.weight_balance.MZFW_Maximum_zero_fuel_weight = 0.0;
ac.weight_balance.Merit.Maximum_payload_per_passenger = 0.0;
ac.weight_balance.Merit.Payload_to_MTOW_at_MFW = 0.0;
ac.weight_balance.Merit.Load_factor_to_MTOW_at_MFW = 0.0;
ac.weight_balance.Merit.Merit_function_OEW_over_MTOW = 0.0;
ac.weight_balance.Merit.Merit_function_W_over_S_gross = 0.0;
ac.weight_balance.Merit.Merit_function_thrust_to_weight = 0.0;
ac.weight_balance.Merit.Merit_function_load_parameter = 0.0;

%
ac.weight_balance.MEW_longitudinal_CoG  = 0.0;
ac.weight_balance.MEW_lateral_CoG  = 0.0;
ac.weight_balance.MEW_vertical_CoG  = 0.0;
ac.weight_balance.Maximum_payload_at_MTOW_longitudinal_CoG = 0.0;
ac.weight_balance.Maximum_payload_at_MTOW_lateral_CoG = 0.0;
ac.weight_balance.Maximum_payload_at_MTOW_vertical_CoG = 0.0;
%
if ~chkfield(ac,'weight_balance.Imat')
   ac.weight_balance.Imat = zeros(3, 3);
end
if ~chkfield(ac,'weight_balance.IXXINER')
   ac.weight_balance.IXXINER = 0.;
   ac.weight_balance.IYYINER = 0.;
   ac.weight_balance.IZZINER = 0.;
   ac.weight_balance.IXZINER = 0.;
   ac.weight_balance.IYZINER = 0.;
   ac.weight_balance.IXYINER = 0.;
end

% ac.weight_balance.miscellaneous.main_landing_gear_x_cg=ac.miscellaneous.main_landing_gear_x_cg;
% ac.weight_balance.miscellaneous.main_landing_gear_z_cg=ac.miscellaneous.main_landing_gear_z_cg;
% ac.weight_balance.miscellaneous.aux_landing_gear_x_cg=ac.miscellaneous.aux_landing_gear_x_cg;
% ac.weight_balance.miscellaneous.aux_landing_gear_z_cg=ac.miscellaneous.aux_landing_gear_z_cg;

% end of wb_struct_init.m

% check whether field specified by path string exists in structure 'a'
function [res]=chkfield(a,path)
res=0;
[tok,rem]=strtok(path,'.');
path='';
while ~isempty(rem)
    if eval(['~isfield(a' path ',''' tok ''')'])
        return
    end
    path=[path '.' tok];
    [tok,rem]=strtok(rem,'.');
end
if eval(['~isstruct(a' path ')'])
    return
end
res=eval(['isfield(a' path ',''' tok ''')']);

% add .dat extension to airfoil's name if not NACA
function [af]=chkaf(af)
ind=findstr(af,'.dat');
if ~isempty(ind)
    af=af(1:ind-1);
    return
end
