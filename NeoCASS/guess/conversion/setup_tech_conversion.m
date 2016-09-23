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

function [tech] = setup_tech_conversion(tech_user_friendly,aircraft)

% Modified by Travaglini 19/11/2009
% There are one struct for canard and one for wing2 without confusing wing2
% with canard
%--------------------------------------------------------------------------------------------------
% Define *user_input* section	
if isfield(tech_user_friendly.user_input,'deformation')
    tech.deformation = tech_user_friendly.user_input.deformation;
end
tech.optimization_smonoq = tech_user_friendly.user_input.analysis_setup.optimization_smonoq;
%

tech.wing = tech_user_friendly.user_input.material_property.wing;
% tech.wing.esw   =	tech_user_friendly.user_input.material_property.wing.esw;
% tech.wing.fcsw	=	tech_user_friendly.user_input.material_property.wing.fcsw;
% tech.wing.dsw	=	tech_user_friendly.user_input.material_property.wing.dsw;
% tech.wing.kcon	=	tech_user_friendly.user_input.material_property.wing.kcon;

tech.fus = tech_user_friendly.user_input.material_property.fus;
% tech.fus.kcon	=	tech_user_friendly.user_input.material_property.fus.kcon;
% tech.fus.fts	=	tech_user_friendly.user_input.material_property.fus.fts;
% tech.fus.fcs	=	tech_user_friendly.user_input.material_property.fus.fcs;
% tech.fus.es     =	tech_user_friendly.user_input.material_property.fus.es;
% tech.fus.ef     =	tech_user_friendly.user_input.material_property.fus.ef;
% tech.fus.ds     =	tech_user_friendly.user_input.material_property.fus.ds;
% tech.fus.df     =	tech_user_friendly.user_input.material_property.fus.df;

tech.vtail = tech_user_friendly.user_input.material_property.vtail;
% tech.vtail.kcon	=	tech_user_friendly.user_input.material_property.vtail.kcon;
% tech.vtail.esw	=	tech_user_friendly.user_input.material_property.vtail.esw;
% tech.vtail.dsw	=	tech_user_friendly.user_input.material_property.vtail.dsw;
% tech.vtail.fcsw	=	tech_user_friendly.user_input.material_property.vtail.fcsw;
if aircraft.Horizontal_tail.present
    tech.htail = tech_user_friendly.user_input.material_property.htail;
end
% tech.htail.kcon	=	tech_user_friendly.user_input.material_property.htail.kcon;
% tech.htail.esw	=	tech_user_friendly.user_input.material_property.htail.esw;
% tech.htail.dsw	=	tech_user_friendly.user_input.material_property.htail.dsw;
% tech.htail.fcsw	=	tech_user_friendly.user_input.material_property.htail.fcsw;

if aircraft.Wing2.present
    tech.wing2 = tech_user_friendly.user_input.material_property.wing2;
    % tech.wing2.kcon	=	tech_user_friendly.user_input.material_property.wing2.kcon;
    % tech.wing2.esw	=	tech_user_friendly.user_input.material_property.wing2.esw;
    % tech.wing2.dsw	=	tech_user_friendly.user_input.material_property.wing2.dsw;
    % tech.wing2.fcsw	=	tech_user_friendly.user_input.material_property.wing2.fcsw;
end
if aircraft.Canard.present
    tech.canard = tech_user_friendly.user_input.material_property.canard;
end
% 



tech.trdata.deslf	=	tech_user_friendly.user_input.loading.normal_load_factor;
tech.trdata.clan	=	tech_user_friendly.user_input.loading.weight_fraction.clan;

tech.load = tech_user_friendly.user_input.loading.weight_fraction;
% tech.load.cman    =	tech_user_friendly.user_input.loading.weight_fraction.cman;
% tech.weight.cbum	=	tech_user_friendly.user_input.loading.weight_fraction.cbum;
% tech.weight.clan	=	tech_user_friendly.user_input.loading.weight_fraction.clan;

tech.stability_der = tech_user_friendly.user_input.loading.aero_data;

% tech.stability_der.Cn_dr        =	tech_user_friendly.user_input.loading.aero_data.Cn_dr;
% tech.stability_der.Cl_dr        =	tech_user_friendly.user_input.loading.aero_data.Cl_dr;
% tech.stability_der.Cn_dw        =	tech_user_friendly.user_input.loading.aero_data.Cn_dw;
% tech.stability_der.Cl_dw        =	tech_user_friendly.user_input.loading.aero_data.Cl_dw;
% tech.stability_der.Cn_b         =	tech_user_friendly.user_input.loading.aero_data.Cn_b;
% tech.stability_der.Cl_b         =	tech_user_friendly.user_input.loading.aero_data.Cl_b;
% tech.stability_der.CY_b         =	tech_user_friendly.user_input.loading.aero_data.CY_b;
% tech.stability_der.CY_dr        =	tech_user_friendly.user_input.loading.aero_data.CY_dr;
% tech.stability_der.CY_dw        =	tech_user_friendly.user_input.loading.aero_data.CY_dw;
% tech.stability_der.L_alpha_s    =	tech_user_friendly.user_input.loading.aero_data.L_alpha_s;
% tech.stability_der.M_alpha_s    =	tech_user_friendly.user_input.loading.aero_data.M_alpha_s;
% tech.stability_der.L_delta_e    =	tech_user_friendly.user_input.loading.aero_data.L_delta_e;
% tech.stability_der.M_delta_e    =	tech_user_friendly.user_input.loading.aero_data.M_delta_e;
% tech.stability_der.Lc	        =	tech_user_friendly.user_input.loading.aero_data.Lc;
% tech.stability_der.Mc	        =	tech_user_friendly.user_input.loading.aero_data.Mc;
% tech.stability_der.dM_025dalpha	=	tech_user_friendly.user_input.loading.aero_data.dM_025dalpha;
% tech.stability_der.CL	        =	tech_user_friendly.user_input.loading.aero_data.CL;
% tech.stability_der.CY_alpha	    =	tech_user_friendly.user_input.loading.aero_data.CY_alpha;

% canard

% tech.stability_der.L_alpha_sc    =	tech_user_friendly.user_input.loading.aero_data.L_alpha_sc;
% tech.stability_der.M_alpha_sc    =	tech_user_friendly.user_input.loading.aero_data.M_alpha_sc;
% tech.stability_der.L_delta_ec    =	tech_user_friendly.user_input.loading.aero_data.L_delta_ec;
% tech.stability_der.M_delta_ec    =    tech_user_friendly.user_input.loading.aero_data.M_delta_ec;

tech.defl_max	=	tech_user_friendly.user_input.loading.maximum_deflection;
% 
%             Rudder_limit_deflection
%        user_input.loading.maximum_deflection.Elevator_limit_deflection_up
%      user_input.loading.maximum_deflection.Elevator_limit_deflection_down
%       limit_tailplane_deflection_up
%     limit_tailplane_deflection_down

% if there is a canard, there are also
%        CElevator_limit_deflection_up
%      CElevator_limit_deflection_down
%       limit_canard_deflection_up
%     limit_canard_deflection_down

% 
tech.fact.stab	    =	tech_user_friendly.user_input.analysis_setup.pressure_stabilization;
tech.fact.ischrenk	=	tech_user_friendly.user_input.analysis_setup.lift_distribution;


% tech.fact = tech_user_friendly.user_input.analysis_setup.regression;
tech.fact.analf	    =	tech_user_friendly.user_input.analysis_setup.regression.analf;
tech.fact.analw	    =	tech_user_friendly.user_input.analysis_setup.regression.analw;
tech.fact.analh	    =	tech_user_friendly.user_input.analysis_setup.regression.analh;
tech.fact.analv	    =	tech_user_friendly.user_input.analysis_setup.regression.analv;
tech.fact.analw2	=	tech_user_friendly.user_input.analysis_setup.regression.analw2;

if aircraft.Canard.present
    tech.fact.analc	    =	tech_user_friendly.user_input.analysis_setup.regression.analc;
end

tech.stick.model	=	tech_user_friendly.user_input.analysis_setup.beam_model;


tech.stability_der.ival	=	tech_user_friendly.user_input.analysis_setup.vlm_calculation;
tech.ibredt	=	tech_user_friendly.user_input.analysis_setup.torsion_stiffness;
% 
tech.stick.nwing_inboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing.inboard;
tech.stick.nwing_midboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing.midboard;
tech.stick.nwing_outboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing.outboard;
tech.stick.nwing_carryth	=	tech_user_friendly.user_input.geometry.beam_model.nwing_carryth;

tech.stick.nfuse	=	tech_user_friendly.experienced_user_input.geometry.guess.fus;



tech.stick.nvtail_inboard	=	tech_user_friendly.experienced_user_input.geometry.guess.vert.inboard;
tech.stick.nvtail_outboard	=	tech_user_friendly.experienced_user_input.geometry.guess.vert.outboard;

tech.stick.nhtail_inboard	=	tech_user_friendly.experienced_user_input.geometry.guess.hori.inboard;
tech.stick.nhtail_outboard	=	tech_user_friendly.experienced_user_input.geometry.guess.hori.outboard;
tech.stick.nhtail_carryth	=	tech_user_friendly.user_input.geometry.beam_model.nhtail_carryth;

if aircraft.Wing2.present
    tech.stick.nwing2_inboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing2.inboard;
    tech.stick.nwing2_midboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing2.midboard;
    tech.stick.nwing2_outboard	=	tech_user_friendly.experienced_user_input.geometry.guess.wing2.outboard;
    tech.stick.nwing2_carryth	=	tech_user_friendly.user_input.geometry.beam_model.nwing2_carryth;
end
if aircraft.Canard.present
    tech.stick.ncanard_inboard	=	tech_user_friendly.experienced_user_input.geometry.guess.canard.inboard;
    tech.stick.ncanard_outboard	=	tech_user_friendly.experienced_user_input.geometry.guess.canard.outboard;
    tech.stick.ncanard_carryth	=	tech_user_friendly.user_input.geometry.beam_model.ncanard_carryth;
end
% 
tech.stick.nwing_inboard_coarse   	=	tech_user_friendly.user_input.geometry.beam_model.nwing_inboard;
tech.stick.nwing_midboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nwing_midboard;
tech.stick.nwing_outboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nwing_outboard;
tech.stick.nwing_carryth_coarse	    =	tech_user_friendly.user_input.geometry.beam_model.nwing_carryth;

tech.stick.nfuse_coarse	            =	tech_user_friendly.user_input.geometry.beam_model.nfuse;

tech.stick.nvtail_inboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nvtail_inboard;
tech.stick.nvtail_outboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nvtail_outboard;

tech.stick.nhtail_inboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nhtail_inboard;
tech.stick.nhtail_outboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nhtail_outboard;
tech.stick.nhtail_carryth_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nhtail_carryth;

if aircraft.Wing2.present
    tech.stick.nwing2_inboard_coarse   	=	tech_user_friendly.user_input.geometry.beam_model.nwing2_inboard;
    tech.stick.nwing2_midboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nwing2_midboard;
    tech.stick.nwing2_outboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nwing2_outboard;
    tech.stick.nwing2_carryth_coarse	=	tech_user_friendly.user_input.geometry.beam_model.nwing2_carryth;
end
if aircraft.Canard.present
    tech.stick.ncanard_inboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.ncanard_inboard;
    tech.stick.ncanard_outboard_coarse	=	tech_user_friendly.user_input.geometry.beam_model.ncanard_outboard;
    tech.stick.ncanard_carryth_coarse	=	tech_user_friendly.user_input.geometry.beam_model.ncanard_carryth;
end
% 
tech.stick.nx	=	tech_user_friendly.user_input.geometry.aero_panel.nx;
tech.stick.ny	=	tech_user_friendly.user_input.geometry.aero_panel.ny;
% Add spar location 
tech.spar.W = tech_user_friendly.user_input.geometry.spar_location.Wing1;
tech.spar.HT = tech_user_friendly.user_input.geometry.spar_location.Horizontal_tail;
tech.spar.VT = tech_user_friendly.user_input.geometry.spar_location.Vertical_tail;

if aircraft.Wing2.present
    tech.spar.W2 = tech_user_friendly.user_input.geometry.spar_location.Wing2;
end
if aircraft.Canard.present
    tech.spar.CA = tech_user_friendly.user_input.geometry.spar_location.Canard;
end

%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Define *experienced_user_input* section
tech.wing.tmgw	=	tech_user_friendly.experienced_user_input.material_property.wing.tmgw;
tech.wing.effw	=	tech_user_friendly.experienced_user_input.material_property.wing.effw;
tech.wing.effc	=	tech_user_friendly.experienced_user_input.material_property.wing.effc;
tech.wing.cf	=	tech_user_friendly.experienced_user_input.material_property.wing.cf;

tech.fus.ckf	=	tech_user_friendly.experienced_user_input.material_property.fus.ckf;
tech.fus.ec     =	tech_user_friendly.experienced_user_input.material_property.fus.ec;
tech.fus.kgc	=	tech_user_friendly.experienced_user_input.material_property.fus.kgc;
tech.fus.kgw	=	tech_user_friendly.experienced_user_input.material_property.fus.kgw;
tech.fus.tmg	=	tech_user_friendly.experienced_user_input.material_property.fus.tmg;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PER ORA TAILBOOMS CON LO STESSO MATERIALE DELLA FUSOLIERA!!!!!
if aircraft.Tailbooms.present
    tech.tbooms = tech.fus;
end


tech.vtail.tmgw	=	tech_user_friendly.experienced_user_input.material_property.vtail.tmgw;
tech.htail.tmgw	=	tech_user_friendly.experienced_user_input.material_property.htail.tmgw;

if aircraft.Wing2.present
    tech.wing2.tmgw	=	tech_user_friendly.experienced_user_input.material_property.wing2.tmgw;
%    tech.wing2.effw	=	tech_user_friendly.experienced_user_input.material_property.wing2.effw;
%    tech.wing2.effc	=	tech_user_friendly.experienced_user_input.material_property.wing2.effc;
%    tech.wing2.cf	=	tech_user_friendly.experienced_user_input.material_property.wing2.cf;
end
if aircraft.Canard.present
    tech.canard.tmgw	=	tech_user_friendly.experienced_user_input.material_property.canard.tmgw;
end
% 
tech.stability_der.q	        =	tech_user_friendly.experienced_user_input.loading.aero_data.q;
tech.stability_der.V	        =	tech_user_friendly.experienced_user_input.loading.aero_data.V;
tech.stability_der.dalphas_dnz	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dnz;
tech.stability_der.dalphas_dLt	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dLt;
tech.stability_der.dalphas_dMt	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dMt;

if  aircraft.Canard.present
    tech.stability_der.dalphac_dnz	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphac_dnz;
    tech.stability_der.dalphac_dLc	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphac_dLc;
    tech.stability_der.dalphac_dMc	=	tech_user_friendly.experienced_user_input.loading.flexibility.dalphac_dMc;
end

% 
tech.guess	=	tech_user_friendly.experienced_user_input.geometry.guess;
%--------------------------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PER ORA TAILBOOMS CON LO STESSO NUMERO DI PUNTI DELLA FUSOLIERA!!!!!
if aircraft.Tailbooms.present
    tech.stick.model.tbooms = 1;
    tech.stick.ntbooms = tech.stick.nfuse;
    tech.stick.ntbooms_coarse = tech.stick.nfuse_coarse;
    tech.guess.tbooms = tech.guess.fus;
end