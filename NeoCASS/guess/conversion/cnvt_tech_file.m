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

function [tech_user_friendly] = cnvt_tech_file(tech)
% Reads the tech file and write it in user-friendly version

%**************************************************************************************************
% Main substructure to rewrite the tech file. Add here more fields if necessary.
%**************************************************************************************************
% Inputs easily defined by user
tech_user_friendly.user_input = [];
tech_user_friendly.user_input.geometry = [];
tech_user_friendly.user_input.geometry.beam_model = [];
tech_user_friendly.user_input.geometry.aero_panel = [];
tech_user_friendly.user_input.material_property = [];
tech_user_friendly.user_input.loading = [];
tech_user_friendly.user_input.loading.normal_load_factor = [];
tech_user_friendly.user_input.loading.weight_fraction = [];
tech_user_friendly.user_input.loading.aero_data = [];
tech_user_friendly.user_input.loading.maximum_deflection = [];
tech_user_friendly.user_input.analysis_setup = [];
tech_user_friendly.user_input.analysis_setup.lift_distribution = [];
tech_user_friendly.user_input.analysis_setup.pressure_stabilization = [];
tech_user_friendly.user_input.analysis_setup.regression = [];
tech_user_friendly.user_input.analysis_setup.beam_model = [];
tech_user_friendly.user_input.analysis_setup.vlm_calculation = [];
tech_user_friendly.user_input.analysis_setup.torsion_stiffness = [];
% Inputs modified only by experienced user, otherwise not modified
tech_user_friendly.experienced_user_input = [];
tech_user_friendly.experienced_user_input.geometry = [];
tech_user_friendly.experienced_user_input.geometry.guess = [];
tech_user_friendly.experienced_user_input.material_property = [];
tech_user_friendly.experienced_user_input.loading = [];
tech_user_friendly.experienced_user_input.loading.aero_data = [];
tech_user_friendly.experienced_user_input.loading.flexibility = [];
tech_user_friendly.experienced_user_input.analysis_setup = [];
%**************************************************************************************************

%--------------------------------------------------------------------------------------------------
% Define *user_input* section
tech_user_friendly.user_input.material_property.wing.esw  = tech.wing.esw;
tech_user_friendly.user_input.material_property.wing.fcsw = tech.wing.fcsw;
tech_user_friendly.user_input.material_property.wing.dsw  = tech.wing.dsw;
tech_user_friendly.user_input.material_property.wing.kcon = tech.wing.kcon;
tech_user_friendly.user_input.material_property.wing.nstr = tech.wing.nstr;
tech_user_friendly.user_input.material_property.wing.rpitch = tech.wing.rpitch;
tech_user_friendly.user_input.material_property.fus.kcon = tech.fus.kcon;
tech_user_friendly.user_input.material_property.fus.fts  = tech.fus.fts;
tech_user_friendly.user_input.material_property.fus.fcs  = tech.fus.fcs;
tech_user_friendly.user_input.material_property.fus.es   = tech.fus.es;
tech_user_friendly.user_input.material_property.fus.ef   = tech.fus.ef;
tech_user_friendly.user_input.material_property.fus.ds   = tech.fus.ds;
tech_user_friendly.user_input.material_property.fus.df   = tech.fus.df;
tech_user_friendly.user_input.material_property.vtail.kcon = tech.vtail.kcon;
tech_user_friendly.user_input.material_property.vtail.esw  = tech.vtail.esw;
tech_user_friendly.user_input.material_property.vtail.dsw  = tech.vtail.dsw;
tech_user_friendly.user_input.material_property.vtail.fcsw = tech.vtail.fcsw;
tech_user_friendly.user_input.material_property.vtail.nstr = tech.vtail.nstr;
tech_user_friendly.user_input.material_property.vtail.rpitch = tech.vtail.rpitch;
tech_user_friendly.user_input.material_property.htail.kcon = tech.htail.kcon;
tech_user_friendly.user_input.material_property.htail.esw  = tech.htail.esw;
tech_user_friendly.user_input.material_property.htail.dsw  = tech.htail.dsw;
tech_user_friendly.user_input.material_property.htail.fcsw = tech.htail.fcsw;
tech_user_friendly.user_input.material_property.htail.nstr = tech.htail.nstr;
tech_user_friendly.user_input.material_property.htail.rpitch = tech.htail.rpitch;
%
tech_user_friendly.user_input.loading.normal_load_factor = tech.trdata.deslf;
tech_user_friendly.user_input.loading.weight_fraction.cman  = tech.load.cman;
tech_user_friendly.user_input.loading.weight_fraction.cbum  = tech.weight.cbum;
tech_user_friendly.user_input.loading.weight_fraction.clan  = tech.weight.clan;
tech_user_friendly.user_input.loading.aero_data.Cn_dr        = tech.stability_der.Cn_dr;
tech_user_friendly.user_input.loading.aero_data.Cl_dr        = tech.stability_der.Cl_dr;
tech_user_friendly.user_input.loading.aero_data.Cn_dw        = tech.stability_der.Cn_dw;
tech_user_friendly.user_input.loading.aero_data.Cl_dw        = tech.stability_der.Cl_dw;
tech_user_friendly.user_input.loading.aero_data.Cn_b         = tech.stability_der.Cn_b;
tech_user_friendly.user_input.loading.aero_data.Cl_b         = tech.stability_der.Cl_b;
tech_user_friendly.user_input.loading.aero_data.CY_b         = tech.stability_der.CY_b;
tech_user_friendly.user_input.loading.aero_data.CY_dr        = tech.stability_der.CY_dr;
tech_user_friendly.user_input.loading.aero_data.CY_dw        = tech.stability_der.CY_dw;
tech_user_friendly.user_input.loading.aero_data.L_alpha_s    = tech.stability_der.L_alpha_s;
tech_user_friendly.user_input.loading.aero_data.M_alpha_s    = tech.stability_der.M_alpha_s;
tech_user_friendly.user_input.loading.aero_data.L_delta_e    = tech.stability_der.L_delta_e;
tech_user_friendly.user_input.loading.aero_data.M_delta_e    = tech.stability_der.M_delta_e;
tech_user_friendly.user_input.loading.aero_data.Lc           = tech.stability_der.Lc;
tech_user_friendly.user_input.loading.aero_data.Mc           = tech.stability_der.Mc;
tech_user_friendly.user_input.loading.aero_data.dM_025dalpha = tech.stability_der.dM_025dalpha;
tech_user_friendly.user_input.loading.aero_data.CL           = tech.stability_der.CL;
tech_user_friendly.user_input.loading.aero_data.CY_alpha     = tech.stability_der.CY_alpha;
tech_user_friendly.user_input.loading.maximum_deflection = tech.defl_max;
%
tech_user_friendly.user_input.analysis_setup.pressure_stabilization = tech.fact.stab;
tech_user_friendly.user_input.analysis_setup.lift_distribution      = tech.fact.ischrenk;
tech_user_friendly.user_input.analysis_setup.regression.analf       = tech.fact.analf;
tech_user_friendly.user_input.analysis_setup.regression.analw       = tech.fact.analw;
tech_user_friendly.user_input.analysis_setup.regression.analh       = tech.fact.analh;
tech_user_friendly.user_input.analysis_setup.regression.analv       = tech.fact.analv;
tech_user_friendly.user_input.analysis_setup.beam_model             = tech.stick.model;
tech_user_friendly.user_input.analysis_setup.vlm_calculation        = tech.stability_der.ival;
tech_user_friendly.user_input.analysis_setup.torsion_stiffness      = tech.ibredt;
tech_user_friendly.user_input.analysis_setup.optimization_smonoq    = tech.pbarsm1;
%
tech_user_friendly.user_input.geometry.beam_model.nwing_inboard   = tech.stick.nwing_inboard;
tech_user_friendly.user_input.geometry.beam_model.nwing_midboard  = tech.stick.nwing_midboard;
tech_user_friendly.user_input.geometry.beam_model.nwing_outboard  = tech.stick.nwing_outboard;
tech_user_friendly.user_input.geometry.beam_model.nwing_carryth   = tech.stick.nwing_carryth;
tech_user_friendly.user_input.geometry.beam_model.nfuse           = tech.stick.nfuse;
tech_user_friendly.user_input.geometry.beam_model.nvtail_inboard  = tech.stick.nvtail_inboard;
tech_user_friendly.user_input.geometry.beam_model.nvtail_outboard = tech.stick.nvtail_outboard;
tech_user_friendly.user_input.geometry.beam_model.nhtail_inboard  = tech.stick.nhtail_inboard;
tech_user_friendly.user_input.geometry.beam_model.nhtail_outboard = tech.stick.nhtail_outboard;
tech_user_friendly.user_input.geometry.beam_model.nhtail_carryth  = tech.stick.nhtail_carryth;
%
tech_user_friendly.user_input.geometry.aero_panel.nx = tech.stick.nx;
tech_user_friendly.user_input.geometry.aero_panel.ny = tech.stick.ny;
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Define *experienced_user_input* section
tech_user_friendly.experienced_user_input.material_property.wing.tmgw = tech.wing.tmgw;
tech_user_friendly.experienced_user_input.material_property.wing.effw = tech.wing.effw;
tech_user_friendly.experienced_user_input.material_property.wing.effc = tech.wing.effc;
tech_user_friendly.experienced_user_input.material_property.wing.cf   = tech.wing.cf;
tech_user_friendly.experienced_user_input.material_property.fus.ckf = tech.fus.ckf;
tech_user_friendly.experienced_user_input.material_property.fus.ec  = tech.fus.ec;
tech_user_friendly.experienced_user_input.material_property.fus.kgc = tech.fus.kgc;
tech_user_friendly.experienced_user_input.material_property.fus.kgw = tech.fus.kgw;
tech_user_friendly.experienced_user_input.material_property.fus.tmg = tech.fus.tmg;
tech_user_friendly.experienced_user_input.material_property.vtail.tmgw = tech.vtail.tmgw;
tech_user_friendly.experienced_user_input.material_property.htail.tmgw = tech.htail.tmgw;
%
tech_user_friendly.experienced_user_input.loading.aero_data.q = tech.stability_der.q;
tech_user_friendly.experienced_user_input.loading.aero_data.V = tech.stability_der.V;
tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dnz = tech.stability_der.dalphas_dnz;
tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dLt = tech.stability_der.dalphas_dLt;
tech_user_friendly.experienced_user_input.loading.flexibility.dalphas_dMt = tech.stability_der.dalphas_dMt;
%
tech_user_friendly.experienced_user_input.geometry.guess = tech.guess;
%--------------------------------------------------------------------------------------------------

neocass_xmlunwrapper('tech_user_friendly.xml', tech_user_friendly);
