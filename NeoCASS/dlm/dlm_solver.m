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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************
%
% function dlm(geo, interp_model, Mach, k_list, cref, symm, doublet_order, output_name)
%
% - Run Doublet Lattice Method (DLM)
% - Assembly Aerodynamic Influence Coefficients (AIC) 
% - Build aerodynamic transfer matrix (Qhh)
%
% Input:
%
% geo:  Tornado lattice database
% interp_model:  interpolation model with data for structural coupling 
% Mach:          array with flight Mach numbers 
% k_list:        array of reduced frequencies
% cref:          reference chord (cref/2) is used for scaling frequencies
% symm:          required symmetry along vertical xz plane 
%                (0 no symmetry, 1 symmetry, -1 antisymmetry)
% doublet_order: doublet approximation
%                1 or 2 for respectively quadratic or quartic polynomials 
% output_name:   output filename with aerodynamic transfer matrix
%***********************************************************************************************************************

function dlm_model = dlm_solver(geo, interp_model, Mach, k_list, cref, symm, doublet_order, output_name)

global dlm_model;

dlm_model = [];

%***********************************************************************************************************************
% set standard output
%***********************************************************************************************************************
fid = 1;

%***********************************************************************************************************************
% main structure in the code: DLM
%***********************************************************************************************************************
dlm_model = set_struct(fid, Mach, k_list, cref, symm, doublet_order);

%***********************************************************************************************************************
% converts Tornado geometry database into DLM database
%***********************************************************************************************************************
[dlm_model.lattice, dlm_model.ref] = dlm_lattice_setup(fid, geo, symm, dlm_model.aero.cref);
%***********************************************************************************************************************
% set internal parameters
%***********************************************************************************************************************
NMODES = length(interp_model.struct.mode_list);
NMACH  = length(dlm_model.aero.M);
NK     = length(dlm_model.aero.k);

%***********************************************************************************************************************
% assembly AIC matrix
%***********************************************************************************************************************
dlm_model.data.D = assembly_AIC_matrix(fid, dlm_model);

%***********************************************************************************************************************
% boundary condition
%***********************************************************************************************************************
[dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ] = set_boundary_condition(fid, geo, dlm_model, ...
                                                                                                 interp_model, true);

%***********************************************************************************************************************
% solve system and determine Cp for each mode, reduced frequency and Mach number
%***********************************************************************************************************************
dlm_model.data.Cp = solve_system(fid, dlm_model.lattice.np, NK, NMODES, NMACH, dlm_model.data.D, dlm_model.data.dwnwash);

%***********************************************************************************************************************
% build aerodynamic transfer matrix
%***********************************************************************************************************************
dlm_model.data.Qhh = set_generalized_f(fid, NMODES, dlm_model.data.c_displ, NK, NMACH, dlm_model.lattice.area(1:dlm_model.lattice.np), ...
                                       dlm_model.lattice.N(1:dlm_model.lattice.np,:), dlm_model.data.Cp);

%***********************************************************************************************************************
% export aerodynamic transfer matrix
%***********************************************************************************************************************
export_qhh_massa(fid, output_name, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh);
% require FFA matlab toolbox
export_qhh_ffa(fid, output_name, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh, dlm_model.aero.cref);
% close output pointer
fprintf(fid, '\n');



% PROVA

load example/agard_445_6_model.mat
interp=[];
interp.str_set(1).index = [1:121];
interp.struct.Mhh=[];
interp.struct.Khh=[];
interp.str_set(1).param = [1 2, 4, 20, 1.0e+6, 1.0e+6];
interp.struct.modes = structural_modes;
interp.struct.mode_list = [1];
interp.struct.node.coord = structural_nodes;
