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
%  NAEMO - TORNADO
%  Numerical AeroElastic MOdeller for Fluid-Structure Interaction Analysis with Vortex Lattice aerodynamics
%
%                      Luca Cavagna <cavagna@aero.polimi.it>
%                      Sergio Ricci <ricci@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization of these parties may be persecuted.
%
%***********************************************************************************************************************
% Run test_AGARD.m to check aeroelastic interface algorithm contained in mls_interface.m.
% First four structural modes are plotted.
% The black points displayed are the aerodynamic nodes of a CFD grid.
% The red points displayed are the structural nodes as reported in "AGARD Standard Aeroelastic Configurations for dynamic response, Wing
% AGARD 445.6", by E.C. Yates (1985).
% 
% NOTE: the structural model is represented simply by shell planar elements.
%
%
% Author: Luca Cavagna, DIAPM
%
clear all
close all
fprintf('Loading structural and aerodynamic models...');
load agard_445_6_model.mat;

fprintf('done.');
%
% set modes scale amplitude
scale = [0.1 0.1 0.05 0.05 0.05 0.2];
%scale = [0.08 0.01 0.01 0.01 0.09 0.5];
%
fprintf('\nBuilding interface coefficients...');
 H = mls_interface(structural_nodes(1:121,1:3), aerodynamic_nodes(:,1:3), 2, 2, 2220, 0.3, 1.0e+6);
% Use RBF
%H = rbf_interface(structural_nodes(1:121, 1:3), aerodynamic_nodes(:,1:3), 6, 0.2, 0.0);

fprintf('done.');
%
for (k=1:6)
	figure(k);
	xdef  = H * structural_modes(1:121,1,k) * scale(k);
	ydef  = H * structural_modes(1:121,2,k) * scale(k);
	zdef  = H * structural_modes(1:121,3,k) * scale(k);
	plot3(aerodynamic_nodes(:,1) + xdef, aerodynamic_nodes(:,2) + ydef, aerodynamic_nodes(:,3) + zdef,'.k');
	hold on;
	plot3(structural_nodes(1:121,1) + structural_modes(1:121,1,k) * scale(k), structural_nodes(1:121,2) + structural_modes(1:121,2,k) * scale(k), structural_nodes(1:121,3) + structural_modes(1:121,3,k) * scale(k),'.r');
	axis equal;
	grid;
end

