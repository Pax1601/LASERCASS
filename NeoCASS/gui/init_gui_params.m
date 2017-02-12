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
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090312      1.3.4   A. De Gaspari    Modification
%     091025      1.3.7   A. De Gaspari    Modification
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function init_gui_params(filename)
%
function init_gui_params(filename)

solver = [];
guess = [];

% SMARTCAD NUMERICAL SOLVER PARAMETERS
%-------------------------------------------------------------------------------
% maximum number of reduced frequencies
solver.flutter.MAXRFREQ = 12;
% list of reduced frequencies
solver.flutter.RFreq_list = [];
% number of reduced frequencies given
solver.flutter.NRFreq = [];
% list of Mach number
solver.flutter.Mach_list = 0.5;
% number of Mach numbers
solver.flutter.NMach = [];
% maximum velocity for flutter tracking
solver.flutter.VMAX = 340.3;
% velocity points taken in flutter tracking
solver.flutter.NV = 50;
% density for flutter tracking
solver.flutter.RHO = 1.225;
% p-k tracking modes
solver.flutter.FMODES = [];
% Qhh modal base
solver.flutter.MSELECT = [];
% reference chord for reduced frequency definition
solver.aero.CREF = 0.0;
% reference span
solver.aero.BREF = 0.0;
% reference surface
solver.aero.SREF = 0.0;
% vertical symmetry
solver.aero.SXZ = 0;
% horizontal symmetry
solver.aero.SXY = 0;
% height for ground effect
solver.aero.HEIGHT = 0.0;
% Kernel order (1=linear, 2=quadratic)
solver.aero.DLM_ORDER = 2;
% Eig. minimum frequency
solver.eig.MINF = 0.0;
% Eig. maximum frequency
solver.eig.MAXF = 999999;
% Eig. number of roots
solver.eig.NROOTS = [];
% Eig. reference grid point ID
solver.eig.REF_GRID = [];
% Eig. reference grid point coordinate
solver.eig.REF_GRID_C = 1;
% Eig. normalization method
solver.eig.NORM_MET = 1;
% Maximum number of states
solver.aeros.MAXSTATES = 12;
% Control Surface presence
solver.aeros.CS = false;
% Trim labels
solver.aeros.TRIM_LABELS = TRIMlabels;
% State matrix
solver.aeros.STATE_MAT = cell(0,length(solver.aeros.TRIM_LABELS));
% Linear/nonlinear model
solver.param.MODEL_TYPE = 1;
% Number of available states
solver.aeros.NSTATES = 0;
% Assembly and Solver data input file name
solver.file.FILE = '';
solver.file.SOLVER = '';
%
solver.suport = [];
%
solver.EIG_AV = true;
solver.FLUTT_AV = false;
solver.FLUTT_ENV = false;
solver.STATIC_AERO_AV = false;

plot.WAKE_PLOT = 0;
plot.NORM_PLOT = 0;
plot.CONT_PLOT = 0;
plot.SCALE = 1.0;
plot.SET = 1;
plot.NFRAMES = 20;
plot.ROW = 1;
plot.COL = 1;

%
% GUESS MODULE
%-------------------------------------------------------------------------------
guess.file = [];
guess.file.aircraft = '';
guess.file.state = '';
guess.file.trim = '';
guess.file.stick = '';
guess.file.technology = '';
guess.model = [];
guess.model.Joined_wing = false;
guess.model.Strut_wing = false;
guess.model.EnvPoints = [];
guess.model.guessstd = false;
%-------------------------------------------------------------------------------
path = neoguiscratchpath();
%
save(fullfile(path, filename), 'solver', 'plot', 'guess');
