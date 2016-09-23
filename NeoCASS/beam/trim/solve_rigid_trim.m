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
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
% solver rigid body trim problem
function solve_rigid_trim(varargin)

global beam_model

if beam_model.Param.SOL ~= 144
	error('SOL 144 must be given in input file to run rigid body trim solver.');
end

fid = beam_model.Param.FID;
if nargin == 1
  DUMMY_INDEX = varargin{1};
  if (length(DUMMY_INDEX)>1)
    fprintf(fid, '\nWarning: only the first trim case will be run. The following will be ignored.');
    fprintf(fid, '\n         Rerun the solver.')
  end
  TRIM_INDEX = DUMMY_INDEX(1);
%
else
  TRIM_INDEX = 1;
end

beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
nc = beam_model.Aero.geo.nc;

if ~isempty(index)		

  switch (beam_model.Aero.Trim.Type(index))

  case 0
    NDOF = 5;
    STRIM = 0;
    FMDOF = [];
  case 1 % full simmetric trim
    NDOF = 2;
    STRIM = 1; 
    FMDOF = [1,3,5];
%
  end

% set flight mechanics trim params
beam_model.Aero.Trim.FM = get_free_body_trim_params(beam_model.Aero.Trim.NC(index), ...
  beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
[beam_model.Aero.state.alpha, beam_model.Aero.state.betha, beam_model.Aero.state.P, beam_model.Aero.state.Q, beam_model.Aero.state.R] = ...
   get_state_trim_vars(beam_model.Aero.Trim.FM);
beam_model.Aero.state.alpha = D2R(beam_model.Aero.state.alpha);
beam_model.Aero.state.betha = D2R(beam_model.Aero.state.betha);
% make sure angles are converted
beam_model.Aero.Trim.FM.Value(2) = D2R(beam_model.Aero.Trim.FM.Value(2));
beam_model.Aero.Trim.FM.Value(3) = D2R(beam_model.Aero.Trim.FM.Value(3));
% set Tornado state struct
beam_model.Aero.state.ALT = beam_model.Aero.Trim.ALT(index);
[beam_model.Aero.state.rho, p, T, a, mu] = ISA_h(beam_model.Aero.state.ALT);
beam_model.Aero.state.AS = beam_model.Aero.Trim.Mach(index) * a;
% convert angular velocities to dimensional values
beam_model.Aero.state.P = 2*beam_model.Aero.state.P*beam_model.Aero.state.AS/beam_model.Aero.ref.b_ref;
beam_model.Aero.state.Q = 2*beam_model.Aero.state.Q*beam_model.Aero.state.AS/beam_model.Aero.ref.C_mgc;
beam_model.Aero.state.R = 2*beam_model.Aero.state.R*beam_model.Aero.state.AS/beam_model.Aero.ref.b_ref;
beam_model.Aero.state.Mach(1) = beam_model.Aero.Trim.Mach(index);
%
else
error('Unable to find the required TRIM set %d.', TRIM_INDEX);
end

if nc
beam_model.Aero.Trim.CS = get_control_surf_trim_params(beam_model.Aero.geo.nc, beam_model.Aero.Trim.NC(index), ...
  beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data, beam_model.Aero.lattice.Control);
end
% check trim variables
ncs = sum(beam_model.Aero.Trim.CS.Fixed);
nfm = sum(beam_model.Aero.Trim.FM.Fixed);
if (ncs + nfm) ~= beam_model.Aero.Trim.NC(index)

error('Unable to fix trim variables. Wrong variable name given in TRIM card %d.', ...
  beam_model.Aero.Trim.ID(index));

end

% check for duplicated AELINK labels
if (beam_model.Info.nlink) 

  [labels, i] = unique(beam_model.Aero.Trim.Link.ID);
  if (length(labels) ~= beam_model.Info.nlink)

	  n = [1 : beam_model.Info.nlink];
	  dof = beam_model.Aero.Trim.Link.ID(setdiff(n, i));

	  for k=1:length(dof)

		  fprintf(fid, '\n\tWarning: duplicated labels for AELINK card: %d.', ...
              beam_model.Aero.Trim.Link.ID(dof(k)));

	  end

	  error('AELINK entries have duplicated labels.');

  end
end

beam_model.Aero.Trim.CS.MPC = [];   % store control surfaces contraint equations
beam_model.Aero.Trim.CS.Coeff = []; % store control surfaces contraint coefficients

% set output struct
beam_model.Res = [];
beam_model.Res.SOL = 'Static linear non-linear rigid trim';
beam_model.Res.FM.Value = repmat(beam_model.Aero.Trim.FM.Value,2,1); % current flight mechanics solution
beam_model.Res.FM.Fixed = beam_model.Aero.Trim.FM.Fixed;
beam_model.Res.CS.Value = beam_model.Aero.Trim.CS.Value; % current control surfaces solution
beam_model.Res.CS.Fixed = zeros(1,nc); % set later
beam_model.Res.state = beam_model.Aero.state; % Tornado dummy state (changed during nonlinear solution search)
% set constraints for control surfaces
[beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Fixed] = ...
set_constr_eq(nc, beam_model.Aero.lattice.Control, beam_model.Aero.Trim.CS.Fixed, beam_model.Info.nlink, beam_model.Aero.Trim.Link);
% count variables
NC_TOT = sum(beam_model.Res.FM.Fixed) + sum(beam_model.Res.CS.Fixed);
NTOT = length(beam_model.Res.FM.Fixed) + length(beam_model.Res.CS.Fixed);
% free variables
NF_TOT = NTOT - NC_TOT;
%
if NF_TOT ~= NDOF

	error('Number of degrees of freedom for rigid body trim is different from %d (%d).', NDOF, NF_TOT);

end
%
THRUST_FOUND = false;
% check for THRUST 
if (beam_model.Info.nflw==0) 
	fprintf(fid, '\nWarning: no THRUST card given for engine modelling.');
else
	% look for THRUST cards to be used
	for k = 1:beam_model.Info.nflw
		if beam_model.F_FLW.ID(k) == beam_model.Param.LOAD
			THRUST_FOUND = true; % thrust found then exit loop
			break;
		end
	end
end

%if ~THRUST_FOUND
%	error('No THRUST card loaded or wrong LOAD parameter set.');
%end
fprintf(fid,'\nSolving non-linear rigid body trim analysis...');
%
TOL = zeros(1,2);
PARAM = zeros(1,5);
sol = zeros(NDOF,1);
% overwrite aerodynamic reference point with center of gravity
beam_model.Aero.geo.ref_point = beam_model.WB.CG;
beam_model.Aero.geo.CG = beam_model.WB.CG;
beam_model.Res.Aero.x_hist = [];
beam_model.Res.Aero.resid = [];
% Newton Raphon iterations
EPS    = beam_model.Param.EPS;
TOLL   = beam_model.Param.RES_TOL;
NREL   = beam_model.Param.NITER;
%
for N=1:1

%for N=1:beam_model.Param.NSTEP
  % Actual residual and store solution
  fprintf(fid,'\n\nIter: %4d. ', N);
  RES0 = get_rigid_body_res(sol, true, NDOF);
  NORM = norm(RES0);
  beam_model.Res.Aero.x_hist = [beam_model.Res.Aero.x_hist, sol];
  beam_model.Res.Aero.resid  = [beam_model.Res.Aero.resid, NORM];
  % Check convergence
  if (NORM < TOLL)
    fprintf(fid, '\n Convergence criteria satisfied.');
    break;
  end
  % determine numeric jacobian
  if (N==1 || ~mod(N,NREL)) % modified NR
    fprintf(fid, '\nAssemblying numeric Jacobian...');
    J = zeros(NDOF, NDOF);
    for k = 1:NDOF
      dsol = sol;
      dsol(k) = sol(k) + EPS;
      RES = get_rigid_body_res(dsol, false, NDOF);
      J(:,k) = (RES-RES0)./EPS;
    end
    fprintf(fid, 'done.');
  end
  sol = sol -J\RES0;
end
%
beam_model.Res.Aero.J = J;
fprintf(fid, '\ndone.\n\n');
beam_model.Aero.lattice = [];
%beam_model.Aero.lattice = beam_model.Aero.lattice_defo;
%clear beam_model.Aero.lattice_defo;
%
% convert angular velocities to nondimensional values
beam_model.Res.state.P = beam_model.Res.state.P*beam_model.Aero.ref.b_ref/(2*beam_model.Aero.state.AS);
beam_model.Res.state.Q = beam_model.Res.state.Q*beam_model.Aero.ref.C_mgc/(2*beam_model.Aero.state.AS);
beam_model.Res.state.R = beam_model.Res.state.R*beam_model.Aero.ref.b_ref/(2*beam_model.Aero.state.AS);

end
