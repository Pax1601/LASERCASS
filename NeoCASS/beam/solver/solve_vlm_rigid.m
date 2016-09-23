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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function solve_vlm_rigid(varargin)

global beam_model;

if (~isempty(find(beam_model.Param.MSOL == 700)))
  
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
%
    nc = beam_model.Aero.geo.nc;
%
%   select trim case
%
		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
    dotpos = strfind(beam_model.Param.FILE,'.');
    outf = [beam_model.Param.FILE(1:dotpos-1), '_vlm_', num2str(beam_model.Aero.Trim.ID(index)),'.txt'];
      
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;

		if ~isempty(index)		

	    fprintf(fid,'\nSolving linear static aerodynamic analysis...');
      beam_model.Res = [];
	    beam_model.Res.SOL = 'Static linear aerodynamic';
      beam_model.Res.Aero = [];

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
      beam_model.Aero.state.Mach(1) = beam_model.Aero.Trim.Mach(index);
%    convert angular velocities to dimensional values
      beam_model.Aero.state.P = 2*beam_model.Aero.state.P*beam_model.Aero.state.AS/beam_model.Aero.ref.b_ref;
      beam_model.Aero.state.Q = 2*beam_model.Aero.state.Q*beam_model.Aero.state.AS/beam_model.Aero.ref.C_mgc;
      beam_model.Aero.state.R = 2*beam_model.Aero.state.R*beam_model.Aero.state.AS/beam_model.Aero.ref.b_ref;
      %
      if (beam_model.Param.GRDPNT==0)
        beam_model.Aero.geo.ref_point = zeros(1,3);
        beam_model.Aero.geo.CG = zeros(1,3);
%         beam_model.Aero.geo.ref_point = beam_model.WB.CG;
%         beam_model.Aero.geo.CG        = beam_model.WB.CG;
      else
        beam_model.Aero.geo.ref_point = beam_model.Node.Coord(beam_model.Param.GRDPNT,:);
        beam_model.Aero.geo.CG = beam_model.Node.Coord(beam_model.Param.GRDPNT,:);
      end
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
%***********************************************************************************************************************
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
    beam_model.Res.FM.Fixed = beam_model.Aero.Trim.FM.Fixed;
    beam_model.Res.CS.Value = beam_model.Aero.Trim.CS.Value; % current control surfaces solution
    beam_model.Res.CS.Fixed = zeros(1,nc); % set later
    beam_model.Res.state = beam_model.Aero.state; % Tornado dummy state (changed during nonlinear solution search)
    % set constraints for control surfaces
    if (~isempty(beam_model.Aero.lattice))
      [beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Fixed] = ...
      set_constr_eq(nc, beam_model.Aero.lattice.Control, beam_model.Aero.Trim.CS.Fixed, beam_model.Info.nlink, beam_model.Aero.Trim.Link);
      % apply constraint equation to slave surfaces
      beam_model.Res.CS.Value = apply_constr_eq(nc, beam_model.Res.CS.Fixed, beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Value);

      beam_model.Aero.lattice_defo = rotate_control_surf(beam_model.Aero.ref, beam_model.Res.state, beam_model.Aero.geo, ...
                                                     beam_model.Aero.lattice, beam_model.Res.CS.Value, ...
                                                     beam_model.Aero.lattice.Control.Hinge,1);
    end
    state = beam_model.Res.state;
    fprintf(fid, '\n\n - Aerodynamic state:'); 
    fprintf(fid, '\n\tFlight speed [m/s]: %g.', state.AS); 
    fprintf(fid, '\n\tMach number: %g.', state.Mach(1)); 
    fprintf(fid, '\n\tAltitude [m]: %g.', state.ALT); 
    fprintf(fid, '\n\tDensity [kg/m^3]: %g.', state.rho); 
    fprintf(fid, '\n\tAngle of attack [deg]: %g.', state.alpha*180/pi); 
    fprintf(fid, '\n\tAngle of sideslip [deg]: %g.', state.betha*180/pi); 
    fprintf(fid, '\n\tAngular velocites [rad/s]: [%g, %g, %g].', ...
                         state.P, state.Q, state.R); 
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    if (~isempty(nr))
      for n=1:length(nr)
        fprintf(fid,'\n\tControl %s:   %g [deg].', beam_model.Aero.lattice.Control.Name{nr(n)}, 180*beam_model.Res.CS.Value(nr(n))/pi);
      end
    end
    fprintf(fid, '\n - Aerodynamic solver settings:'); 
    if (state.SIMXZ ~=0)
      fprintf(fid, '\n     Vertical plane symmetry enabled.'); 
    else
      fprintf(fid, '\n     Vertical plane symmetry disabled.'); 
    end
    if (state.SIMXY ~=0)
      fprintf(fid, '\n     Horizontal plane symmetry enabled.'); 
    else
      fprintf(fid, '\n     Horizontal plane symmetry disabled.'); 
    end
    if (state.pgcorr ~=0)
      fprintf(fid, '\n     Prandtl-Glauert correction enabled.'); 
    else
      fprintf(fid, '\n     Prandtl-Glauert correction disabled.'); 
    end
%
    outp = fopen(outf, 'w');
    print_state(outp, beam_model.Aero.state); 
    print_attitude(outp, beam_model.Aero.Trim.FM);
%
%   solve aerodynamic forces
%-------------------------------------------------------------------------------
%   Extract VLM matrices
%
    GAMMA_P = []; GAMMA_I = []; DOFCT = [];
    if ~isempty(beam_model.Aero.lattice)
      [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(beam_model.Aero.geo, beam_model.Aero.lattice_vlm, beam_model.Res.state);
    end
%-------------------------------------------------------------------------------
%   Extract BODY matrices
%
    nbody = beam_model.Info.nbaero;
    Sij = {}; DijY = {}; DijZ = {}; SijV = {}; DijYV = {}; DijZV = {}; GAMMA_HB = {}; GAMMA_VEL = {};
    for i=1:nbody
      [Sij{i}, DijY{i}, DijZ{i}, SijV{i}, DijYV{i}, DijZV{i}, GAMMA_HB{i}, GAMMA_VEL{i}] = ...
         get_body_matrix(i, beam_model.Aero.lattice_vlm, beam_model.Aero.body, beam_model.Res.state, beam_model.Param);
    end
%-------------------------------------------------------------------------------
%  Combine VLM and BODY and determine aero forces
%
    for i=1:nbody
      for k=1:length(beam_model.Aero.body.geo.CAERO_INT{i})
        patch = beam_model.Aero.body.geo.CAERO_INT{i}(k);
        DOFCT = [DOFCT, [beam_model.Aero.lattice_vlm.DOF(patch, 1):beam_model.Aero.lattice_vlm.DOF(patch, 2)]];
      end
    end
    if (beam_model.Param.BCOU == 2)
      for i=1:nbody
        GAMMA_HB{i}(:,DOFCT) = 0.0;
        GAMMA_VEL{i}(:,DOFCT) = 0.0;
      end
    end
%
   [beam_model.Res.Aero.results, GAMMA_MAT, NELEM] = solve_vlm_body(beam_model.Aero.geo, beam_model.Aero.lattice_defo, beam_model.Aero.body, ...
      beam_model.Res.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, DOFCT);
%   store reference load
    results0 = beam_model.Res.Aero.results;
%-------------------------------------------------------------------------------
%
    figure(100); close; figure(100);
    colormap(hot);
    if beam_model.Info.amesh_av_vlm
        fill3(beam_model.Aero.lattice_defo.XYZ(:,:,1)', beam_model.Aero.lattice_defo.XYZ(:,:,2)', ...
            beam_model.Aero.lattice_defo.XYZ(:,:,3)', beam_model.Res.Aero.results.FN');
    title_string = ['Normal force distribution, TRIM card ', num2str(beam_model.Aero.Trim.ID(TRIM_INDEX))];
    title(title_string);
    colorbar('vert');
    axis equal; grid;
    end
    % Run stability derivatives solver
    % store stability derivatives
    CREF = beam_model.Aero.ref.C_mgc;
    BREF = beam_model.Aero.ref.b_ref;
    SREF = beam_model.Aero.ref.S_ref;
    VREF = beam_model.Aero.state.AS;
    print_refvalues(outp, CREF, BREF, SREF, beam_model.Aero.geo.ref_point, beam_model.Aero.geo.CG);
    RHOREF = beam_model.Aero.state.rho;
    QINF = 0.5 * RHOREF * VREF^2;
    QINFS = QINF * SREF;
    LREF = [1.0 1.0 1.0 1/BREF 1/CREF 1/BREF]./QINFS;
    NDIM = diag(LREF);
    beam_model.Res.Aero.RStab_Der = [];
    beam_model.Res.Aero.RIntercept = []; 
%
    EPS = D2R(0.001); % perturbation value to extract aerodynamic derivatives
    AEROMAT = zeros(6, 5+length(nr));
    HINGEMAT = zeros(beam_model.Aero.geo.nc, 5+length(nr));
%   1 case: static rigid load at NULL reference condition
%		fprintf(fid,'\n - Reference rigid case...');
%    dummy_aero = beam_model.Aero; 
%    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
%    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
%    results0 = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
%      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
%		fprintf(fid,'done.');
%
%   2 case: rigid body attitude variations
%   Alpha
		fprintf(fid,'\n - Alpha perturbation...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = dummy_aero.state.alpha + EPS; 
    results = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
    AEROMAT(:,1) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
    if nc
      HINGEMAT(:,1) = (results.HINGE-results0.HINGE)./EPS;
    end
		fprintf(fid,'done.');
%   Beta
		fprintf(fid,'\n - Sideslip perturbation...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.betha = dummy_aero.state.betha + EPS;
    results = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
    AEROMAT(:,2) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
    if nc
      HINGEMAT(:,2) = (results.HINGE-results0.HINGE)./EPS;
    end
		fprintf(fid,'done.');
%   P
		fprintf(fid,'\n - Angular velocities...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.P = dummy_aero.state.P + EPS;
    results = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
    AEROMAT(:,3) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
    if nc
      HINGEMAT(:,3) = (results.HINGE-results0.HINGE)./EPS;
    end
%   Q
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.Q = dummy_aero.state.Q + EPS;
    results = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
    AEROMAT(:,4) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
    if nc
      HINGEMAT(:,4) = (results.HINGE-results0.HINGE)./EPS;
    end
%   R
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.R = dummy_aero.state.R + EPS;
    results = solve_vlm_body_re(dummy_aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
    AEROMAT(:,5) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
    if nc
      HINGEMAT(:,5) = (results.HINGE-results0.HINGE)./EPS;
    end
		fprintf(fid,'done.');
%
%   Control surfaces
%
    dummy_aero = beam_model.Aero; 
%
    if (~isempty(nr))
  		fprintf(fid,'\n - Controls...');
  %
      LUMP_DOF = [];
      LUMP_COEFF = [];
      for k=1:length(nr)
        % erase all rotations
        beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
        % set master rotation
        beam_model.Res.CS.Value(nr(k)) = EPS;
        % look for slave
        cdof = find(beam_model.Aero.Trim.CS.MPC == nr(k));
        LUMP_DOF(k).data = [nr(k)];
        LUMP_COEFF(k).data = [1];
        if (~isempty(cdof))
          beam_model.Res.CS.Value(cdof) = EPS*beam_model.Aero.Trim.CS.Coeff(cdof);
          LUMP_DOF(k).data = [nr(k), cdof];
          LUMP_COEFF(k).data = [1, beam_model.Aero.Trim.CS.Coeff(cdof)];
        end
        lattice_defo = rotate_control_surf(beam_model.Aero.ref, dummy_aero.state, beam_model.Aero.geo, ...
                                                 beam_model.Aero.lattice, beam_model.Res.CS.Value, ...
                                                 beam_model.Aero.lattice.Control.Hinge);
        results = solve_vlm_body_re(dummy_aero.geo, lattice_defo, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
        AEROMAT(:,5+k) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
        HINGEMAT(:,5+k) = (results.HINGE-results0.HINGE)./EPS;
      end
      beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
  		fprintf(fid,'done.');
%     tranform moments to coefficients
      HINGEMAT = HINGEMAT./(QINFS*CREF);
      results0.HINGE = results0.HINGE./(QINFS*CREF);
%     lump rows
      HINGEMAT = lump_hinge(HINGEMAT, LUMP_DOF, LUMP_COEFF);
      results0.HINGE = lump_hinge(results0.HINGE, LUMP_DOF, LUMP_COEFF);
%
    end
%
    AEROMAT = NDIM * AEROMAT;
    beam_model.Res.Aero.RStab_Der = get_stab_der(AEROMAT, HINGEMAT, CREF, BREF, VREF);
    beam_model.Res.Aero.RIntercept = get_aero_intercept(NDIM*[results0.FORCES';results0.MOMENTS(1,:)'], results0.HINGE);
    if (~isempty(nr))
      beam_model.Res.Aero.RStab_Der.Control.Name = beam_model.Aero.lattice.Control.Name(nr);
    end
    %
    print_intercept(outp, beam_model.Res.Aero.RIntercept);
    print_hinge_intercept(outp, beam_model.Res.Aero.RIntercept.cmh0, beam_model.Res.Aero.RStab_Der.Control.Name);
    print_aeroderR(outp, beam_model.Res.Aero.RStab_Der, nr);
%   [FUSD, FUSL] = RecoverFusGeoDataFromBeamModel(beam_model);
    if beam_model.Info.amesh_av_vlm
        beam_model.Res.Aero.RStability = get_stab_marginR(beam_model.Aero.geo.ref_point, beam_model.Aero.geo.CG, ...
            CREF, SREF, [], [], beam_model.Res.Aero.RStab_Der, outp);
    end
    fclose(outp);
%
    fprintf(fid, '\n\ncompleted.\n\n');
    beam_model.Aero.lattice = [];
%
else	
  
  error('SOL 700 must be given in input file to run rigid aerodynamic analysis.');

end		

end
%***********************************************************************************************************************
function Stab_Der = get_stab_der(AEROMAT, HINGEMAT, CREF, BREF, VREF)

  if ~isempty(HINGEMAT)
    nc = 1;
  else
    nc = 0;
  end
  Stab_Der = [];
  Stab_Der.Alpha = [];
  Stab_Der.Beta = [];
  Stab_Der.P_rate = [];
  Stab_Der.Q_rate = [];
  Stab_Der.R_rate = [];
  Stab_Der.Control = [];
%  Stab_Der.Alpha.dcd_dalpha = AEROMAT(1,1);  
  Stab_Der.Alpha.dcs_dalpha = AEROMAT(2,1);  
  Stab_Der.Alpha.dcl_dalpha = AEROMAT(3,1);
  %
  Stab_Der.Alpha.dcml_dalpha = AEROMAT(4,1);  
  Stab_Der.Alpha.dcmm_dalpha = AEROMAT(5,1);  
  Stab_Der.Alpha.dcmn_dalpha = AEROMAT(6,1);
%
  if nc
    Stab_Der.Alpha.dcmh_dalpha = HINGEMAT(:,1);
    Stab_Der.Beta.dcmh_dbeta = HINGEMAT(:,2);
    Stab_Der.P_rate.dcmh_dP = HINGEMAT(:,3)*2*VREF/BREF;
    Stab_Der.Q_rate.dcmh_dQ = HINGEMAT(:,4)*2*VREF/CREF;
    Stab_Der.R_rate.dcmh_dR = HINGEMAT(:,5)*2*VREF/BREF;
  end
  %
  % DELTA BETA
%  Stab_Der.Beta.dcd_dbeta = AEROMAT(1,2);  
  Stab_Der.Beta.dcs_dbeta = AEROMAT(2,2);  
  Stab_Der.Beta.dcl_dbeta = AEROMAT(3,2);  
  %
  Stab_Der.Beta.dcml_dbeta = AEROMAT(4,2);  
  Stab_Der.Beta.dcmm_dbeta = AEROMAT(5,2);
  Stab_Der.Beta.dcmn_dbeta = AEROMAT(6,2);
  %
  % DELTA P rate
%  Stab_Der.P_rate.dcd_dP = AEROMAT(1,3);  
  Stab_Der.P_rate.dcs_dP = AEROMAT(2,3)*2*VREF/BREF;  
  Stab_Der.P_rate.dcl_dP = AEROMAT(3,3)*2*VREF/BREF;  
  %
  Stab_Der.P_rate.dcml_dP = AEROMAT(4,3)*2*VREF/BREF;  
  Stab_Der.P_rate.dcmm_dP = AEROMAT(5,3)*2*VREF/BREF;  
  Stab_Der.P_rate.dcmn_dP = AEROMAT(6,3)*2*VREF/BREF;
  %
  % DELTA Q rate
%  Stab_Der.Q_rate.dcd_dQ = AEROMAT(1,4);  
  Stab_Der.Q_rate.dcs_dQ = AEROMAT(2,4)*2*VREF/CREF;  
  Stab_Der.Q_rate.dcl_dQ = AEROMAT(3,4)*2*VREF/CREF;  
  %
  Stab_Der.Q_rate.dcml_dQ = AEROMAT(4,4)*2*VREF/CREF;  
  Stab_Der.Q_rate.dcmm_dQ = AEROMAT(5,4)*2*VREF/CREF;  
  Stab_Der.Q_rate.dcmn_dQ = AEROMAT(6,4)*2*VREF/CREF;
  %
  % DELTA R rate
%  Stab_Der.R_rate.dcd_dR = AEROMAT(1,5);  
  Stab_Der.R_rate.dcs_dR = AEROMAT(2,5)*2*VREF/BREF;  
  Stab_Der.R_rate.dcl_dR = AEROMAT(3,5)*2*VREF/BREF;  
  %
  Stab_Der.R_rate.dcml_dR = AEROMAT(4,5)*2*VREF/BREF;  
  Stab_Der.R_rate.dcmm_dR = AEROMAT(5,5)*2*VREF/BREF;  
  Stab_Der.R_rate.dcmn_dR = AEROMAT(6,5)*2*VREF/BREF;
  %
  % controls
  nc = 0;
  for n=6:size(AEROMAT,2)
    nc = nc+1;
    Stab_Der.Control.dcs_dDelta(:,nc)  =  AEROMAT(2,n);
    Stab_Der.Control.dcl_dDelta(:,nc)  =  AEROMAT(3,n);
    Stab_Der.Control.dcml_dDelta(:,nc) =  AEROMAT(4,n);
    Stab_Der.Control.dcmm_dDelta(:,nc) =  AEROMAT(5,n);
    Stab_Der.Control.dcmn_dDelta(:,nc) =  AEROMAT(6,n);
    Stab_Der.Control.dcmh_dDelta(:,nc) =  HINGEMAT(:,n);
  end

end
