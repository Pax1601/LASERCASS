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
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
function solve_free_lin_trim_aa(varargin)

global beam_model;
%
fid = beam_model.Param.FID; 
%
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

LOAD_SCALE = 1.0;
EPS = D2R(0.001); % perturbation value to extract aerodynamic derivatives

if (~isempty(find(beam_model.Param.MSOL == 144)))

		fprintf(fid,'\nSolving linear static trim in attached axes...\n\n');
%
    nc = beam_model.Aero.geo.nc;
%
%   select trim case
%
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
    LOADI = beam_model.Aero.Trim.ID(index);    
%
		if ~isempty(index)  
      dotpos = strfind(beam_model.Param.FILE,'.');		
      outf = [beam_model.Param.FILE(1:dotpos-1), '_man_', num2str(beam_model.Aero.Trim.ID(index)),'.txt'];
%
%     This option is not considered here. Specify consistent problem through SUPORT
%
%      switch (beam_model.Aero.Trim.Type(index))
%
%      case 0
%        NDOF = 6;
%        STRIM = 0;
%        FMDOF = [];
%      case 1 % full simmetric trim
%        NDOF = 3;
%        STRIM = 1; 
%        FMDOF = [1,3,5];
%      case 2 % pitch spin
%        NDOF = 1;
%        STRIM = 1; 
%        FMDOF = [5];
%      case -1 % full anti-simmetric trim
%        NDOF = 3;
%        STRIM = 1; 
%        FMDOF = [2,4,6];
%      case -2 % roll spin
%        NDOF = 1;
%        STRIM = 1;  
%        FMDOF = [4];
%      case -3 % yaw spin
%        NDOF = 1;
%        STRIM = 1;  
%        FMDOF = [6];
%
%      end
%
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
    beam_model.Res = []; 
    beam_model.Res.SOL = 'Static linear unrestrained trim';
    beam_model.Res.FM.Value = repmat(beam_model.Aero.Trim.FM.Value,2,1); % current flight mechanics solution
    beam_model.Res.FM.Fixed = beam_model.Aero.Trim.FM.Fixed;
    beam_model.Res.CS.Value = beam_model.Aero.Trim.CS.Value; % current control surfaces solution
    beam_model.Res.CS.Fixed = zeros(1,nc); % set later
    beam_model.Res.state = beam_model.Aero.state; % Tornado dummy state (changed during nonlinear solution search)
    % set constraints for control surfaces
    [beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Fixed] = ...
    set_constr_eq(nc, beam_model.Aero.lattice.Control, beam_model.Aero.Trim.CS.Fixed, beam_model.Info.nlink, beam_model.Aero.Trim.Link);
%-------------------------------------------------------------------------------
%   Check for SUPORTED node
%
    SUP_INDEX = [];
    SUP_NODE = -1;
    if (~isempty(beam_model.Param.SUPORT))
      SUP_NODE = beam_model.Param.SUPORT(1);
      s_dof = num2str(beam_model.Param.SUPORT(1,2));
      fprintf(fid, '\n - SUPORTED node: %d.', SUP_NODE);
      fprintf(fid, '\n - SUPORTED dofs:');
      for k=1:length(s_dof)
        SUP_INDEX = [SUP_INDEX, str2num(s_dof(k))];
        fprintf(fid, ' %d', SUP_INDEX(k)); 
      end
      fprintf(fid, '.');
    else 
      fprintf('Warning: no SUPORT card given.');
    end
%-------------------------------------------------------------------------------
%   Check for SPC constraints and constrain relative fm dofs
%   
    if (beam_model.Info.cc_nspc>0)
      fprintf(fid, '\n - SPC card found...');
      spcindex = find(beam_model.SPC.ID == beam_model.Param.SPC);
      for k=1:length(spcindex)
        spc_set = spcindex(k);
        nspc_nodes = length(beam_model.SPC.Nodes(spc_set).list);
        for i=1:nspc_nodes
          if (beam_model.Node.ID(beam_model.SPC.Nodes(spc_set).list(i)) == SUP_NODE)
            for (m=1:length(beam_model.SPC.DOF(spc_set).list))
  %
              if (~isempty(find(beam_model.SPC.DOF(spc_set).list(m)==SUP_INDEX)))
                error('DOF %d included in both SPC and SUPORT cards.', beam_model.SPC.DOF(spc_set).list(m));
              end
              switch (beam_model.SPC.DOF(spc_set).list(m))
                case 1
                    beam_model.Res.FM.Fixed(7) = true; % ux
                case 2
                    beam_model.Res.FM.Fixed(8) = true; % uy
                case 3
                    beam_model.Res.FM.Fixed(9) = true; % uz
                case 4
                    beam_model.Res.FM.Fixed(4) = true; % p
                    beam_model.Res.FM.Fixed(10) = true;% pdot
                case 5
                    beam_model.Res.FM.Fixed(5)  = true;% q
                    beam_model.Res.FM.Fixed(11) = true;% qdot
                case 6
                    beam_model.Res.FM.Fixed(6)  = true;% r
                    beam_model.Res.FM.Fixed(12) = true;% rdot
              end
              break;
            end
          end
        end
      end
    end  
%-------------------------------------------------------------------------------
%   Initialize data
%
    NC_TOT = sum(beam_model.Res.FM.Fixed) + sum(beam_model.Res.CS.Fixed);
    NTOT = length(beam_model.Res.FM.Fixed) + length(beam_model.Res.CS.Fixed);
    NF_TOT = NTOT - NC_TOT;
    NDOF = length(SUP_INDEX);
    if ~isempty(SUP_INDEX)
      % free variables
      fprintf(fid, '\n - Total number of dofs: %d.', NTOT);
      fprintf(fid, '\n - Total number of fixed dofs: %d.', NC_TOT);
      fprintf(fid, '\n - Free dofs: %d.', NF_TOT);
      fprintf(fid, '\n - Number of trim equations: %d.', NDOF);
      %
      if NF_TOT ~= NDOF
        if NF_TOT<NDOF
          error('Trim problem not consistent.');
        else
          fprintf(fid, '\n - Non linear trim solution...');
        end
      end
    end
    %
    outp = fopen(outf, 'w');
    print_state(outp, beam_model.Aero.state); 
    print_attitude(outp, beam_model.Res.FM);
    print_trim_problem(outp, NF_TOT, NDOF);
		ngrid = beam_model.Info.ngrid;
		nbar  = beam_model.Info.nbar;
		nbeam = beam_model.Info.nbeam;
		ndof  = beam_model.Info.ndof;
    nbody = beam_model.Info.nbaero;
    npa = length(beam_model.Aero.lattice_vlm.COLLOC);
    SPLINE_TYPE = beam_model.Info.spline_type;
		% create intermediate storage struct
		fprintf(fid, '\n - Setting internal database...');
		% store bar internal forces
		beam_model.Res.Bar.CForces = zeros(2, 6, nbar);
		beam_model.Res.Beam.CForces = zeros(2, 6, nbeam);
		% store bar internal strains and curvatures
		beam_model.Res.Bar.CStrains = zeros(2, 6, nbar);
		beam_model.Res.Beam.CStrains = zeros(2, 6, nbeam);
		% store bar stresses
		beam_model.Res.Bar.CStresses = zeros(2, 4, nbar);
		beam_model.Res.Beam.CStresses = zeros(2, 4, nbeam);
    beam_model.Res.Bar.CSM = [];
    beam_model.Res.Beam.CSM = [];
		% store nodal displacement
		beam_model.Res.NDispl = zeros(ngrid, 6);
		NODEPOS = beam_model.Node.Coord;
		% store updated bar rotations
		beam_model.Res.Bar.R = beam_model.Bar.R;
		beam_model.Res.Bar.Colloc = beam_model.Bar.Colloc;
		% store updated beam rotations
		beam_model.Res.Beam.R = beam_model.Beam.R;
		beam_model.Res.Beam.Colloc = beam_model.Beam.Colloc;
		% store updated node rotation
		beam_model.Res.NRd = beam_model.Node.R;
		fprintf(fid, 'done.');
%-------------------------------------------------------------------------------
%   Create matrix data
%
%   run aeroelastic interpolation
    aeroelastic_interface;
    % stiffness matrix   
		fprintf(fid, '\n- Assemblying stiffness matrix...');
         
		[K] = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam, beam_model.Celas);
		 
        fprintf(fid, 'done.');
    % assembly mass matrix 
		fprintf(fid, '\n- Assemblying mass matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
		fprintf(fid, 'done.'); 
    if (SPLINE_TYPE == 1)
  		fprintf(fid, '\n- Assemblying linear aerodynamic influence matrix...');
      [Qaa, CPaeroDef, GAMMA_P] = st_linaer_matrix(beam_model.Aero.Interp.Ic, beam_model.Aero.Interp.Imv, beam_model.Aero.geo, beam_model.Aero.lattice, beam_model.Aero.state);
		  fprintf(fid, 'done.');
    else
      GAMMA_P = [];
      CPaeroDef = zeros(npa*3,ndof);
    end

%   RHS 
    if ~isempty(beam_model.RBE2.ID)
        K = RBE2Assembly(beam_model.RBE2,K);
        M = RBE2Assembly(beam_model.RBE2,M);
    end
        fprintf(fid,'\n- Setting system rhs...');
    %      
    beam_model.Res.Aero = [];                         
		% set generalized forces 
    fprintf(fid, '\n     External forces...');
		F = gf_lin_nodal(LOADI, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
                         beam_model.Node.DOF, LOAD_SCALE);
    fprintf(fid, 'done.');
%   initialize these data for constrained cases
    ldof = [1:size(M,1)];
    rdof = [];
    D = []; 
    Mll = M; Kll = K;
%   Stiffness matrix
    if (~isempty(beam_model.Param.SUPORT))
      fprintf(fid,'\n- Setting SUPORT matrix...');
      dummy = beam_model.Node;
      if ~isempty(beam_model.RBE2.ID)
      dummy.DOF = dummy.DOF2;
      end
      DSUP = beam_model.Param.SUPORT;
%     DSUP(1,2) = 123456;
      [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, dummy, DSUP, beam_model.Param.EPS);
      nr = find(KEPS < beam_model.Param.EPS);
      if (~isempty(nr))
        fprintf(fid, '\nWarning: %d SUPORT rigid modes exceed deformation energy tolerance %g.', length(nr), beam_model.Param.EPS);
      end
      fprintf(fid, 'done.');
%     Mass matrix
      Mll = M(ldof, ldof); Mlr = M(ldof, rdof); Mrr = M(rdof, rdof); Mrl = M(rdof, ldof); 
%     Rigid body mass matrix for the SUPORTED DOFs 
      mr = Mrr + Mrl*D + D'*Mlr + D'*Mll*D;   
      MSRR = mr; 
    end
%-------------------------------------------------------------------------------
%   Aerodynamic influence matrix
%
		fprintf(fid,'\nGenerating aerodynamic database...');
    if (~isempty(SUP_INDEX))
      sindex = find(beam_model.Param.SUPORT(1,1) == beam_model.Node.ID);
      if (~isempty(sindex))
%       set for moments calculations
        beam_model.Aero.geo.ref_point = beam_model.Node.Coord(sindex,:);
%       set for angular velocities
        beam_model.Aero.geo.CG = beam_model.Node.Coord(sindex,:);
      end
    end
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
    beam_model.Res.Aero.DStab_Der = []; 
    beam_model.Res.Aero.DIntercept = [];
    beam_model.Res.Aero.RTrim_sol = [];
    beam_model.Res.Aero.DTrim_sol = [];
%
%   1 case: static rigid load at NULL reference condition
		fprintf(fid,'\n - Reference rigid case...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
%-------------------------------------------------------------------------------
%     Extract VLM matrices
%
    GAMMA_I = []; DOFCT = [];
    if ~isempty(beam_model.Aero.lattice)
      if (SPLINE_TYPE==1)
        [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(dummy_aero.geo, dummy_aero.lattice, dummy_aero.state, GAMMA_P);
      else
        [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(dummy_aero.geo, dummy_aero.lattice, dummy_aero.state);
      end
    end
%-------------------------------------------------------------------------------
%     Extract BODY matrices
%
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
    [results0, GAMMA_MAT, NELEM] = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, DOFCT);
    Fa0 = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results0);
%   Total external forces (Applied loads + Follower + reference rigid aero condition)
    F = F + F_flw + Fa0; 
		fprintf(fid,'done.'); 
    if ~isempty(beam_model.RBE2.ID)
       F = RBE2Assembly2(beam_model.RBE2,F);
    end   
%
    Fa_State_aero = zeros(npa*3, 5);
%
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    HINGEMAT = zeros(beam_model.Aero.geo.nc, 5+length(nr));
%
%   2 case: rigid body attitude variations
%   Alpha
		fprintf(fid,'\n - Alpha perturbation...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = EPS; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
    Fa_ALPHA = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results);
    Fa_ALPHA = (Fa_ALPHA - Fa0)./ EPS; % get force variation
    Fa_State_aero([1:3:end], 1) = [results.F(:,1)-results0.F(:,1)]./ EPS;
    Fa_State_aero([2:3:end], 1) = [results.F(:,2)-results0.F(:,2)]./ EPS;
    Fa_State_aero([3:3:end], 1) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    Fa_ALPHAl = Fa_ALPHA(ldof,1); Fa_ALPHAr = Fa_ALPHA(rdof,1);
    if nc
      HINGEMAT(:,1) = (results.HINGE-results0.HINGE)./EPS;
    end
		fprintf(fid,'done.');
%   Beta
		fprintf(fid,'\n - Sideslip perturbation...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = EPS;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
    Fa_BETA = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        results);
    Fa_BETA = (Fa_BETA - Fa0) ./ EPS;
    Fa_State_aero([1:3:end], 2) = [results.F(:,1)-results0.F(:,1)]./ EPS;
    Fa_State_aero([2:3:end], 2) = [results.F(:,2)-results0.F(:,2)]./ EPS;
    Fa_State_aero([3:3:end], 2) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    Fa_BETAl = Fa_BETA(ldof,1); Fa_BETAr = Fa_BETA(rdof,1);
    if nc
      HINGEMAT(:,2) = (results.HINGE-results0.HINGE)./EPS;
    end
		fprintf(fid,'done.');
%   P
		fprintf(fid,'\n - Angular velocities...');
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = EPS;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
    Fa_P = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        results);
    Fa_P = (Fa_P - Fa0) ./ EPS;
    if nc
      HINGEMAT(:,3) = (2*VREF/BREF).*(results.HINGE-results0.HINGE)./EPS;
    end
%   set to adimensional angular speed
    Fa_P = (2*VREF)*Fa_P ./ BREF;
    Fa_State_aero([1:3:end], 3) = [results.F(:,1)-results0.F(:,1)]./ EPS;
    Fa_State_aero([2:3:end], 3) = [results.F(:,2)-results0.F(:,2)]./ EPS;
    Fa_State_aero([3:3:end], 3) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    Fa_Pl = Fa_P(ldof,1); Fa_Pr = Fa_P(rdof,1);
%   Q
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = EPS;      dummy_aero.state.R = 0;
    results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
    Fa_Q = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        results);
    Fa_Q = (Fa_Q - Fa0) ./ EPS;
    if nc
      HINGEMAT(:,4) = (2*VREF/CREF).*(results.HINGE-results0.HINGE)./EPS;
    end
%   set to adimensional angular speed
    Fa_Q = (2*VREF)*Fa_Q ./ CREF;
    Fa_State_aero([1:3:end], 4) = [results.F(:,1)-results0.F(:,1)]./ EPS;
    Fa_State_aero([2:3:end], 4) = [results.F(:,2)-results0.F(:,2)]./ EPS;
    Fa_State_aero([3:3:end], 4) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    Fa_Ql = Fa_Q(ldof,1); Fa_Qr = Fa_Q(rdof,1);
%   R
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = EPS;
    results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
    Fa_R = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        results);
    Fa_R = (Fa_R - Fa0) ./ EPS;
    if nc
      HINGEMAT(:,5) = (2*VREF/BREF).*(results.HINGE-results0.HINGE)./EPS;
    end
    %   set to adimensional angular speed
    Fa_R = (2*VREF)*Fa_R ./ BREF;
    Fa_State_aero([1:3:end], 5) = [results.F(:,1)-results0.F(:,1)]./ EPS;
    Fa_State_aero([2:3:end], 5) = [results.F(:,2)-results0.F(:,2)]./ EPS;
    Fa_State_aero([3:3:end], 5) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    Fa_Rl = Fa_R(ldof,1); Fa_Rr = Fa_R(rdof,1);
		fprintf(fid,'done.');
%
%   Control surfaces
%
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    FCl = [];
    FCr = [];
%
    if (~isempty(nr))
  		fprintf(fid,'\n - Controls...');
      print_controls(outp, nr, beam_model.Aero.lattice_vlm.Control.Name, beam_model.Res.CS);
      FCtot = zeros(ndof, length(nr));
      FCtot_aero = zeros(npa*3, length(nr));
      FCl = zeros(length(ldof), length(nr));
      FCr = zeros(length(rdof), length(nr)); 
  %
      LUMP_DOF = [];
      LUMP_COEFF = [];
  %
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
                                                 beam_model.Aero.lattice.Control.Hinge, 1);

        results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
        Fa_C = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
            results);
        Fa_C = (Fa_C - Fa0) ./ EPS;
        FCtot_aero([1:3:end], k) = [results.F(:,1)-results0.F(:,1)]./ EPS;
        FCtot_aero([2:3:end], k) = [results.F(:,2)-results0.F(:,2)]./ EPS;
        FCtot_aero([3:3:end], k) = [results.F(:,3)-results0.F(:,3)]./ EPS;
        FCl(:,k) = Fa_C(ldof,1);     
        FCr(:,k) = Fa_C(rdof,1);     
        FCtot(:,k) = Fa_C;
        HINGEMAT(:,5+k) = (results.HINGE-results0.HINGE)./EPS;
  %
      end
      beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
%     tranform moments to coefficients
      HINGEMAT = HINGEMAT./(QINFS*CREF);
      results0.HINGE = results0.HINGE./(QINFS*CREF);
%     lump rows
      HINGEMAT = lump_hinge(HINGEMAT, LUMP_DOF, LUMP_COEFF);
      results0.HINGE = lump_hinge(results0.HINGE, LUMP_DOF, LUMP_COEFF);
  		fprintf(fid,'done.');
    end

%   Assembly derivatives 
    Kax  =  [Fa_ALPHA, Fa_BETA, Fa_P, Fa_Q, Fa_R, FCtot];
    KaxDOF = Kax;
    if ~isempty(beam_model.RBE2.ID) 
        Kax = RBE2Assembly2(beam_model.RBE2,Kax);
    end
%     Kax(beam_model.Node.DOF((45:55),2),9) = Kax(beam_model.Node.DOF((45:55),2),9)*0; 
%     Kaxl =  -[Fa_ALPHAl, Fa_BETAl, Fa_Pl, Fa_Ql, Fa_Rl, FCl];  
    Kaxl = Kax(ldof,:); 
    Kaxr = Kax(rdof,:);
    
%     Kaxr =  -[Fa_ALPHAr, Fa_BETAr, Fa_Pr, Fa_Qr, Fa_Rr, FCr]; 
%***********************************************************************************************************************
%
%   Assembly deformability influence coefficients 
%
    if (SPLINE_TYPE > 1)
		  fprintf(fid,'\n - Deformability influence coefficients...');
      Qaa = zeros(ndof, ndof); 
      dummy_aero = beam_model.Aero;  
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
  %   Get master nodes connected to aerodynamics
      MNODE = [];
      AERO_DOF = [];
      for n = 1:ngrid
        if (~isempty(beam_model.Node.Aero.Index(n).data))
          MNODE = [MNODE, n];
        end
      end
  %
      CPaeroDef = zeros(size(beam_model.Aero.lattice_vlm.COLLOC,1),3,ndof);

      for m = 1:length(MNODE)
        n = MNODE(m);
        % DISPLACEMENT DOF
        for k=1:3
          % loop on node displacements DOFs
          if (beam_model.Node.DOF(n,k))
            AERO_DOF = [AERO_DOF, beam_model.Node.DOF(n,k)];
            NODEPOS = beam_model.Node.Coord;
            NODEPOS(n,k) = beam_model.Node.Coord(n,k) + EPS;  
  %         determine updated lattice
	          lattice_defo = update_vlm_mesh(beam_model.Node, NODEPOS, beam_model.Node.R, dummy_aero);
            results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
              dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT); 
            Fa_D = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results);
            CPaeroDef([1:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,1)-results0.F(:,1)]./ EPS;
            CPaeroDef([2:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,2)-results0.F(:,2)]./ EPS;
            CPaeroDef([3:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,3)-results0.F(:,3)]./ EPS;
            Qaa(:,beam_model.Node.DOF(n, k)) = Fa_D ./ EPS; 
          end
        end
  %
        for k=4:6
          % erase displacements
          % erase rotations
          beam_model.Res.NRd = beam_model.Node.R;
          % loop on node displacements DOFs
          if (beam_model.Node.DOF(n,k))
            AERO_DOF = [AERO_DOF, beam_model.Node.DOF(n,k)];
            dummyrot = zeros(1,3);
            dummyrot(k-3) = EPS;
			      beam_model.Res.NRd(:,:,n) = Rmat(dummyrot);
  %         determine updated lattice
	          lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord, beam_model.Res.NRd, dummy_aero);
            results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
              dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT); 
            Fa_D = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results);
            CPaeroDef([1:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,1)-results0.F(:,1)]./ EPS;
            CPaeroDef([2:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,2)-results0.F(:,2)]./ EPS;
            CPaeroDef([3:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,3)-results0.F(:,3)]./ EPS;
           Qaa(:,beam_model.Node.DOF(n, k)) = Fa_D ./ EPS; 
          end
        end
      end
  		fprintf(fid,'done.');
    end
%
		fprintf(fid,'\ndone.');

    beam_model.Res.NDispl = zeros(ngrid, 6);
		beam_model.Res.NRd = beam_model.Node.R;
%    
    if ~isempty(beam_model.RBE2.ID)
        Qaa = RBE2Assembly(beam_model.RBE2,Qaa);
    end
    Qaall = Qaa(ldof, ldof);
    Qaalr = Qaa(ldof, rdof);
    Qaarl = Qaa(rdof, ldof);
    Qaarr = Qaa(rdof, rdof);
%-------------------------------------------------------------------------------
% TRIM SOLUTION
%  
  nc = find(beam_model.Aero.Trim.CS.MPC == 0); % get master rotations
  UX = zeros(length(nc)+5,2);
%      
  if (~isempty(SUP_INDEX)) % trim required
%
    NDIM = NDIM(SUP_INDEX,SUP_INDEX);
    for ntrim = 1:2

      if ntrim == 1 % switch off deformability effects on aerodynamics
	  	  fprintf(fid,'\nSolving rigid aircraft trim condition...');
        Kall = Kll;
        Kalr = Klr;
        Karl = Krl;
        Karr = Krr;
        invKall = inv(Kall);
        if (beam_model.Param.DIVERG > 0)
          KDIV = Kll;
        end
      else
	  	  fprintf(fid,'\nSolving deformable aircraft trim condition...');
        Kall = Kll - Qaall;
        Kalr = Klr - Qaalr;
        Karl = Krl - Qaarl;
        Karr = Krr - Qaarr; 
        invKall = inv(Kall);
      end
%
      DUMMY = (D' * Kall + Karl) * invKall;
      MSRRAER = - DUMMY * (Mll * D + Mlr);
      TMAT_ACC = MSRR + MSRRAER; 
      IPZF2 = (D' - DUMMY) * F(ldof,1) + F(rdof,1);
      IPZF  = IPZF2;
%
      Z1ZX = (D' - DUMMY) * Kaxl + Kaxr;
%-------------------------------------------------------------------------------
%     Determine trim unknows
%
%     set acceleration dofs
      acc_dof = beam_model.Res.FM.Fixed(SUP_INDEX+6);

%      acc_dof = beam_model.Res.FM.Fixed(7:12);
      indexa = find(acc_dof); % fixed acc dofs
%      UDD(:,ntrim) = beam_model.Aero.Trim.FM.Value(7:12)';
      UDD(:,ntrim) = zeros(6,1);
      UDD(SUP_INDEX,ntrim) = beam_model.Aero.Trim.FM.Value(SUP_INDEX+6)';
 
      index_acc = setdiff([1:length(SUP_INDEX)], indexa); % free acc dofs 
      if ~isempty(indexa)
        IPZF = IPZF - MSRR(:, indexa) * UDD(SUP_INDEX(indexa),ntrim) - MSRRAER(:, indexa) * UDD(SUP_INDEX(indexa),ntrim);
        TMAT_ACC = MSRR(:, index_acc) + MSRRAER(:, index_acc); % clean for constrained acc
      end
%
      Z1ZX_FM = Z1ZX(:,1:5);
      fm_dof = beam_model.Res.FM.Fixed(2:6);
      UX(1:5,ntrim) = beam_model.Aero.Trim.FM.Value(2:6)';
      indexfm = find(fm_dof); % fixed fm dofs
      index_fm = setdiff([1:5], indexfm); % free fm dofs
      if ~isempty(indexfm)
        IPZF = IPZF + Z1ZX_FM(:, indexfm) * UX(indexfm,ntrim);
        Z1ZX_FM = Z1ZX_FM(:, index_fm); % clean for constrained fm param
      end
%
      Z1ZX_CS = [];
      index_cs = [];
      if (~isempty(nc))
        Z1ZX_CS = Z1ZX(:,6:end);
        cs_dof = beam_model.Aero.Trim.CS.Fixed(nc); % check if master deflections are fixed
        defl = beam_model.Aero.Trim.CS.Value(nc)'; 
        UX(6:end,ntrim) = defl;
        indexcs = find(cs_dof); % fixed cs dofs

        index_cs = setdiff([1:length(nc)], indexcs); % free cs dofs
        if (~isempty(indexcs))
          UCS(:,ntrim) = defl(indexcs);
          IPZF = IPZF + Z1ZX_CS(:, indexcs) * UCS(:,ntrim);
          Z1ZX_CS = Z1ZX_CS(:, index_cs); % clean for constrained fm param
        end
        free_cs = intersect(nc,  find(beam_model.Aero.Trim.CS.Fixed==0));
        NAME = beam_model.Aero.lattice.Control.Name(nc);
      end
%
      TMAT = [TMAT_ACC, -Z1ZX_FM, -Z1ZX_CS]; 

%
      if NF_TOT ~= NDOF
        funmin = @(x) handlemin(x,index_acc,index_fm,index_cs);
%         funcon = @(x) handlecon(x,TMAT,IPZF);
        options = optimset('Algorithm','active-set','TolCon', 1e-6);
        sol = fmincon(funmin, zeros(size(TMAT,2),1), [] , [] ,full(TMAT) , IPZF, [], [], [], options);
      else
        sol = inv(TMAT) * IPZF;
      end
      sol_offset = 0;
%   recover solution acceleration
%
      if (~isempty(index_acc))
        UDD(SUP_INDEX(index_acc),ntrim) = sol([1:length(index_acc)]);
        sol_offset = sol_offset + length(index_acc);
        beam_model.Res.FM.Value(ntrim, 7:12) = UDD(:,ntrim)';
      end
%   recover solution fm
      if (~isempty(index_fm))
        UX(index_fm,ntrim) = sol([sol_offset+1:sol_offset+length(index_fm)]);
        sol_offset = sol_offset + length(index_fm);
        beam_model.Res.FM.Value(ntrim, 2:6) = UX(1:5,ntrim)';
      end
%   recover solution cs
      beam_model.Res.CS.Value(ntrim, :) = beam_model.Aero.Trim.CS.Value;
      if (~isempty(index_cs))
        UX(index_cs+5,ntrim) = sol([sol_offset+1:sol_offset+length(index_cs)]);
        sol_offset = sol_offset + length(index_cs);
        beam_model.Res.CS.Value(ntrim, free_cs) = UX(index_cs+5,ntrim);
      end

  % store stability derivatives 
    FMDER = NDIM * Z1ZX;
    MSRRAERDER = NDIM * MSRRAER;
%  MSRRAERDER(:,1:3) = MSRRAERDER(:,1:3) * beam_model.Param.G;
%  MSRRAERDER(:,[4,6]) = MSRRAERDER(:,[4,6]) * 2 * beam_model.Param.G / BREF;
%  MSRRAERDER(:,5) = MSRRAERDER(:,5) * 2 * beam_model.Param.G / CREF;
%
    FM0 = NDIM * IPZF2;  
    if ntrim==1
      beam_model.Res.Aero.RStab_Der = get_stab_der_aa(SUP_INDEX, FMDER, MSRRAERDER, HINGEMAT); 
      beam_model.Res.Aero.RIntercept = get_aero_intercept_aa(SUP_INDEX, FM0, results0.HINGE);
      beam_model.Res.Aero.RStab_Der.Control.Name = {};
      if (~isempty(nr))
        beam_model.Res.Aero.RStab_Der.Control.Name = beam_model.Aero.lattice_vlm.Control.Name(nr);
      end
      beam_model.Res.Aero.RTrim_sol = store_trim_sol(fid, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
      fprintf(outp, '\n\nRIGID TRIM RESULTS\n');
      dummy = store_trim_sol(outp, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
      print_intercept_aa(outp, SUP_INDEX, beam_model.Res.Aero.RIntercept);
      print_hinge_intercept(outp, beam_model.Res.Aero.RIntercept.cmh0, beam_model.Res.Aero.RStab_Der.Control.Name);
    else
      beam_model.Res.Aero.DStab_Der = get_stab_der_aa(SUP_INDEX, FMDER, MSRRAERDER, []);
      beam_model.Res.Aero.DIntercept = get_aero_intercept_aa(SUP_INDEX, FM0, []);
      beam_model.Res.Aero.DStab_Der.Control.Name = {};
      if (~isempty(nr))
        beam_model.Res.Aero.DStab_Der.Control.Name = beam_model.Aero.lattice_vlm.Control.Name(nr);
      end 
      beam_model.Res.Aero.DTrim_sol = store_trim_sol(fid, UDD(:,2), UX(:,2), beam_model.Res.Aero.DStab_Der.Control.Name);
      fprintf(outp, '\n\nELASTIC TRIM RESULTS\n');
      dummy = store_trim_sol(outp, UDD(:,2), UX(:,2), beam_model.Res.Aero.DStab_Der.Control.Name);
      print_intercept_aa(outp, SUP_INDEX, beam_model.Res.Aero.DIntercept);
    end 
    fprintf(fid,'\ndone.');
%
%
  end % trim loop%
%
% Recover structural displacements and rotations
%
  UL = invKall * (Kaxl * UX(:,ntrim) + F(ldof,1) - (Mll * D + Mlr) * UDD(SUP_INDEX,ntrim) );

else
      ntrim=1;
      if (beam_model.Param.DIVERG > 0) 
        KDIV = Kll;
      end
	  	fprintf(fid,'\nSolving deformable constrained trim condition...');
      Kall = Kll - Qaall;
      invKall = inv(Kall);
      nc = find(beam_model.Aero.Trim.CS.MPC == 0); % get master rotations
      defl = beam_model.Aero.Trim.CS.Value(nc)'; 
			UL = zeros(size(Kll,1));
      UX = zeros(length(nc)+5,1);
      UX(1:5) = beam_model.Aero.Trim.FM.Value(2:6)';
      UX(6:end) = defl;
      UDD = beam_model.Aero.Trim.FM.Value(7:12)';
      GF = Kaxl * UX + F(ldof,1);
      for k=1:3
        accdof = beam_model.Node.DOF(:,k);
				accindex1 = find(accdof<=size(Mll,1));
				accindex2 = find(accdof>0);
				accindex = intersect(accindex1,accindex2);				
				accdof = accdof(accindex);
        ACC = zeros(size(Mll,1),1);
        ACC(accdof,1) = UDD(k);
        GF = GF - Mll * ACC;
      end 
      UL = invKall * GF; 

end
% Determine divergence dynamic pressure for the restrained aircraft if required
%
  beam_model.Res.Aero.DIVERG_Q = 0;
%
  if (beam_model.Param.DIVERG > 0)
    fprintf(fid, '\n Solving for restrained aeroelastic divergence...');
    DIV_FOUND = false;



%
    [DMODE, dynp] = eig(full(KDIV), full(Qaall)./QINF);
%
    dynp = diag(dynp);
    index = find(real(dynp)>0);
    mindex = [];
    e = [];
    for n=1:length(index)
      if isreal(dynp(index(n)))
        e = [e, dynp(index(n))];
        mindex = [mindex, index(n)];
      end
    end
%
    if (~isempty(e))

      DIV_FOUND = true;
%     sort eigenvalues
      [e, index] = sort(e);
      mindex = mindex(index);
      beam_model.Res.Aero.DIVERG_Q = e(1:beam_model.Param.DIVERG);
%
      for k=1:beam_model.Param.DIVERG
        SOL = zeros(length(ldof)+length(rdof), 1); 
        SOL(ldof,1) = DMODE(:,mindex(k));
        SOL(rdof,1) = 0.0;
        %
        if ~isempty(beam_model.RBE2.ID)
            SOL = RBE2disp(beam_model.RBE2,SOL,ndof);
        end
        
        gdef = zeros(beam_model.Info.ngrid, 6);
        for n = 1:beam_model.Info.ngrid
	        dof = beam_model.Node.DOF(n, 1:6);
	        index = find(dof);
	        if ~isempty(index)
		        gdef(n, index) = SOL(dof(index));
	        end
        end
        % store nodal displacement
        beam_model.Res.NDispl(:,:,1+k) = gdef;
        % set delta Rot
        for n = 1:beam_model.Info.ngrid

	        beam_model.Res.NRd(:,:,n,1+k) = Rmat(gdef(n, 4:6));

        end
        if (beam_model.Info.nrbe0 > 0)
	           AERO_POS = update_aerobeam_node(beam_model.Info.ngrid, beam_model.Node, beam_model.Res.NDispl(:,1:3,1+k),...
                                             beam_model.Res.NRd(:,:,:,1+k)-repmat(eye(3,3),[1,1,beam_model.Info.ngrid]));
            % update coord database with slave nodes position
            for n=1:beam_model.Info.ngrid
      	      ne = length(beam_model.Node.Aero.Index(n).data);
      	      if ne
		            beam_model.Res.NDispl(beam_model.Node.Aero.Index(n).data, 1:3, 1+k) = AERO_POS(n).data';
      	      end
            end
            clear AERO_POS;
        end
%
      end % mode loop
%
    end % divergence detected

    if (DIV_FOUND)
      fprintf(fid, '\n - Divergence dynamic pressure: %4e [Pa].', beam_model.Res.Aero.DIVERG_Q); 
    else
      fprintf(fid, '\n - No divergence detected.'); 
    end
    fprintf(fid,'\ndone.');

  end % divergence calculation
%     
  SOL =  zeros(size(K,1),1); SOL(rdof,1) = 0.0; SOL(ldof,1) = UL; 
	
  if ~isempty(beam_model.RBE2.ID)
      SOL = RBE2disp(beam_model.RBE2,SOL,ndof);
  end
  gdef = zeros(beam_model.Info.ngrid, 6);                     
  %  
  for n = 1:beam_model.Info.ngrid 
	  dof = beam_model.Node.DOF(n, 1:6);
	  index = find(dof);
	  if ~isempty(index)
		  gdef(n, index) = SOL(dof(index));
	  end
  end
  % store nodal displacement
  beam_model.Res.NDispl(:,:,1) = gdef;
  % set delta Rot
  for n = 1:beam_model.Info.ngrid

	  beam_model.Res.NRd(:,:,n,1) = Rmat(gdef(n, 4:6));

  end
  % recover solution in matrix form
  [DS, DR] = get_nodal_displ(beam_model.Info.ngrid, beam_model.Node.DOF, SOL); %
	% update BAR rotations
  beam_model.Res.Bar.R = update_bar_rot(nbar, DR, beam_model.Bar.Conn, beam_model.Res.Bar.R , DS); % ok
	% update BEAM rotations
	beam_model.Res.Beam.R = update_bar_rot(nbeam, DR, beam_model.Beam.Conn, beam_model.Res.Beam.R , DS);
  %
  COORD = beam_model.Node.Coord + beam_model.Res.NDispl(:,1:3);
  beam_model.Res.Bar.Colloc = bar_defo_colloc(nbar, beam_model.Bar, beam_model.Node.DOF, COORD, beam_model.Res.NRd);
  beam_model.Res.Beam.Colloc = bar_defo_colloc(nbeam, beam_model.Beam, beam_model.Node.DOF, COORD, beam_model.Res.NRd);
  beam_model.Res.WB = [];
  % calculate new mass matrix
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] = ...
     wb_set_conm_mass(beam_model.Info.nconm, beam_model.Node.Index, COORD, beam_model.Res.NRd, beam_model.Param.GRDPNT, beam_model.ConM);
  % set bar mass CG
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] =...
     wb_add_bar_mass(nbar, COORD, beam_model.Res.NRd, beam_model.Res.WB.CG, beam_model.Param.GRDPNT, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP, beam_model.Bar);
  % set beam mass CG
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] =...
     wb_add_bar_mass(nbeam, COORD, beam_model.Res.NRd, beam_model.Res.WB.CG, beam_model.Param.GRDPNT, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP, beam_model.Beam);
  % get principal axes
  [beam_model.Res.WB.MCG_pa, beam_model.Res.WB.R_pa] = wb_principal_axis(beam_model.Res.WB.MCG);
  %
  % update aerobeam nodes (if any)    
  %
  if (beam_model.Info.nrbe0 > 0)
	     AERO_POS = update_aerobeam_node(beam_model.Info.ngrid, beam_model.Node, beam_model.Res.NDispl(:,1:3,1),...
                                       beam_model.Res.NRd(:,:,:,1)-repmat(eye(3,3),[1,1,beam_model.Info.ngrid]));
      % update coord database with slave nodes position
      for n=1:beam_model.Info.ngrid
      	ne = length(beam_model.Node.Aero.Index(n).data);
      	if ne
		      beam_model.Res.NDispl(beam_model.Node.Aero.Index(n).data, 1:3, 1) = AERO_POS(n).data';
      	end
      end
      clear AERO_POS;
  end
  %
  % assembly BAR contributions directly in the undeformed position
  [beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
    get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, ...
    beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

  % assembly BEAM contributions directly in the undeformed position
  [beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
    get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, ...
    beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

  % save internal aero database
  beam_model.Res.Aero.Qaa = Qaa;
  beam_model.Res.Aero.Kax = Kax;
  beam_model.Res.Aero.Fa0 = Fa0;
  beam_model.Res.Aero.Fa0tot = Fa0 - KaxDOF * UX(:,1);
  clear KaxDOF;
  %
  % update final vector
  index = find(beam_model.Aero.Trim.CS.MPC);
  for k=1:length(index)
      beam_model.Res.CS.Value(ntrim,index(k)) = beam_model.Aero.Trim.CS.Coeff(index(k)) * beam_model.Res.CS.Value(ntrim,beam_model.Aero.Trim.CS.MPC(index(k)));
  end
  
  beam_model.Aero.lattice_vlm = rotate_control_surf(beam_model.Aero.ref, beam_model.Aero.state, beam_model.Aero.geo, ...
      beam_model.Aero.lattice_vlm, beam_model.Res.CS.Value(ntrim,:), ...
      beam_model.Aero.lattice_vlm.Control.Hinge);
  if (SPLINE_TYPE==1)
    beam_model.Aero.lattice_defo = update_vlm_mesh1(beam_model.Node, SOL, beam_model.Aero);
  else
    beam_model.Aero.lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord + beam_model.Res.NDispl(:,1:3), ...
        beam_model.Res.NRd, beam_model.Aero);
  end
  beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
  beam_model.Aero.lattice = [];
  beam_model.Res.CPaero.State =  Fa_State_aero;
  beam_model.Res.CPaero.Control =  FCtot_aero;
  beam_model.Res.CPaero.Defo    =  CPaeroDef;
  beam_model.Res.CPaero.F0 =  results0.F;
  beam_model.Res.Gamma.Gamma_P = GAMMA_P;
  beam_model.Res.Gamma.Gamma_I = GAMMA_I;
%
	fprintf(fid, '\n\ncompleted.\n\n');
%
  if ~isempty(SUP_INDEX)
    print_aeroderRE_aa(outp, SUP_INDEX, beam_model.Res.Aero.RStab_Der, beam_model.Res.Aero.DStab_Der, nr);
  end
%  
  if (length(find(ismember(SUP_INDEX,[3 5])))==2)
    if (nbody)
      FUSD = []; FUSL = [];
    else  
      [FUSD, FUSL] = RecoverFusGeoDataFromBeamModel(beam_model);
    end

   [beam_model.Res.Aero.RStability, beam_model.Res.Aero.DStability] = get_stab_marginRE(beam_model.Aero.geo.ref_point, beam_model.WB.CG, ...
                                                                     CREF, SREF, FUSD, FUSL, beam_model.Res.Aero.RStab_Der, beam_model.Res.Aero.DStab_Der, outp);
  end
%
  fclose(outp);
	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
		
end		
%***********************************************************************************************************************
function INTER = get_aero_intercept_aa(SUP_INDEX, AEROMAT, HINGEMAT)

  INTER = [];

  for k=1:length(SUP_INDEX)

    dof = SUP_INDEX(k);

    switch(dof)
    case 2
    INTER.cs0  = AEROMAT(k,1);
    case 3
    INTER.cl0  = AEROMAT(k,1);
    case 4
    INTER.cml0 = AEROMAT(k,1);
    case 5
    INTER.cmm0 = AEROMAT(k,1);
    case 6
    INTER.cmn0 = AEROMAT(k,1);

    end

    if ~isempty(HINGEMAT)
      INTER.cmh0 = HINGEMAT(:,1);
    end

  end
end
%***********************************************************************************************************************
function print_intercept_aa(fid, SUP_INDEX, INT);

  fprintf(fid,'\n\n Reference intercepts\n');
  for k=1:length(SUP_INDEX)
    dof = SUP_INDEX(k);
    switch(dof)
      case 2
        fprintf(fid,'\n - Cy0: %g.', INT.cs0);
      case 3
            fprintf(fid,'\n - Cz0: %g.', INT.cl0);
      case 4
            fprintf(fid,'\n - Cl0: %g.', INT.cml0);
      case 5
            fprintf(fid,'\n - Cm0: %g.', INT.cmm0);
      case 6
            fprintf(fid,'\n - Cn0: %g.', INT.cmn0);
    end
  end
end
%***********************************************************************************************************************
function print_aeroderRE_aa(fid, SUP_INDEX, RStab_Der, DStab_Der, nr)

fprintf(fid, '\n\nAERODYNAMIC DERIVATIVES\n');

T2 = find(SUP_INDEX==2);
T3 = find(SUP_INDEX==3);
T4 = find(SUP_INDEX==4);
T5 = find(SUP_INDEX==5);
T6 = find(SUP_INDEX==6);

fprintf(fid,'\nALPHA\n');
fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
if T2
  fprintf(fid,'\n Cy/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcs_dalpha);    e=sprintf('%-20.5f',DStab_Der.Alpha.dcs_dalpha);     er=sprintf('%-20.5f', DStab_Der.Alpha.dcs_dalpha/RStab_Der.Alpha.dcs_dalpha);   fprintf(fid,'%s%s%s',r,e,er);                                                                       
end
if T3
  fprintf(fid,'\n Cz/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcl_dalpha);    e=sprintf('%-20.5f', DStab_Der.Alpha.dcl_dalpha);    er=sprintf('%-20.5f', DStab_Der.Alpha.dcl_dalpha/RStab_Der.Alpha.dcl_dalpha);   fprintf(fid,'%s%s%s',r,e,er);
end
if T4
  fprintf(fid,'\n Cl/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcml_dalpha);   e=sprintf('%-20.5f', DStab_Der.Alpha.dcml_dalpha);   er=sprintf('%-20.5f', DStab_Der.Alpha.dcml_dalpha/RStab_Der.Alpha.dcml_dalpha); fprintf(fid,'%s%s%s',r,e,er);
end
if T5
  fprintf(fid,'\n Cm/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcmm_dalpha);   e=sprintf('%-20.5f', DStab_Der.Alpha.dcmm_dalpha);   er=sprintf('%-20.5f', DStab_Der.Alpha.dcmm_dalpha/RStab_Der.Alpha.dcmm_dalpha); fprintf(fid,'%s%s%s',r,e,er);
end
if T6
  fprintf(fid,'\n Cn/alpha            '); r=sprintf('%-20.5f', RStab_Der.Alpha.dcmn_dalpha);   e=sprintf('%-20.5f', DStab_Der.Alpha.dcmn_dalpha);   er=sprintf('%-20.5f', DStab_Der.Alpha.dcmn_dalpha/RStab_Der.Alpha.dcmn_dalpha); fprintf(fid,'%s%s%s',r,e,er);
end
fprintf(fid,'\n\nBETA\n');
fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
if T2
  fprintf(fid,'\n Cy/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcs_dbeta);      e=sprintf('%-20.5f', DStab_Der.Beta.dcs_dbeta);      er=sprintf('%-20.5f', DStab_Der.Beta.dcs_dbeta/RStab_Der.Beta.dcs_dbeta);   fprintf(fid,'%s%s%s',r,e,er);
end
if T3
  fprintf(fid,'\n Cz/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcl_dbeta);      e=sprintf('%-20.5f', DStab_Der.Beta.dcl_dbeta);      er=sprintf('%-20.5f', DStab_Der.Beta.dcl_dbeta/RStab_Der.Beta.dcl_dbeta);   fprintf(fid,'%s%s%s',r,e,er);
end
if T4
  fprintf(fid,'\n Cl/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcml_dbeta);     e=sprintf('%-20.5f', DStab_Der.Beta.dcml_dbeta);     er=sprintf('%-20.5f', DStab_Der.Beta.dcml_dbeta/RStab_Der.Beta.dcml_dbeta); fprintf(fid,'%s%s%s',r,e,er);
end
if T5
  fprintf(fid,'\n Cm/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcmm_dbeta);     e=sprintf('%-20.5f', DStab_Der.Beta.dcmm_dbeta);     er=sprintf('%-20.5f', DStab_Der.Beta.dcmm_dbeta/RStab_Der.Beta.dcmm_dbeta); fprintf(fid,'%s%s%s',r,e,er);
end
if T6
  fprintf(fid,'\n Cn/beta             '); r=sprintf('%-20.5f', RStab_Der.Beta.dcmn_dbeta);     e=sprintf('%-20.5f', DStab_Der.Beta.dcmn_dbeta);     er=sprintf('%-20.5f', DStab_Der.Beta.dcmn_dbeta/RStab_Der.Beta.dcmn_dbeta); fprintf(fid,'%s%s%s',r,e,er);
end
fprintf(fid,'\n\nROLL RATE\n');
fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
if T2
  fprintf(fid,'\n Cy/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcs_dP);       e=sprintf('%-20.5f', DStab_Der.P_rate.dcs_dP);       er=sprintf('%-20.5f', DStab_Der.P_rate.dcs_dP/RStab_Der.P_rate.dcs_dP);   fprintf(fid,'%s%s%s',r,e,er);
end
if T3
  fprintf(fid,'\n Cz/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcl_dP);       e=sprintf('%-20.5f', DStab_Der.P_rate.dcl_dP);       er=sprintf('%-20.5f', DStab_Der.P_rate.dcl_dP/RStab_Der.P_rate.dcl_dP);   fprintf(fid,'%s%s%s',r,e,er);
end
if T4
  fprintf(fid,'\n Cl/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcml_dP);      e=sprintf('%-20.5f', DStab_Der.P_rate.dcml_dP);      er=sprintf('%-20.5f', DStab_Der.P_rate.dcml_dP/RStab_Der.P_rate.dcml_dP); fprintf(fid,'%s%s%s',r,e,er);
end
if T5
  fprintf(fid,'\n Cm/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcmm_dP);      e=sprintf('%-20.5f', DStab_Der.P_rate.dcmm_dP);      er=sprintf('%-20.5f', DStab_Der.P_rate.dcmm_dP/RStab_Der.P_rate.dcmm_dP); fprintf(fid,'%s%s%s',r,e,er);
end
if T6
  fprintf(fid,'\n Cn/p                '); r=sprintf('%-20.5f', RStab_Der.P_rate.dcmn_dP);      e=sprintf('%-20.5f', DStab_Der.P_rate.dcmn_dP);      er=sprintf('%-20.5f', DStab_Der.P_rate.dcmn_dP/RStab_Der.P_rate.dcmn_dP); fprintf(fid,'%s%s%s',r,e,er);
end
fprintf(fid,'\n\nPITCH RATE\n');
fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
if T2
  fprintf(fid,'\n Cy/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcs_dQ);       e=sprintf('%-20.5f', DStab_Der.Q_rate.dcs_dQ);       er=sprintf('%-20.5f', DStab_Der.Q_rate.dcs_dQ/RStab_Der.Q_rate.dcs_dQ);   fprintf(fid,'%s%s%s',r,e,er);
end
if T3
  fprintf(fid,'\n Cz/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcl_dQ);       e=sprintf('%-20.5f', DStab_Der.Q_rate.dcl_dQ);       er=sprintf('%-20.5f', DStab_Der.Q_rate.dcl_dQ/RStab_Der.Q_rate.dcl_dQ);   fprintf(fid,'%s%s%s',r,e,er);
end
if T4
  fprintf(fid,'\n Cl/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcml_dQ);      e=sprintf('%-20.5f', DStab_Der.Q_rate.dcml_dQ);      er=sprintf('%-20.5f', DStab_Der.Q_rate.dcml_dQ/RStab_Der.Q_rate.dcml_dQ); fprintf(fid,'%s%s%s',r,e,er);
end
if T5
  fprintf(fid,'\n Cm/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcmm_dQ);      e=sprintf('%-20.5f', DStab_Der.Q_rate.dcmm_dQ);      er=sprintf('%-20.5f', DStab_Der.Q_rate.dcmm_dQ/RStab_Der.Q_rate.dcmm_dQ); fprintf(fid,'%s%s%s',r,e,er);
end
if T6
  fprintf(fid,'\n Cn/q                '); r=sprintf('%-20.5f', RStab_Der.Q_rate.dcmn_dQ);      e=sprintf('%-20.5f', DStab_Der.Q_rate.dcmn_dQ);      er=sprintf('%-20.5f', DStab_Der.Q_rate.dcmn_dQ/RStab_Der.Q_rate.dcmn_dQ); fprintf(fid,'%s%s%s',r,e,er);
end
fprintf(fid,'\n\nYAW RATE\n');
fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
if T2
  fprintf(fid,'\n Cy/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcs_dR);       e=sprintf('%-20.5f', DStab_Der.R_rate.dcs_dR);       er=sprintf('%-20.5f', DStab_Der.R_rate.dcs_dR/RStab_Der.R_rate.dcs_dR);   fprintf(fid,'%s%s%s',r,e,er);
end
if T3
  fprintf(fid,'\n Cz/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcl_dR);       e=sprintf('%-20.5f', DStab_Der.R_rate.dcl_dR);       er=sprintf('%-20.5f', DStab_Der.R_rate.dcl_dR/RStab_Der.R_rate.dcl_dR);   fprintf(fid,'%s%s%s',r,e,er);
end
if T4
  fprintf(fid,'\n Cl/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcml_dR);      e=sprintf('%-20.5f', DStab_Der.R_rate.dcml_dR);      er=sprintf('%-20.5f', DStab_Der.R_rate.dcml_dR/RStab_Der.R_rate.dcml_dR); fprintf(fid,'%s%s%s',r,e,er);
end
if T5
  fprintf(fid,'\n Cm/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcmm_dR);      e=sprintf('%-20.5f', DStab_Der.R_rate.dcmm_dR);      er=sprintf('%-20.5f', DStab_Der.R_rate.dcmm_dR/RStab_Der.R_rate.dcmm_dR); fprintf(fid,'%s%s%s',r,e,er);
end
if T6
  fprintf(fid,'\n Cn/r                '); r=sprintf('%-20.5f', RStab_Der.R_rate.dcmn_dR);      e=sprintf('%-20.5f', DStab_Der.R_rate.dcmn_dR);      er=sprintf('%-20.5f', DStab_Der.R_rate.dcmn_dR/RStab_Der.R_rate.dcmn_dR); fprintf(fid,'%s%s%s',r,e,er);
end
for i=1:length(nr)
  fprintf(fid,'\n\n%s\n', RStab_Der.Control.Name{i});
  fprintf(fid,'\n NAME                RIGID               ELASTIC             RATIO E/R');
  if T2
    fprintf(fid,'\n Cy/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcs_dDelta(i));   e=sprintf('%-20.5f', DStab_Der.Control.dcs_dDelta(i));      er=sprintf('%-20.5f', DStab_Der.Control.dcs_dDelta(i)/RStab_Der.Control.dcs_dDelta(i));   fprintf(fid,'%s%s%s',r,e,er);
  end
  if T3
    fprintf(fid,'\n Cz/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcl_dDelta(i));   e=sprintf('%-20.5f', DStab_Der.Control.dcl_dDelta(i));      er=sprintf('%-20.5f', DStab_Der.Control.dcl_dDelta(i)/RStab_Der.Control.dcl_dDelta(i));   fprintf(fid,'%s%s%s',r,e,er);
  end
  if T4
    fprintf(fid,'\n Cl/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcml_dDelta(i));  e=sprintf('%-20.5f', DStab_Der.Control.dcml_dDelta(i));     er=sprintf('%-20.5f', DStab_Der.Control.dcml_dDelta(i)/RStab_Der.Control.dcml_dDelta(i)); fprintf(fid,'%s%s%s',r,e,er);
  end
  if T5
    fprintf(fid,'\n Cm/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcmm_dDelta(i));  e=sprintf('%-20.5f', DStab_Der.Control.dcmm_dDelta(i));     er=sprintf('%-20.5f', DStab_Der.Control.dcmm_dDelta(i)/RStab_Der.Control.dcmm_dDelta(i)); fprintf(fid,'%s%s%s',r,e,er);
  end
  if T6
    fprintf(fid,'\n Cn/Delta            '); r=sprintf('%-20.5f', RStab_Der.Control.dcmn_dDelta(i));  e=sprintf('%-20.5f', DStab_Der.Control.dcmn_dDelta(i));     er=sprintf('%-20.5f', DStab_Der.Control.dcmn_dDelta(i)/RStab_Der.Control.dcmn_dDelta(i)); fprintf(fid,'%s%s%s',r,e,er);
  end
end
end