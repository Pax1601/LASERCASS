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
%                      Sergio Ricci           <ricci@aero.polimi.it>
%                      Luca Cavagna           <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari  <degaspari@aero.polimi.it>
%                      Luca Riccobene         <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function solve_free_lin_trim_cfd(varargin)

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

    nc = beam_model.Aero.geo.nc;
%
%   select trim case
%
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
%   select external forces having the same trim ID
    LOADI = beam_model.Aero.Trim.ID(index);    
		if ~isempty(index)		
      dotpos = strfind(beam_model.Param.FILE,'.');
      outf = [beam_model.Param.FILE(1:dotpos-1), '_man_', num2str(beam_model.Aero.Trim.ID(index)),'.txt'];
%
      switch (beam_model.Aero.Trim.Type(index))

      case 0
        NDOF = 6;
        STRIM = 0;
        FMDOF = [];
      case 1 % full simmetric trim
        NDOF = 3;
        STRIM = 1; 
        FMDOF = [1,3,5];
      case 2 % pitch spin
        NDOF = 1;
        STRIM = 1; 
        FMDOF = [5];
      case 3 % plunge spin
        NDOF = 1;
        STRIM = 1; 
        FMDOF = [3];
      case -1 % full anti-simmetric trim
        NDOF = 3;
        STRIM = 1; 
        FMDOF = [2,4,6];
      case -2 % roll spin
        NDOF = 1;
        STRIM = 1;  
        FMDOF = [4];
      case -3 % yaw spin
        NDOF = 1;
        STRIM = 1;  
        FMDOF = [6];
%
      end
  		fprintf(fid,'\nSolving linear static unrestrained trim (ID %d)...\n\n', beam_model.Aero.Trim.ID(index));
%
			% set flight mechanics trim params
      beam_model.Aero.Trim.Extra = check_extra_param(beam_model.Aero.Trim.NC(index), ...
        beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
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
    ne  = sum(beam_model.Aero.Trim.Extra.Fixed);
		if (ncs + nfm + ne) ~= beam_model.Aero.Trim.NC(index)    

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
    % count variables
    NC_TOT = sum(beam_model.Res.FM.Fixed) + sum(beam_model.Res.CS.Fixed);
    NTOT = length(beam_model.Res.FM.Fixed) + length(beam_model.Res.CS.Fixed);
    % free variables
    NF_TOT = NTOT - NC_TOT;
%
    %
    if NF_TOT ~= NDOF
        if NF_TOT<NDOF
            error('Number of degrees of freedom are less than number of DOF %d (%d).', NDOF, NF_TOT);
        else
            % 	    
            fprintf(fid, '\n - Non linear trim solution...');
        end
    end
    %
    %
    outp = fopen(outf, 'w');
    print_state(outp, beam_model.Aero.state); 
    print_attitude(outp, beam_model.Res.FM);
    print_trim_problem(outp, NF_TOT, NDOF);
    %
		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;
    nbody = beam_model.Info.nbaero;
    npa = length(beam_model.Aero.lattice_vlm.COLLOC);
    SPLINE_TYPE = beam_model.Info.spline_type;
		% create intermediate storage struct
		fprintf(fid, '\n- Setting internal database...');
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
    %
    % run aeroelastic interpolation
    aeroelastic_interface;
    % run aeroelastic interpolation for ext loads
    MastSurf = [];
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    for i=1:length(nr)
      MastSurf{i} = beam_model.Aero.lattice_vlm.Control.Name{nr(i)};
    end
    aeroelastic_interface_ext(MastSurf);
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
%
		fprintf(fid,'\n- Setting SUPORT matrix...');
%   SUPORT DOF
    if (beam_model.Info.cc_nspc)
      error('SPC case control card detected. This solver allows trim analysis for free-free aircraft.');
    end  
%   Stiffness matrix
    if (~isempty(beam_model.Param.SUPORT))
        dummy = beam_model.Node;
        if ~isempty(beam_model.RBE2.ID)
            dummy.DOF = dummy.DOF2;
        end
      [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, dummy, beam_model.Param.SUPORT, beam_model.Param.EPS);
      nr = find(KEPS < beam_model.Param.EPS);
      if (~isempty(nr))
        fprintf(fid, '\nWarning: %d SUPORT rigid modes exceed deformation energy tolerance %g.', length(nr), beam_model.Param.EPS);
      end
    else 
      error('No SUPORT card given.');
    end
%   Provisory 
    if ( (size(beam_model.Param.SUPORT,1)>1) || (length(num2str(beam_model.Param.SUPORT(1,2)))~=6))
      error('Provisory: one one suport point with 6 dofs is enabled at the moment.');
    end
    fprintf(fid, 'done.');
%   Mass matrix
    Mll = M(ldof, ldof); Mlr = M(ldof, rdof); Mrr = M(rdof, rdof); Mrl = M(rdof, ldof); 
%   Rigid body mass matrix for the SUPORTED DOFs 
    mr = Mrr + Mrl*D + D'*Mlr + D'*Mll*D;   
    MSRR = mr; 
%
%   Aerodynamic influence matrix
		fprintf(fid,'\nGenerating aerodynamic database...');
    sindex = find(beam_model.Param.SUPORT(1,1) == beam_model.Node.ID);
    beam_model.Aero.geo.ref_point = beam_model.Node.Coord(sindex,:); % set for moments calculations
    beam_model.Aero.geo.CG = beam_model.Node.Coord(sindex,:); % set for angular velocities
%***********************************************************************************************************************
%
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
%
%   exclude TRIMEXT caeros
%   add to CT panels, external patches
%   set BODYINT to null value
%
    DOFCTE = DOFCT;
    DOFCTVLM = DOFCT;
%
    BODYINT = ones(nbody,1);
%
    for i=1:beam_model.Info.ntrimext
      for k=1:length(beam_model.Aero.Trim.Ext.CAERO(i).data)
        patch = beam_model.Aero.Trim.Ext.CAERO(i).data(k);
        if patch>=100
          index = find(beam_model.Aero.ID == patch);
          DOFCTE = [DOFCTE, [beam_model.Aero.lattice_vlm.DOF(index, 1):beam_model.Aero.lattice_vlm.DOF(index, 2)]];
          fprintf(fid, '\n\t CAERO %d excluded from VLM force calculation.', beam_model.Aero.ID(index));
        else
         index = find(beam_model.Aero.body.ID == patch);
         BODYINT(index) = 0;
         fprintf(fid, '\n\t BCAERO %d excluded from VLM force calculation.', beam_model.Aero.body.ID(index));
        end
      end
    end
%
    for k=1:length(beam_model.Param.AEROINT)
      patch = beam_model.Param.AEROINT(k);
      if patch>=100
        index = find(beam_model.Aero.ID == patch);
        DOFCTE = [DOFCTE, [beam_model.Aero.lattice_vlm.DOF(index, 1):beam_model.Aero.lattice_vlm.DOF(index, 2)]];
        fprintf(fid, '\n\t CAERO %d excluded from VLM force calculation.', beam_model.Aero.ID(index));
        DOFCTE = unique(DOFCTE);
      else
       index = find(beam_model.Aero.body.ID == patch);
       BODYINT(index) = 0;
       fprintf(fid, '\n\t BCAERO %d excluded from VLM force calculation.', beam_model.Aero.body.ID(index));
      end
    end
%
%   REFERENCE LOAD ON WHOLE VLM (used for derivatives)
%
    [results0, GAMMA_MAT, NELEM] = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, DOFCTVLM);
    Fa0 = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results0);
%   save data for the case the user does not provide ext derivative
    Fa0VLM = Fa0;
    results0VLM = results0;
%
%
%   REFERENCE LOAD ON REDUCED VLM (used for reference load)
%
    DER_VLM = zeros(5+length(MastSurf),1); % require derivative on WHOLE VLM mesh if 0
%
    if beam_model.Info.ntrimext
      results0e = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTE);    
%     reference load on VLM
      Fa0ed = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results0e, BODYINT);
      Fa0e = Fa0ed;
%     add external
      for i=1:beam_model.Info.ntrimext
        Fa0e = Fa0e + beam_model.Aero.Trim.Ext.F0{i};
      end
    else
      Fa0e = Fa0;
    end
%
%
%   Total external forces (Applied loads + Follower + reference rigid aero condition)
    F = F + F_flw + Fa0e; 
		fprintf(fid,'done.'); 
    if ~isempty(beam_model.RBE2.ID)
       F = RBE2Assembly2(beam_model.RBE2,F);
    end   
%
%   MODIFY REFERENCE LOAD FOR DERIVATIVES provided by user
%
    if (beam_model.Param.DER_TYPE == 2)
      results0 = results0e;
      Fa0 = Fa0ed;
      DOFCT = DOFCTE;
      for n=1:beam_model.Info.nderext
        DER_VLM(beam_model.Aero.Trim.DExt.Index(n).data) = 1; %set to zero to require
      end
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
    if DER_VLM(1)
      fprintf(fid,'\n\tRecovered from hybrid model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
      Fa_ALPHA = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results, BODYINT);
      Fa_ALPHA = (Fa_ALPHA - Fa0)./ EPS; % get force variation
      Fa_State_aero([1:3:end], 1) = [results.F(:,1)-results0.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 1) = [results.F(:,2)-results0.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 1) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    else % whole VLM
      fprintf(fid,'\n\tRecovered from whole VLM model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
      Fa_ALPHA = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, results, ones(nbody,1));
      Fa_ALPHA = (Fa_ALPHA - Fa0VLM)./ EPS; % get force variation
      Fa_State_aero([1:3:end], 1) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 1) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 1) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
    end
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
    if DER_VLM(2)
      fprintf(fid,'\n\tRecovered from hybrid model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
      Fa_BETA = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results, BODYINT);
      Fa_BETA = (Fa_BETA - Fa0) ./ EPS;
      Fa_State_aero([1:3:end], 2) = [results.F(:,1)-results0.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 2) = [results.F(:,2)-results0.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 2) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    else % whole VLM
      fprintf(fid,'\n\tRecovered from whole VLM model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
      Fa_BETA = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results);
      Fa_BETA = (Fa_BETA - Fa0VLM) ./ EPS;
      Fa_State_aero([1:3:end], 2) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 2) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 2) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
    end
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
    if DER_VLM(3)
      fprintf(fid,'\n\tRoll rate derivative recovered from hybrid model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
      Fa_P = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results, BODYINT);
      Fa_P = (Fa_P - Fa0) ./ EPS;
      Fa_State_aero([1:3:end], 3) = [results.F(:,1)-results0.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 3) = [results.F(:,2)-results0.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 3) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    else
      fprintf(fid,'\n\tRoll rate derivative recovered from whole VLM model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
      Fa_P = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results);
      Fa_P = (Fa_P - Fa0VLM) ./ EPS;
      Fa_State_aero([1:3:end], 3) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 3) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 3) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
    end
    if nc
      HINGEMAT(:,3) = (2*VREF/BREF).*(results.HINGE-results0.HINGE)./EPS;
    end
%   set to adimensional angular speed
    Fa_P = (2*VREF)*Fa_P ./ BREF;
    Fa_Pl = Fa_P(ldof,1); Fa_Pr = Fa_P(rdof,1);
%   Q
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = EPS;      dummy_aero.state.R = 0;
    if DER_VLM(4)
      fprintf(fid,'\n\tPitch rate derivative recovered from hybrid model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
      Fa_Q = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results, BODYINT);
      Fa_Q = (Fa_Q - Fa0) ./ EPS;
      Fa_State_aero([1:3:end], 4) = [results.F(:,1)-results0.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 4) = [results.F(:,2)-results0.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 4) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    else
      fprintf(fid,'\n\tPitch rate derivative recovered from whole VLM model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
      Fa_Q = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results);
      Fa_Q = (Fa_Q - Fa0VLM) ./ EPS;
      Fa_State_aero([1:3:end], 4) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 4) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 4) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
    end
    if nc
      HINGEMAT(:,4) = (2*VREF/CREF).*(results.HINGE-results0.HINGE)./EPS;
    end
%   set to adimensional angular speed
    Fa_Q = (2*VREF)*Fa_Q ./ CREF;
    Fa_Ql = Fa_Q(ldof,1); Fa_Qr = Fa_Q(rdof,1);
%   R
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = EPS;
    if DER_VLM(5)
      fprintf(fid,'\n\tYaw rate derivative recovered from hybrid model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
      Fa_R = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results, BODYINT);
      Fa_R = (Fa_R - Fa0) ./ EPS;
      Fa_State_aero([1:3:end], 5) = [results.F(:,1)-results0.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 5) = [results.F(:,2)-results0.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 5) = [results.F(:,3)-results0.F(:,3)]./ EPS;
    else
      fprintf(fid,'\n\tYaw rate derivative recovered from VLM model.');
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
      Fa_R = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
          results);
      Fa_R = (Fa_R - Fa0VLM) ./ EPS;
      Fa_State_aero([1:3:end], 5) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
      Fa_State_aero([2:3:end], 5) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
      Fa_State_aero([3:3:end], 5) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
    end
    if nc
      HINGEMAT(:,5) = (2*VREF/BREF).*(results.HINGE-results0.HINGE)./EPS;
    end
    %   set to adimensional angular speed
    Fa_R = (2*VREF)*Fa_R ./ BREF;
    Fa_Rl = Fa_R(ldof,1); Fa_Rr = Fa_R(rdof,1);
		fprintf(fid,'done.');
%
%   Control surfaces
%
    dummy_aero = beam_model.Aero; 
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    FCl = [];
    FCr = [];
%
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
                                                 beam_model.Aero.lattice.Control.Hinge, beam_model.Aero.ID, 1);
        if DER_VLM(5+k)
          fprintf(fid,'\n\tControl %s derivative recovered from hybrid model.',cell2mat(MastSurf(k)));
          results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);    
          Fa_C = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results, BODYINT);
          Fa_C = (Fa_C - Fa0) ./ EPS;
          FCtot_aero([1:3:end], k) = [results.F(:,1)-results0.F(:,1)]./ EPS;
          FCtot_aero([2:3:end], k) = [results.F(:,2)-results0.F(:,2)]./ EPS;
          FCtot_aero([3:3:end], k) = [results.F(:,3)-results0.F(:,3)]./ EPS;
        else
         fprintf(fid,'\n\tControl %s derivative recovered from VLM model.',cell2mat(MastSurf(k)));
          results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
          dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCTVLM);    
          Fa_C = gf_transfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results);
          Fa_C = (Fa_C - Fa0VLM) ./ EPS;
          FCtot_aero([1:3:end], k) = [results.F(:,1)-results0VLM.F(:,1)]./ EPS;
          FCtot_aero([2:3:end], k) = [results.F(:,2)-results0VLM.F(:,2)]./ EPS;
          FCtot_aero([3:3:end], k) = [results.F(:,3)-results0VLM.F(:,3)]./ EPS;
        end
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
%
%
%   Assembly derivatives 
    Kax  =  -[Fa_ALPHA, Fa_BETA, Fa_P, Fa_Q, Fa_R, FCtot];
%
%   Apply external corrections
%
    switch (beam_model.Param.DER_TYPE)
%
      case {1} % correct global derivatives only
        DERK = load_trim_ext_corr(beam_model.Param.DER_FILE, MastSurf);
        for i=1:size(Kax,2)
          Kax(:,i) = Kax(:,i).*DERK(i);
        end
%
      case {2} % include spatial derivative
        for i=1:beam_model.Info.ntrimext
          index = beam_model.Aero.Trim.Ext.DExt(i);
          cols = beam_model.Aero.Trim.DExt.Index(index).data;
          Kax(:, cols) = Kax(:, cols) - beam_model.Aero.Trim.Ext.DF{i}.data;    
        end
    end
%
    KaxDOF = Kax;
    if ~isempty(beam_model.RBE2.ID) 
        Kax = RBE2Assembly2(beam_model.RBE2,Kax);
    end
%
    Kaxl = Kax(ldof,:); 
    Kaxr = Kax(rdof,:);
%    
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
%     Get master nodes connected to aerodynamics
      MNODE = [];
      AERO_DOF = [];
      for n = 1:ngrid
        if (~isempty(beam_model.Node.Aero.Index(n).data))
          MNODE = [MNODE, n];
        end
      end
%
      for m = 1:length(MNODE)
        n = MNODE(m);
        % DISPLACEMENT DOF
        for k=1:3
          % loop on node displacements DOFs
          if (beam_model.Node.DOF(n,k))
            AERO_DOF = [AERO_DOF, beam_model.Node.DOF(n,k)];
            NODEPOS = beam_model.Node.Coord;
            NODEPOS(n,k) = beam_model.Node.Coord(n,k) + EPS;  
%           determine updated lattice
	          lattice_defo = update_vlm_mesh(beam_model.Node, NODEPOS, beam_model.Node.R, dummy_aero);
            results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
              dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT); 
            Fa_D = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results, BODYINT);
            CPaeroDef([1:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,1)-results0.F(:,1)]./ EPS;
            CPaeroDef([2:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,2)-results0.F(:,2)]./ EPS;
            CPaeroDef([3:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,3)-results0.F(:,3)]./ EPS;
            Qaa(:,beam_model.Node.DOF(n, k)) = (Fa_D - Fa0) ./ EPS; 
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
            Fa_D = gf_transfer_aero_nodal_ext(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
              results, BODYINT);
            CPaeroDef([1:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,1)-results0.F(:,1)]./ EPS;
            CPaeroDef([2:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,2)-results0.F(:,2)]./ EPS;
            CPaeroDef([3:3:end], beam_model.Node.DOF(n, k)) = [results.F(:,3)-results0.F(:,3)]./ EPS;
           Qaa(:,beam_model.Node.DOF(n, k)) = (Fa_D - Fa0) ./ EPS; 
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
    QaaDOF = Qaa; 
    if ~isempty(beam_model.RBE2.ID)
        Qaa = RBE2Assembly(beam_model.RBE2,Qaa);
    end
    Qaall = Qaa(ldof, ldof);
    Qaalr = Qaa(ldof, rdof);
    Qaarl = Qaa(rdof, ldof);
    Qaarr = Qaa(rdof, rdof);
%  
    UX = zeros(length(find(beam_model.Aero.Trim.CS.MPC == 0))+5, 2);
%  
  for ntrim = 1:2

    if ntrim == 1 % switch off deformability effects on aerodynamics
	  	fprintf(fid,'\nSolving rigid aircraft trim condition...');
      Kall = Kll;
      Kalr = Klr;
      Karl = Krl;
      Karr = Krr;
      invKall = inv(Kall);
      RFLEX_MAT = invKall;
    else
	  	fprintf(fid,'\nSolving deformable aircraft trim condition...');
      Kall = Kll - Qaall;
      Kalr = Klr - Qaalr;
      Karl = Krl - Qaarl;
      Karr = Krr - Qaarr; 
      invKall = inv(Kall);
    end
%

    AMLR  = invKall * (Mll*D + Mlr); 
    ARLR  = invKall * Kalr;
    ALX   = invKall * Kaxl;
    UINTL = invKall * F(ldof,1);
%
    DUMMY = D'*Mll + Mrl;
    M2RR =  (D'*Mlr + Mrr) - DUMMY * ARLR;
    M3RR = -DUMMY * AMLR;
    K3LX = -DUMMY * ALX;
    TMP1 = DUMMY * UINTL;
%
    invM2RR = inv(M2RR);
    M4RR = invM2RR * M3RR;
    K4LX = invM2RR * K3LX;
    TMP2 = invM2RR * TMP1;
%
    DUMMY = D'*Kall + Karl;
    K2RR = -DUMMY * ARLR + (D'*Kalr + Karr);
    KAZL = DUMMY;
%    KARZX = KAZL - KAXL * ALX;
    KARZX = (D'*Kaxl + Kaxr) - KAZL * ALX;
    INTZ = D'*F(ldof,1) + F(rdof,1);
    IPZ = INTZ - DUMMY * UINTL;
%
    M5RR = -K2RR * M4RR + MSRR;
    MIRR = -KAZL * AMLR + M5RR;
    invMIRR = inv(MIRR);
    KR1ZX = -K2RR * K4LX + KARZX;
    IPZF = K2RR * TMP2 + IPZ;
%
    IPZF1 = invMIRR * IPZF;
    IPZF2 = MSRR * IPZF1;
    KR2ZX = -invMIRR * KR1ZX;
    Z1ZX = MSRR * KR2ZX;
%
    IPZF = IPZF2;
%
%   Determine trim unknows
%
%   assemble rectangular system matrix
    TMAT_ACC = MSRR;
%   select independent dofs
%   structural suport accelerations     
%   set acceleration dofs
    acc_dof = beam_model.Aero.Trim.FM.Fixed(7:12);
    indexa = find(acc_dof); % fixed acc dofs
    UDD(:,ntrim) = beam_model.Aero.Trim.FM.Value(7:12)';
%
    index_acc = setdiff([1:6], indexa); % free acc dofs 
    if ~isempty(indexa)
      IPZF = IPZF - MSRR(:, indexa) * UDD(indexa,ntrim);
      TMAT_ACC = MSRR(:, index_acc); % clean for constrained acc
    end
%
    Z1ZX_FM = Z1ZX(:,1:5);
    fm_dof = beam_model.Aero.Trim.FM.Fixed(2:6);
    nc = find(beam_model.Aero.Trim.CS.MPC == 0); % get master rotations
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
      NAME = beam_model.Aero.lattice.Control.Name;
    end
%
    TMAT = [TMAT_ACC, -Z1ZX_FM, -Z1ZX_CS]; 
%
    switch (STRIM)
      case 1  % simmetric trim
        TMAT = TMAT(FMDOF,:);
        IPZF = IPZF(FMDOF);
      case -1 % antisimmetric trim
        TMAT = TMAT(FMDOF,:);
        IPZF = IPZF(FMDOF);
    end 
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
      UDD(index_acc,ntrim) = sol([1:length(index_acc)]);
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
  FM0 = NDIM * IPZF2;
  if ntrim==1
    beam_model.Res.Aero.RStab_Der = get_stab_der(FMDER, HINGEMAT); 
    beam_model.Res.Aero.RIntercept = get_aero_intercept(FM0, results0.HINGE);
    beam_model.Res.Aero.RStab_Der.Control.Name = {};
    if (~isempty(nr))
      beam_model.Res.Aero.RStab_Der.Control.Name = beam_model.Aero.lattice_vlm.Control.Name(nr);
    end
    beam_model.Res.Aero.RTrim_sol = store_trim_sol(fid, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
    fprintf(outp, '\n\nRIGID TRIM RESULTS\n');
    dummy = store_trim_sol(outp, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
    print_intercept(outp, beam_model.Res.Aero.RIntercept);
  else
    beam_model.Res.Aero.DStab_Der = get_stab_der(FMDER, []);
    beam_model.Res.Aero.DIntercept = get_aero_intercept(FM0, []);
    beam_model.Res.Aero.DStab_Der.Control.Name = {};
    if (~isempty(nr))
      beam_model.Res.Aero.DStab_Der.Control.Name = beam_model.Aero.lattice_vlm.Control.Name(nr);
    end 
    beam_model.Res.Aero.DTrim_sol = store_trim_sol(fid, UDD(:,2), UX(:,2), beam_model.Res.Aero.DStab_Der.Control.Name);
    fprintf(outp, '\n\nELASTIC TRIM RESULTS\n');
    dummy = store_trim_sol(outp, UDD(:,2), UX(:,2), beam_model.Res.Aero.DStab_Der.Control.Name);
    print_intercept(outp, beam_model.Res.Aero.DIntercept);
  end 
  fprintf(fid,'\ndone.');
  end % trim loop%
% Determine divergence dynamic pressure for the restrained aircraft if required
%
 ndof2 = ndof;
 ndof = size(Kax,1);
  beam_model.Res.Aero.DIVERG_Q = 0;
%
  if (beam_model.Param.DIVERG > 0)
    fprintf(fid, '\n Solving for unrestrained aeroelastic divergence...');
    DIV_FOUND = false;
    HR = [eye(length(rdof)); D];
    IR = eye(ndof) - HR*inv(MSRR)*HR'*[Mrr, Mrl; Mlr,Mll]; % inertia relief matrix
    AF = IR * [zeros(6,ndof); zeros(length(ldof),6),RFLEX_MAT] * IR'; % mean axes flex matrix
%
    [DMODE, dynp] = eig(eye(ndof), AF*[Qaarr, Qaarl; Qaalr, Qaall]./QINF);
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
        SOL = zeros(ndof, 1); 
        SOL(ldof,1) = DMODE(length(rdof)+1:end,mindex(k));
        SOL(rdof,1) = DMODE(1:length(rdof),mindex(k));
        %
        if ~isempty(beam_model.RBE2.ID)
            SOL = RBE2disp(beam_model.RBE2,SOL,ndof2);
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
  ndof = ndof2;
% Recover structural displacements and rotations
%
  UR = -M4RR * UDD(:,ntrim) - K4LX * UX(:,ntrim) - TMP2; 
  UL = -AMLR * UDD(:,ntrim) - ARLR * UR - ALX * UX(:,ntrim) + UINTL;
%     
  SOL =  zeros(size(K,1),1); SOL(rdof,1) = UR; SOL(ldof,1) = UL;  
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
  beam_model.Res.Aero.KaxDOF = KaxDOF;
%
  beam_model.Res.Aero.Fa0tot(:,1) = Fa0e - KaxDOF * UX(:,1);

  DS = zeros(size(Fa0,1),1);
  for k=1:ngrid
    index = find(beam_model.Node.DOF(k,:));
    if (~isempty(index))
      DS(beam_model.Node.DOF(k,index),1) = gdef(k,index);
    end
  end
  beam_model.Res.Aero.Fa0tot(:,2) = Fa0e - KaxDOF * UX(:,2) + QaaDOF * DS;
  clear KaxDOF;
  clear QaaDOF;
  %
  % update final vector
  index = find(beam_model.Aero.Trim.CS.MPC);
  for k=1:length(index)
      beam_model.Res.CS.Value(1,index(k)) = beam_model.Aero.Trim.CS.Coeff(index(k)) * beam_model.Res.CS.Value(1,beam_model.Aero.Trim.CS.MPC(index(k)));
      beam_model.Res.CS.Value(2,index(k)) = beam_model.Aero.Trim.CS.Coeff(index(k)) * beam_model.Res.CS.Value(2,beam_model.Aero.Trim.CS.MPC(index(k)));
  end
  fprintf(fid, '\n - Updating vlm model in Aero.lattice_defo...'); 
  beam_model.Aero.lattice_vlm = rotate_control_surf(beam_model.Aero.ref, beam_model.Aero.state, beam_model.Aero.geo, ...
      beam_model.Aero.lattice_vlm, beam_model.Res.CS.Value(2,:), ...
      beam_model.Aero.lattice_vlm.Control.Hinge,beam_model.Aero.ID);

  if (SPLINE_TYPE==1)
    beam_model.Aero.lattice_defo = update_vlm_mesh1(beam_model.Node, SOL, beam_model.Aero);
  else
     beam_model.Aero.lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord + beam_model.Res.NDispl(:,1:3), ...
        beam_model.Res.NRd, beam_model.Aero);
  end
  fprintf(fid, 'done.'); 
  fprintf(fid, '\n - Solution summary exported to %s file.', outf);
  beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
  beam_model.Aero.lattice = [];
  beam_model.Res.CPaero.State   =  Fa_State_aero;
  beam_model.Res.CPaero.Control =  FCtot_aero;
  beam_model.Res.CPaero.Defo    =  CPaeroDef;
  beam_model.Res.CPaero.F0 =  zeros(npa*3,1);
  beam_model.Res.CPaero.F0(1:3:end,1) = results0.F(:,1);
  beam_model.Res.CPaero.F0(2:3:end,1) = results0.F(:,2);
  beam_model.Res.CPaero.F0(3:3:end,1) = results0.F(:,3);
  beam_model.Res.Gamma.Gamma_P = GAMMA_P;
  beam_model.Res.Gamma.Gamma_I = GAMMA_I;
%
	fprintf(fid, '\n\ncompleted.\n\n');
%
% RUN gust
%
  if (beam_model.Aero.Trim.Extra.Value(1)~=0.0)
    solve_free_lin_gust(TRIM_INDEX, outp); 
  end
%
% RUN landing
%
  if ( beam_model.Aero.Trim.Extra.Fixed(2) && beam_model.Aero.Trim.Extra.Fixed(3) ) 
    solve_free_lin_land(TRIM_INDEX, outp); 
  end
%	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
%
print_aeroderRE(outp, beam_model.Res.Aero.RStab_Der, beam_model.Res.Aero.DStab_Der, nr);
%
if (nbody)
  FUSD = []; FUSL = [];
else
  [FUSD, FUSL] = RecoverFusGeoDataFromBeamModel(beam_model);
end
[beam_model.Res.Aero.RStability, beam_model.Res.Aero.DStability] = get_stab_marginRE(beam_model.Aero.geo.ref_point, beam_model.WB.CG, ...
    CREF, SREF, FUSD, FUSL, beam_model.Res.Aero.RStab_Der, beam_model.Res.Aero.DStab_Der, outp);
%
fclose(outp);
%
end		
%
function Z1ZXC = apply_der_corr(Z1ZX, EXTDER)
  nr = size(Z1ZX,1);
  nc = size(Z1ZX,2);
  Z1ZXC = ones(nr,nc);
  index = find(EXTDER);
  Z1ZXC(index) = EXTDER(index)./Z1ZX(index);
end

function Fa = gf_transfer_aero_nodal_ext(INFO, DOF, NODE, AERO, results, BODYINT)

% generalized aero forces vector on structural master nodes
Fa = zeros(INFO.ndof, 1);
% check 
ncaero = INFO.ncaero;
nbody = INFO.nbaero;
ngrid = INFO.ngrid;
%
SPLINE_TYPE = INFO.spline_type;
%
if ncaero

  if (SPLINE_TYPE == 1)
    Faero = zeros(3*length(results.F),1);
    Faero(1:3:end) = results.F(:,1); Faero(2:3:end) = results.F(:,2); Faero(3:3:end) = results.F(:,3);
    Imat = (AERO.Interp.Imv);
    Fa = Imat * Faero;
  else
	  % get structural nodal forces
	  Fxs = zeros(ngrid, 1);	     Fys = zeros(ngrid, 1);       Fzs = zeros(ngrid, 1);
    Imat = (AERO.Interp.Imv)';
    Fxa = results.F(:,1);        Fya = results.F(:,2);        Fza = results.F(:,3); 
	  Fxs = Imat * Fxa;            Fys = Imat * Fya;            Fzs = Imat * Fza;
	  % master node forces
	  Fxm = zeros(ngrid, 1); 	     Fym = zeros(ngrid, 1); 	    Fzm = zeros(ngrid, 1);
	  Mxm = zeros(ngrid, 1); 	     Mym = zeros(ngrid, 1); 	    Mzm = zeros(ngrid, 1);
	  for n = 1:ngrid
		  if (NODE.Index(n)) % if master node
			  Fxm(n) = Fxs(n);
			  Fym(n) = Fys(n);
			  Fzm(n) = Fzs(n);
      end
    end      
  % get slave forces
	  for n = 1:ngrid
		  if (~isempty(NODE.Aero.Coord(n).data)) % master
			  % forces on slaves
			  fxs = Fxs(NODE.Aero.Index(n).data);
			  fys = Fys(NODE.Aero.Index(n).data);
			  fzs = Fzs(NODE.Aero.Index(n).data);
			  Fxm(n) = sum(fxs) + Fxm(n);
			  Fym(n) = sum(fys) + Fym(n);
			  Fzm(n) = sum(fzs) + Fzm(n);
			  M = sum( cross(NODE.Aero.Coord(n).data', [fxs, fys, fzs], 2), 1);
			  Mxm(n) = Mxm(n) + M(1); 
			  Mym(n) = Mym(n) + M(2); 
			  Mzm(n) = Mzm(n) + M(3);
		  end 
	  end % end node loop
	  F = zeros(ngrid, 3);
	  M = zeros(ngrid, 3);
	  F = [Fxm, Fym, Fzm];
	  M = [Mxm, Mym, Mzm];
	  % store master node forces and moments in the correct DOF position
	  for n = 1:ngrid
		  if (NODE.Index(n)) % if master node
			  index = find(DOF(n, 1:3)); % get free dofs
			  pos = DOF(n, index);
			  Fa(pos, 1) = F(n, index);  % assembly
			  index = find(DOF(n, 4:6)); % get free dofs
			  if ~isempty(index)
				  indexoff = index + 3;
				  pos = DOF(n, indexoff);
				  Fa(pos, 1) = M(n, index);  % assembly
			  end
		  end
	  end
  end
else
	error('No aerodynamic VLM mesh available for aeroelastic calculation.');
end
% add body if required by BODYINT
if (nbody)
  for n=1:length(results.Fb)
    if BODYINT(n)
      Faero = zeros(3*length(results.Fb{n}),1);
      Faero(1:3:end) = results.Fb{n}(:,1); Faero(2:3:end) = results.Fb{n}(:,2); Faero(3:3:end) = results.Fb{n}(:,3);
      Imat = (AERO.body.Interp.Ic{n});
      Fb = Imat * Faero;
      Fa = Fa + Fb;
    end
  end
end
end