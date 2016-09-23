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
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
%
% This functions is specifically developed for GUESS to get aero forces from rigid trim
%
function outp = solve_free_lin_trim_guess_std(TRIM_INDEX, Res)
global beam_model;
%
if (~isempty(Res))
  SKIP = 1;
else
  SKIP = 0;
end
%
fid = beam_model.Param.FID;
LOAD_SCALE = 1.0;
EPS = D2R(1); % perturbation value to extract aerodynamic derivatives

    nc = beam_model.Aero.geo.nc;
    %
    %   select trim case
    %
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
    index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
    LOADI = beam_model.Aero.Trim.ID(index);    
    
    if ~isempty(index)
        
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
                FMDOF = 5;
            case -1 % full anti-simmetric trim
                NDOF = 3;
                STRIM = 1;
                FMDOF = [2,4,6];
            case -2 % roll spin
                NDOF = 1;
                STRIM = 1;
                FMDOF = 4;
            case -3 % yaw spin
                NDOF = 1;
                STRIM = 1;
                FMDOF = 6;
                %
        end
        IDMAN = beam_model.Aero.Trim.ID(index);
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
        %
    else
        error('Unable to find the required TRIM set %d.', IDMAN);
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
                
                fprintf(fid, '\n Warning: duplicated labels for AELINK card: %d.', ...
                    beam_model.Aero.Trim.Link.ID(dof(k)));
                
            end
            
            error('AELINK entries have duplicated labels.');
            
        end
    end
    
    beam_model.Aero.Trim.CS.MPC = [];   % store control surfaces contraint equations
    beam_model.Aero.Trim.CS.Coeff = []; % store control surfaces contraint coefficients
    
    % set output struct
    if ~isfield(beam_model,'Res')
        beam_model.Res = [];
    end
    beam_model.Res.SOL = 'Static linear unrestrained trim rigid';
    beam_model.Res.FM.Value = beam_model.Aero.Trim.FM.Value; % current flight mechanics solution
    beam_model.Res.FM.Fixed = beam_model.Aero.Trim.FM.Fixed;
    beam_model.Res.CS.Value = beam_model.Aero.Trim.CS.Value; % current control surfaces solution
    beam_model.Res.Extra.Fixed = beam_model.Aero.Trim.Extra.Fixed;
    beam_model.Res.Extra.Value = beam_model.Aero.Trim.Extra.Value;
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
        if NF_TOT<NDOF
            error('Number of degrees of freedom are less than number of DOF %d (%d).', NDOF, NF_TOT);
        else
            % 	    
            fprintf(fid, '\n  - Non linear trim solution...');
        end
    end
    %
    dotpos = strfind(beam_model.Param.FILE,'.');
    outf = [beam_model.Param.FILE(1:dotpos-1), '_man_', num2str(IDMAN),'.txt'];
    outp = fopen(outf, 'w');
    print_state(outp, beam_model.Aero.state); 
    print_attitude(outp, beam_model.Res.FM);
    print_trim_problem(outp, NF_TOT, NDOF);
    %
    ngrid = beam_model.Info.ngrid;
    ndof =  beam_model.Info.ndof;
    MSRR =  beam_model.WB.MRP;
    %   RHS
    if ~isfield(beam_model.Res,'Aero')
        beam_model.Res.Aero = [];
    end
    %   Aerodynamic influence matrix
    if (beam_model.Param.GRDPNT~=0)
      beam_model.Aero.geo.ref_point = beam_model.Node.Coord(beam_model.Param.GRDPNT,:); % set for moments calculations
      beam_model.Aero.geo.CG = beam_model.Node.Coord(beam_model.Param.GRDPNT,:);
    else
      beam_model.Aero.geo.ref_point = zeros(1,3);
      beam_model.Aero.geo.CG = zeros(1,3);
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
    % ONLY RIGID TRIM
    beam_model.Res.Aero.RStab_Der = [];
    beam_model.Res.Aero.RIntercept = [];
    beam_model.Res.Aero.RTrim_sol = [];
    %
    nbody = beam_model.Info.nbaero;
    if (SKIP == 0)
      %   1 case: static rigid load at NULL reference condition
      fprintf(fid,'\n  Aerodynamic database');
      fprintf(fid,'\n  - Reference rigid case...');
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
%-------------------------------------------------------------------------------
%     Extract VLM matrices
%
      nr = find(beam_model.Aero.Trim.CS.MPC == 0);
      nsurf = length(nr);
      np = length(beam_model.Aero.lattice_vlm.COLLOC);
      GAMMA_P = []; GAMMA_I = []; AEROMAT = zeros(6, 5+nsurf); DOFCT = [];
      FXMAT = zeros(np,6+nsurf); FYMAT = zeros(np,6+nsurf); FZMAT = zeros(np,6+nsurf);
      FXBMAT = {}; FYBMAT = {}; FZBMAT = {}; Fxb0 = {}; Fyb0 = {}; Fzb0 = {};
      if ~isempty(beam_model.Aero.lattice)
        [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(beam_model.Aero.geo, beam_model.Aero.lattice_vlm, dummy_aero.state);
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
%-------------------------------------------------------------------------------
%      [results0, GAMMA_MAT, COLLOC, NORM, NELEM, BTYPE] = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
%        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
%      [LMAT,UMAT] = lu(GAMMA_MAT);
%
      HINGEMAT = zeros(beam_model.Aero.geo.nc, 5+length(nr));
      [results0, GAMMA_MAT, NELEM] = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB, DOFCT);
      Fxa0 = results0.F(:,1); Fya0 = results0.F(:,2); Fza0 = results0.F(:,3);
      HM0 = results0.HINGE;
%      [Fxa0, Fya0, Fza0, results0] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      Fa0 = [results0.FORCES';results0.MOMENTS(1,:)'];
      FXMAT(:,1) = Fxa0; FYMAT(:,1) = Fya0; FZMAT(:,1) = Fza0;
      for i=1:nbody
        Fxb0{i} = results0.Fb{i}(:,1); Fyb0{i} = results0.Fb{i}(:,2); Fzb0{i} = results0.Fb{i}(:,3);
        FXBMAT{i}(:,1) = results0.Fb{i}(:,1); FYBMAT{i}(:,1) = results0.Fb{i}(:,2); FZBMAT{i}(:,1) = results0.Fb{i}(:,3);
      end
      fprintf(fid,'done.');
      %
      %   2 case: rigid body attitude variations
      %   Alpha
      fprintf(fid,'\n  - Alpha perturbation...');
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = EPS; dummy_aero.state.betha = 0;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
      Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%      [Fxa, Fya, Fza, results] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      AEROMAT(:,1) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
      FXMAT(:,2) = (Fxa-Fxa0)./EPS; FYMAT(:,2) = (Fya-Fya0)./EPS; FZMAT(:,2) = (Fza-Fza0)./EPS;
      for i=1:nbody
        FXBMAT{i}(:,2) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
        FYBMAT{i}(:,2) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
        FZBMAT{i}(:,2) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
      end
      if nc
        HINGEMAT(:,1) = (results.HINGE-results0.HINGE)./EPS;
      end
      fprintf(fid,'done.');
      %   Beta
      fprintf(fid,'\n  - Sideslip perturbation...');
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = EPS;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
      %results = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      %  dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
      Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%      [Fxa, Fya, Fza, results] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      AEROMAT(:,2) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
      FXMAT(:,3) = (Fxa-Fxa0)./EPS; FYMAT(:,3) = (Fya-Fya0)./EPS; FZMAT(:,3) = (Fza-Fza0)./EPS;
      for i=1:nbody
        FXBMAT{i}(:,3) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
        FYBMAT{i}(:,3) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
        FZBMAT{i}(:,3) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
      end
      if nc
        HINGEMAT(:,2) = (results.HINGE-results0.HINGE)./EPS;
      end
      fprintf(fid,'done.');
      %   P
      fprintf(fid,'\n  - Angular velocities...');
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
      dummy_aero.state.P = EPS;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
      %results = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
      %  dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
      Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%      [Fxa, Fya, Fza, results] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      AEROMAT(:,3) = (2*VREF).*([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./(EPS*BREF);
      FXMAT(:,4) = (Fxa-Fxa0)./EPS; FYMAT(:,4) = (Fya-Fya0)./EPS; FZMAT(:,4) = (Fza-Fza0)./EPS;
      for i=1:nbody
        FXBMAT{i}(:,4) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
        FYBMAT{i}(:,4) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
        FZBMAT{i}(:,4) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
      end
      if nc
        HINGEMAT(:,3) = (2*VREF/BREF)*(results.HINGE-results0.HINGE)./EPS;
      end
      %   Q
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = EPS;      dummy_aero.state.R = 0;
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
%      results = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
%        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
      Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%      [Fxa, Fya, Fza, results] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      AEROMAT(:,4) = (2*VREF).*([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./(EPS*CREF);
      FXMAT(:,5) = (Fxa-Fxa0)./EPS;
      FYMAT(:,5) = (Fya-Fya0)./EPS;
      FZMAT(:,5) = (Fza-Fza0)./EPS;
      for i=1:nbody
        FXBMAT{i}(:,5) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
        FYBMAT{i}(:,5) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
        FZBMAT{i}(:,5) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
      end
      if nc
        HINGEMAT(:,4) = (2*VREF/CREF)*(results.HINGE-results0.HINGE)./EPS;
      end
      %   set to adimensional angular speed
      %   R
      dummy_aero = beam_model.Aero;
      dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
      dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = EPS;
      results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
        dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
%      results = solve_vlm_body(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
%        dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
      Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%      [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
      %   set to adimensional angular speed
      AEROMAT(:,5) = (2*VREF).*([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./(EPS*BREF);
      FXMAT(:,6) = (Fxa-Fxa0)./EPS;
      FYMAT(:,6) = (Fya-Fya0)./EPS;
      FZMAT(:,6) = (Fza-Fza0)./EPS;
      for i=1:nbody
        FXBMAT{i}(:,6) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
        FYBMAT{i}(:,6) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
        FZBMAT{i}(:,6) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
      end
      if nc
        HINGEMAT(:,5) = (2*VREF/BREF)*(results.HINGE-results0.HINGE)./EPS;
      end
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
      LUMP_DOF = [];
      LUMP_COEFF = [];
%
      if (~isempty(nr))
        fprintf(fid,'\n  - Controls...');
        print_controls(outp, nr, beam_model.Aero.lattice_vlm.Control.Name, beam_model.Res.CS);
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
%
%            results = solve_vlm_body_re(beam_model.Aero.geo, dummy_aero.lattice, beam_model.Aero.body, ...
%                                    dummy_aero.state, beam_model.Param, LMAT, UMAT, NORM, COLLOC, GAMMA_I, NELEM, BTYPE, GAMMA_VEL);
%            results = solve_vlm_body(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
%                dummy_aero.state, beam_model.Param, GAMMA_P, GAMMA_I, GAMMA_VEL, Sij, DijY, DijZ, SijV, DijYV, DijZV, GAMMA_HB);
            results = solve_vlm_body_re(beam_model.Aero.geo, lattice_defo, beam_model.Aero.body, ...
              dummy_aero.state, beam_model.Param, GAMMA_I, GAMMA_VEL, GAMMA_MAT, NELEM, DOFCT);
            Fxa = results.F(:,1); Fya = results.F(:,2); Fza = results.F(:,3);
%            [Fxa, Fya, Fza, results] = paero_force(dummy_aero, lattice_defo, GAMMA_P, GAMMA_I);
            AEROMAT(:,5+k) = ([results.FORCES';results.MOMENTS(1,:)']-[results0.FORCES';results0.MOMENTS(1,:)'])./EPS;
            FXMAT(:,6+k) = (Fxa-Fxa0)./EPS;
            FYMAT(:,6+k) = (Fya-Fya0)./EPS;
            FZMAT(:,6+k) = (Fza-Fza0)./EPS;
            for i=1:nbody
              FXBMAT{i}(:,6+k) = (results.Fb{i}(:,1)-Fxb0{i})./EPS; 
              FYBMAT{i}(:,6+k) = (results.Fb{i}(:,2)-Fyb0{i})./EPS; 
              FZBMAT{i}(:,6+k) = (results.Fb{i}(:,3)-Fzb0{i})./EPS;
            end
            HINGEMAT(:,5+k) = (results.HINGE-results0.HINGE)./EPS;
            %
        end
        beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
%       tranform moments to coefficients
        HINGEMAT = HINGEMAT./(QINFS*CREF);
        results0.HINGE = results0.HINGE./(QINFS*CREF);
%       lump rows
        HINGEMAT = lump_hinge(HINGEMAT, LUMP_DOF, LUMP_COEFF);
        results0.HINGE = lump_hinge(results0.HINGE, LUMP_DOF, LUMP_COEFF);
        HM0 = results0.HINGE;
        fprintf(fid,'done.');
      end
    else
      Fa0 = Res.Aero.Fa0;
      HM0 = Res.Aero.HM0
      AEROMAT = Res.Aero.Kax;
      HINGEMAT = Res.Aero.Kah;
      GAMMA_P = Res.Aero.GAMMA_P;
      GAMMA_I = Res.Aero.GAMMA_I;
      FXMAT = Res.Aero.FXMAT;
      FYMAT = Res.Aero.FYMAT;
      FZMAT = Res.Aero.FZMAT;
      FXBMAT = Res.Aero.FXBMAT;
      FYBMAT = Res.Aero.FYBMAT;
      FZBMAT = Res.Aero.FZBMAT;
    end    
    %   Assembly derivatives
    %***********************************************************************************************************************
    %
    fprintf(fid,'\n Solving rigid aircraft trim condition (ID %d)...', IDMAN);
%
    F = gf_lin_nodal(LOADI, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
    Z1ZX = AEROMAT;
    IPZF = Fa0 ;%+ F;
    %   Determine trim unknows
    %
    %   assemble rectangular system matrix
    TMAT_ACC = MSRR;
    %   select independent dofs
    %   structural suport accelerations
    %   set acceleration dofs
    acc_dof = beam_model.Aero.Trim.FM.Fixed(7:12);
    indexa = find(acc_dof); % fixed acc dofs
    
    UDD(:,1) = beam_model.Aero.Trim.FM.Value(7:12)';
    index_acc = setdiff([1:6], indexa); % free acc dofs
    if ~isempty(indexa)
        IPZF = IPZF - MSRR(:, indexa) * UDD(indexa,1);
        TMAT_ACC = MSRR(:, index_acc); % clean for constrained acc
    end
    %
    Z1ZX_FM = Z1ZX(:,1:5);
    fm_dof = beam_model.Aero.Trim.FM.Fixed(2:6);
    nc = find(beam_model.Aero.Trim.CS.MPC == 0); % get master rotations
    UX = zeros(length(nc)+5,1);
    UX(1:5,1) = beam_model.Aero.Trim.FM.Value(2:6)';
    indexfm = find(fm_dof); % fixed fm dofs
    index_fm = setdiff([1:5], indexfm); % free fm dofs
    if ~isempty(indexfm)
        IPZF = IPZF + Z1ZX_FM(:, indexfm) * UX(indexfm,1);
        Z1ZX_FM = Z1ZX_FM(:, index_fm); % clean for constrained fm param
    end
    %
    Z1ZX_CS = [];
    index_cs = [];
    if (~isempty(nc))
        Z1ZX_CS = Z1ZX(:,6:end);
        cs_dof = beam_model.Aero.Trim.CS.Fixed(nc); % check if master deflections are fixed
        defl = beam_model.Aero.Trim.CS.Value(nc)';
        UX(6:end,1) = defl;
        indexcs = find(cs_dof); % fixed cs dofs
        
        index_cs = setdiff([1:length(nc)], indexcs); % free cs dofs
        if (~isempty(indexcs))
            UCS(:,1) = defl(indexcs);
            IPZF = IPZF + Z1ZX_CS(:, indexcs) * UCS(:,1);
            Z1ZX_CS = Z1ZX_CS(:, index_cs); % clean for constrained fm param
        end
        free_cs = intersect(nc,  find(beam_model.Aero.Trim.CS.Fixed==0));
        NAME = beam_model.Aero.lattice.Control.Name(nc);
    end
    %
    TMAT = [TMAT_ACC, -Z1ZX_FM, -Z1ZX_CS];
    %
    
    switch (STRIM)
        case 1  % simmetric trim
            TMAT = TMAT([1,3,5],:);
            IPZF = IPZF([1,3,5]);
        case -1 % antisimmetric trim
            TMAT = TMAT([2,4,6],:);
            IPZF = IPZF([2,4,6]);
    end
    %
    if NF_TOT ~= NDOF
        fprintf(fid, '\n Non linear solution required to solve trim problem.');
        funmin = @(x) handlemin(x,index_acc,index_fm,index_cs);
%         funcon = @(x) handlecon(x,TMAT,IPZF);
        options = optimset('Algorithm','active-set');
        sol = fmincon(funmin, zeros(size(TMAT,2),1), [] , [] ,full(TMAT) , IPZF, [], [], [], options);
    else
        sol = inv(TMAT) * IPZF;
    end
    sol_offset = 0;
    %   recover solution acceleration
    %
    if (~isempty(index_acc))
        UDD(index_acc,1) = sol([1:length(index_acc)]);
        sol_offset = sol_offset + length(index_acc);
        beam_model.Res.FM.Value(1, 7:12) = UDD(:,1)';
    end
    %   recover solution fm
    if (~isempty(index_fm))
        UX(index_fm,1) = sol([sol_offset+1:sol_offset+length(index_fm)]);
        sol_offset = sol_offset + length(index_fm);
        beam_model.Res.FM.Value(1, 2:6) = UX(1:5,1)';
    end
    %   recover solution cs
    beam_model.Res.CS.Value(1, :) = beam_model.Aero.Trim.CS.Value;
    if (~isempty(index_cs))
        UX(index_cs+5,1) = sol([sol_offset+1:sol_offset+length(index_cs)]);
        sol_offset = sol_offset + length(index_cs);
        beam_model.Res.CS.Value(1, free_cs) = UX(index_cs+5,1);
    end
    % store stability derivatives
    FMDER = NDIM * Z1ZX;
    FM0 = NDIM * Fa0;
    beam_model.Res.Aero.RStab_Der = get_stab_der(FMDER, HINGEMAT);
    beam_model.Res.Aero.RIntercept = get_aero_intercept(FM0, HM0);
    beam_model.Res.Aero.RStab_Der.Control.Name = {};
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    if (~isempty(nr))
        beam_model.Res.Aero.RStab_Der.Control.Name = beam_model.Aero.lattice_vlm.Control.Name(nr);
    end
    beam_model.Res.Aero.RTrim_sol = store_trim_sol(fid, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
    fprintf(outp, '\n\nRIGID TRIM RESULTS\n');
    dummy = store_trim_sol(outp, UDD(:,1), UX(:,1), beam_model.Res.Aero.RStab_Der.Control.Name);
    print_intercept(outp, beam_model.Res.Aero.RIntercept);
    print_hinge_intercept(outp, beam_model.Res.Aero.RIntercept.cmh0, beam_model.Res.Aero.RStab_Der.Control.Name);
    print_aeroderR(outp, beam_model.Res.Aero.RStab_Der, nr);
    % structure data
    index = find(beam_model.Aero.Trim.CS.MPC);
    for k=1:length(index)
        beam_model.Res.CS.Value(1,index(k)) = beam_model.Aero.Trim.CS.Coeff(index(k)) * beam_model.Res.CS.Value(1,beam_model.Aero.Trim.CS.MPC(index(k)));
    end
%   update lattice    
    beam_model.Aero.lattice_vlm = rotate_control_surf(beam_model.Aero.ref, beam_model.Aero.state, beam_model.Aero.geo, ...
        beam_model.Aero.lattice_vlm, beam_model.Res.CS.Value(1,:), ...
        beam_model.Aero.lattice_vlm.Control.Hinge);
    beam_model.Aero.lattice_defo = beam_model.Aero.lattice_vlm;
%
    beam_model.Res.Aero.Kax = AEROMAT;
    beam_model.Res.Aero.Kah = HINGEMAT;
    beam_model.Res.Aero.Fa0 = Fa0;
    beam_model.Res.Aero.HM0 = HM0;
    beam_model.Res.Aero.GAMMA_P = GAMMA_P;
    beam_model.Res.Aero.GAMMA_I = GAMMA_I;
    beam_model.Res.Aero.FXMAT  = FXMAT;
    beam_model.Res.Aero.FYMAT  = FYMAT;
    beam_model.Res.Aero.FZMAT  = FZMAT;
    beam_model.Res.Aero.FXBMAT = FXBMAT;
    beam_model.Res.Aero.FYBMAT = FYBMAT;
    beam_model.Res.Aero.FZBMAT = FZBMAT;
%   evaluate forces    
    dummy_aero = beam_model.Aero;
%
% RUN gust
%
    if (beam_model.Aero.Trim.Extra.Value(1)~=0.0)
      DALPHA = solve_free_lin_guess_std_gust(TRIM_INDEX, outp); 
    else
      DALPHA = 0;
    end
    UX(1) = UX(1) + DALPHA;
% recover panel forces
    Fxa = FXMAT(:,1); Fya = FYMAT(:,1); Fza = FZMAT(:,1); 
    for k=1:length(UX)
      Fxa = Fxa + FXMAT(:,k+1) * UX(k);
      Fya = Fya + FYMAT(:,k+1) * UX(k);
      Fza = Fza + FZMAT(:,k+1) * UX(k);
    end
    beam_model.Res.Aero.results.F = [Fxa, Fya, Fza];
    beam_model.Res.Aero.results.FN = sum(beam_model.Aero.lattice_vlm.N .* beam_model.Res.Aero.results.F, 2);
%
    FORCES = Fa0(1:3,1); MOMENTS = Fa0(4:6,1);
    for k=1:length(UX)
      FORCES = FORCES + AEROMAT(1:3,k) .* UX(k);
      MOMENTS = MOMENTS + AEROMAT(4:6,k) .* UX(k);
    end
    beam_model.Res.Aero.results.FORCES = FORCES';
    beam_model.Res.Aero.results.MOMENTS = MOMENTS';
% recover body forces
    for i=1:nbody
      Fxb = FXBMAT{i}(:,1); Fyb = FYBMAT{i}(:,1); Fzb = FZBMAT{i}(:,1); 
      for k=1:length(UX)
        Fxb = Fxb + FXBMAT{i}(:,k+1) * UX(k);
        Fyb = Fyb + FYBMAT{i}(:,k+1) * UX(k);
        Fzb = Fzb + FZBMAT{i}(:,k+1) * UX(k);
      end
      beam_model.Res.Aero.results.Fb{i} = [Fxb, Fyb, Fzb];
    end
%
    beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
    beam_model.Aero.lattice = [];
%
  fprintf(fid, '\n - Solution summary exported to %s file.', outf);
  fprintf(fid, '\n done.\n');
%
end
