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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function RES = restart_solve_free_lin_trim(beam_model, TRIM_INDEX, M, K, Fa0, Kax, Qaa)

%
fid = beam_model.Param.FID;
%

LOAD_SCALE = 1.0;
EPS = D2R(1); % perturbation value to extract aerodynamic derivatives

if (~isempty(find(beam_model.Param.MSOL == 144)))

    nc = beam_model.Aero.geo.nc;

%
%   select trim case
%
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)

		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);

		if ~isempty(index)		
      if (beam_model.Aero.Trim.Type(index))
        NDOF = 3;
        STRIM = true;
      else
        NDOF = 6;
        STRIM = false;
      end

			% set flight mechanics trim params
			beam_model.Aero.Trim.FM = get_free_body_trim_params(beam_model.Aero.Trim.NC(index), ...
        beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
      [beam_model.Aero.state.alpha, beam_model.Aero.betha, beam_model.Aero.state.P, beam_model.Aero.state.Q, beam_model.Aero.state.R] = ...
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
		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;
		% create intermediate storage struct
		% store bar internal forces
		beam_model.Res.Bar.CForces = zeros(2, 6, nbar);
		beam_model.Res.Beam.CForces = zeros(2, 6, nbeam);
		% store bar internal strains and curvatures
		beam_model.Res.Bar.CStrains = zeros(2, 6, nbar);
		beam_model.Res.Beam.CStrains = zeros(2, 6, nbeam);
		% store bar stresses
		beam_model.Res.Bar.CStresses = zeros(2, 4, nbar);;
		beam_model.Res.Beam.CStresses = zeros(2, 4, nbeam);;
    beam_model.Res.Bar.CSM = [];
    beam_model.Res.Beam.CSM = [];
		% store nodal displacement
		beam_model.Res.NDispl = zeros(ngrid, 6);
		NODEPOS = beam_model.Node.Coord;
		% store updated node rotation
		beam_model.Res.NRd = beam_model.Node.R;
    %
%   RHS
    %
		F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
                         beam_model.Node.DOF, LOAD_SCALE);
    F = F + F_flw + Fa0;
    beam_model.Res.Aero = [];
%
%   SUPORT DOF
    if (beam_model.Info.cc_nspc)
      error('SPC case control card detected. This solver allows trim analysis for free-free aircraft.');
    end
%   Stiffness matrix
    if (~isempty(beam_model.Param.SUPORT))
      [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, beam_model.Node, beam_model.Param.SUPORT, EPS);
      nr = find(KEPS < EPS);
    else
      error('No SUPORT card given.');
    end
%   Provisory
    if ( (size(beam_model.Param.SUPORT,1)>1) || (length(num2str(beam_model.Param.SUPORT(1,2)))~=6))
      error('Provisory: one one suport point with 6 dofs is enabled at the moment.');
    end
%   Mass matrix
    Mll = M(ldof, ldof); Mlr = M(ldof, rdof); Mrr = M(rdof, rdof); Mrl = M(rdof, ldof);
%   Rigid body mass matrix for the SUPORTED DOFs
    mr = Mrr + Mrl*D + D'*Mlr + D'*Mll*D;
    MSRR = mr;
%
%   Aerodynamic influence matrix
%***********************************************************************************************************************
%
    % store stability derivatives
    CREF = beam_model.Aero.ref.C_mgc;
    BREF = beam_model.Aero.ref.b_ref;
    SREF = beam_model.Aero.ref.S_ref;
    VREF = beam_model.Aero.state.AS;
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
%***********************************************************************************************************************
    beam_model.Res.NDispl = zeros(ngrid, 6);
		beam_model.Res.NRd = beam_model.Node.R;
    %
    Kaxl = Kax(ldof,:);
    Kaxr = Kax(rdof,:);
    %
    Qaall = Qaa(ldof, ldof);
    Qaalr = Qaa(ldof, rdof);
    Qaarl = Qaa(rdof, ldof);
    Qaarr = Qaa(rdof, rdof);
%
  for ntrim = 1:2

    if ntrim == 1 % switch off deformability
      Kall = Kll;
      Kalr = Klr;
      Karl = Krl;
      Karr = Krr;
    else
      Kall = Kll - Qaall;
      Kalr = Klr - Qaalr;
      Karl = Krl - Qaarl;
      Karr = Krr - Qaarr;
    end
%
    invKall = inv(Kall);
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
    index_acc = setdiff([1:6], indexa); % free acc dofs 
    if ~isempty(indexa)
      IPZF = IPZF - MSRR(:, indexa) * UDD(indexa,ntrim);
      TMAT_ACC = MSRR(:, index_acc); % clean for constrained acc
    end
%
    Z1ZX_FM = Z1ZX(:,1:5);
    fm_dof = beam_model.Aero.Trim.FM.Fixed(2:6);
    nc = find(beam_model.Aero.Trim.CS.MPC == 0); % get master rotations
    UX = zeros(length(nc)+5,2);
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
      cs_dof = beam_model.Aero.Trim.CS.Fixed(nc); % get fixed deflections
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
    if (STRIM)
      TMAT = TMAT([1,3,5],:);
      IPZF = IPZF([1,3,5]);
    end

    sol = inv(TMAT) * IPZF;
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
    end
%   recover solution cs
    if (~isempty(index_cs))
      UX(index_cs+5,ntrim) = sol([sol_offset+1:sol_offset+length(index_cs)]);
      sol_offset = sol_offset + length(index_cs);
      beam_model.Res.CS.Value(ntrim, :) = beam_model.Aero.Trim.CS.Value;
      beam_model.Res.CS.Value(ntrim, free_cs) = UX(index_cs+5,ntrim);
    end
% store stability derivatives 
  FMDER = NDIM * Z1ZX;
  FM0 = NDIM * IPZF2;
  if ntrim==1
    beam_model.Res.Aero.RStab_Der = get_stab_der(FMDER);
    beam_model.Res.Aero.RIntercept = get_aero_intercept(FM0);
    beam_model.Res.Aero.RTrim_sol = store_trim_sol(fid, UDD, UX(:,1), NAME);
  else
    beam_model.Res.Aero.DStab_Der = get_stab_der(FMDER);
    beam_model.Res.Aero.DIntercept = get_aero_intercept(FM0);
    beam_model.Res.Aero.DTrim_sol = store_trim_sol(fid, UDD, UX(:,2), NAME);
  end
  end % trim loop
%
% Determine divergence dynamic pressure for the restrained aircraft if required
%
  if (beam_model.Param.DIVERG)
    DIV_FOUND = false;
    [DMODE, dynp] = eig(full(Kll), Qaall./QINF);

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
    beam_model.Res.Aero.DIVERG_Q = 0;
    [beam_model.Res.Aero.DIVERG_Q, IND] = min(e);
    if (~isempty(IND))
      DIV_FOUND = true;
      beam_model.Res.Aero.DIVERG_MODE = zeros(ndof, 1);
      SOL = zeros(ndof, 1); SOL(rdof,1) = 0; SOL(ldof,1) = DMODE(:,mindex(IND));
      gdef = zeros(beam_model.Info.ngrid, 6);
      for n = 1:beam_model.Info.ngrid
	      dof = beam_model.Node.DOF(n, 1:6);
	      index = find(dof);
	      if ~isempty(index)
		      gdef(n, index) = SOL(dof(index));
	      end
      end
      % store nodal displacement
      beam_model.Res.NDispl(:,:,2) = gdef;
      % set delta Rot
      for n = 1:beam_model.Info.ngrid

	      beam_model.Res.NRd(:,:,n,2) = Rmat(gdef(n, 4:6));

      end
      if (beam_model.Info.nrbe0 > 0)
	         AERO_POS = update_aerobeam_node(beam_model.Info.ngrid, beam_model.Node, beam_model.Res.NDispl(:,1:3,2),...
                                           beam_model.Res.NRd(:,:,:,2)-repmat(eye(3,3),[1,1,beam_model.Info.ngrid]));
          % update coord database with slave nodes position
          for n=1:beam_model.Info.ngrid
      	    ne = length(beam_model.Node.Aero.Index(n).data);
      	    if ne
		          beam_model.Res.NDispl(beam_model.Node.Aero.Index(n).data, 1:3, 1) = AERO_POS(n).data';
      	    end
          end
          clear AERO_POS;
      end

    end
  end % divergence calculation
%
% Recover structural displacements and rotations
%
  UR = -M4RR * UDD(:,ntrim) - K4LX * UX(:,ntrim) - TMP2; 
  UL = -AMLR * UDD(:,ntrim) - ARLR * UR - ALX * UX(:,ntrim) + UINTL;
  SOL = zeros(ndof, 1); SOL(rdof,1) = UR; SOL(ldof,1) = UL;
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
%
%   update aerobeam nodes (if any)    
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

  beam_model.Aero.lattice = [];
  RES = beam_model.Res;

	fprintf(fid, '\n\ncompleted.\n\n');
	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
		
end		
%***********************************************************************************************************************
function [D, Kll, Klv, Kvv, Kvl, rdof, ldof, EPS] = get_suport_shapes(K, NODE, SUPORT, MODE_AMPL)
%
  nnodes = size(SUPORT,1);
  index = [];
  for i=1:nnodes
    m = find(SUPORT(i,1) == [NODE.ID]);
    dof = num2str(SUPORT(i,2));
    for k=1:length(dof)
      index = [index, NODE.DOF(m, str2num(dof(k)))];
    end
  end
  nc = size(K,2);
  col = setdiff([1:nc], index);
  %
  Kll = K(col, col);
  Klv = K(col, index);
  Kvv = K(index, index);
  Kvl = K(index, col);
  %
  D = -inv(Kll)*Klv;
  %
  V     = zeros(length(index), 1);
  Vloc  = zeros(length(col),1);
  RMODE = zeros(nc,length(index));
  EPS   = zeros(length(index),1);
  for n=1:length(index)
    V(:) = 0.0;
    V(n) = MODE_AMPL;
    RMODE(col, n) = D*V;
    RMODE(index(n),n) = MODE_AMPL;
    EPS(n) = RMODE(:,n)' * K * RMODE(:,n);
  end
%
  rdof = index;
  ldof = col;
end
%***********************************************************************************************************************
function [Fxa, Fya, Fza] = paero_force(AERO, lattice_defo, GAMMA_P, GAMMA_I)

	% set rigid body boundary conditions
	RHS = rigid_body_vlm_rhs(AERO.state, AERO.geo, lattice_defo);
  np = length(RHS);
    % get panel forces
  [dwcond, Fxa, Fya, Fza] = get_aero_forces(RHS, AERO.geo, lattice_defo, AERO.state, GAMMA_P, GAMMA_I);

end
%***********************************************************************************************************************
function [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(geo, lattice, state)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;

dwcond = 0;

[np vor_length dim] = size(lattice.VORTEX);

if vor_length ~= 8
  error('Wrong vortex struct dimension.');
end

[dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, 1, symmxz, symmxy);
[dwcond, GAMMA_P2, GAMMA_I] = assembly_vlm_mat(lattice, 2, symmxz, symmxy);
%
GAMMA_P = inv(GAMMA_P);

end
%***********************************************************************************************************************
function [dwcond, Fx, Fy, Fz] = get_aero_forces(RHS, geo, lattice, state, GAMMA_P, GAMMA_I)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;

dwcond = 0;

[np vor_length dim] = size(lattice.VORTEX);

Fx = zeros(np,1); Fy = zeros(np,1); Fz = zeros(np,1);

if vor_length ~= 8
  error('Wrong vortex struct dimension.');
end

gamma = GAMMA_P * RHS;
%
VX = zeros(np, 3); VY = zeros(np, 3); VZ = zeros(np, 3); VD = zeros(np, 3);
%
VX = GAMMA_I(:,:,1) * gamma;
VY = GAMMA_I(:,:,2) * gamma; 
VZ = GAMMA_I(:,:,3) * gamma;

b1 = vor_length / 2;
VD(:,:) = (lattice.VORTEX(:, b1+1, :) - lattice.VORTEX(:, b1, :));

wind = state.AS.*([cos(state.alpha)*cos(state.betha) sin(state.betha) sin(state.alpha)*cos(state.betha)]);

VFLOW = repmat(wind, np, 1) - [VX VY VZ];

OMEGA = [state.P state.Q state.R];
if norm(OMEGA)
  VBODY = cross((lattice.COLLOC - repmat(geo.CG, np, 1)),...
        repmat(OMEGA, np, 1), 2);
  VFLOW = VFLOW + VBODY;
end

F = zeros(np, 3);
F = state.rho .* cross(VFLOW, VD, 2);

Fx = F(:,1).*gamma; Fy = F(:,2).*gamma; Fz = F(:,3).*gamma;

end
%***********************************************************************************************************************
function Fa = gf_tranfer_aero_nodal(INFO, DOF, NODE, AERO, Fxa, Fya, Fza)

% generalized aero forces vector on structural master nodes
Fa = zeros(INFO.ndof, 1);
results = [];
% check 

try
	AERO.Interp.Ic;
catch
	error('Collocation points interface matrix not available.');
end

try
	AERO.Interp.In;
catch
	error('Panel nodes interface matrix not available.');
end

try
	AERO.Interp.Iv;
catch
	error('Vortex points interface matrix not available.');
end

try
	AERO.Interp.Imv;
catch
	error('Midpoint vortex points interface matrix not available.');
end

ncaero = INFO.ncaero;
ngrid = INFO.ngrid;

if ncaero

	% get structural nodal forces
	Fxs = zeros(ngrid, 1);	     Fys = zeros(ngrid, 1);       Fzs = zeros(ngrid, 1);
  Imat = (AERO.Interp.Imv)';
	Fxs = Imat * Fxa;            Fys = Imat * Fya;            Fzs = Imat * Fza;

	% master node forces
	Fxm = zeros(ngrid, 1); 	     Fym = zeros(ngrid, 1); 	    Fzm = zeros(ngrid, 1);
	Mxm = zeros(ngrid, 1); 	     Mym = zeros(ngrid, 1); 	    Mzm = zeros(ngrid, 1);

	for n = 1:ngrid
	
		if (~isempty(NODE.Aero.Coord(n).data))
		
			% sum forces
			fxs = Fxs(NODE.Aero.Index(n).data);
			fys = Fys(NODE.Aero.Index(n).data);
			fzs = Fzs(NODE.Aero.Index(n).data);
			
			Fxm(n) = Fxm(n) + sum(fxs);
			Fym(n) = Fym(n) + sum(fys);
			Fzm(n) = Fzm(n) + sum(fzs);
			
			M = sum( cross(NODE.Aero.Coord(n).data', [fxs, fys, fzs], 2), 1);

			Mxm(n) = Mxm(n) + M(1); 
			Mym(n) = Mym(n) + M(2); 
			Mzm(n) = Mzm(n) + M(3);
		
		end 
			
		% add master node forces (if any)
		Fxm(n) = Fxm(n) + Fxs(n);
		Fym(n) = Fym(n) + Fys(n);
		Fzm(n) = Fzm(n) + Fzs(n);
	
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

else

	error('No aerodynamic mesh available for aeroelastic calculation.');

end

end
%***********************************************************************************************************************
function Stab_Der = get_stab_der(AEROMAT)

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
  Stab_Der.P_rate.dcs_dP = AEROMAT(2,3);  
  Stab_Der.P_rate.dcl_dP = AEROMAT(3,3);  
  %
  Stab_Der.P_rate.dcml_dP = AEROMAT(4,3);  
  Stab_Der.P_rate.dcmm_dP = AEROMAT(5,3);  
  Stab_Der.P_rate.dcmn_dP = AEROMAT(6,3);  
  %
  % DELTA Q rate
%  Stab_Der.Q_rate.dcd_dQ = AEROMAT(1,4);  
  Stab_Der.Q_rate.dcs_dQ = AEROMAT(2,4);  
  Stab_Der.Q_rate.dcl_dQ = AEROMAT(3,4);  
  %
  Stab_Der.Q_rate.dcml_dQ = AEROMAT(4,4);  
  Stab_Der.Q_rate.dcmm_dQ = AEROMAT(5,4);  
  Stab_Der.Q_rate.dcmn_dQ = AEROMAT(6,4);  
  %
  % DELTA R rate
%  Stab_Der.R_rate.dcd_dR = AEROMAT(1,5);  
  Stab_Der.R_rate.dcs_dR = AEROMAT(2,5);  
  Stab_Der.R_rate.dcl_dR = AEROMAT(3,5);  
  %
  Stab_Der.R_rate.dcml_dR = AEROMAT(4,5);  
  Stab_Der.R_rate.dcmm_dR = AEROMAT(5,5);  
  Stab_Der.R_rate.dcmn_dR = AEROMAT(6,5);
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
  end

end
%***********************************************************************************************************************
function INTER = get_aero_intercept(AEROMAT)

  INTER = [];
  INTER.cs0  = AEROMAT(2,1);
  INTER.cl0  = AEROMAT(3,1);
  INTER.cml0 = AEROMAT(4,1);
  INTER.cmm0 = AEROMAT(5,1);
  INTER.cmn0 = AEROMAT(6,1);

end
%***********************************************************************************************************************
function SOL = store_trim_sol(fid, UDD, UX, NAME)


  SOL = [];
  SOL.Control = [];
  SOL.ACC = UDD;
  SOL.Alpha = rad2deg(UX(1));
  SOL.Betha = rad2deg(UX(2));
  SOL.P = UX(3);
  SOL.Q = UX(4);
  SOL.R = UX(5);
  nc = 0;
%  fprintf(fid,'\n - X acc:      %g [m/s^2].', SOL.ACC(1));
%  fprintf(fid,'\n - Y acc:      %g [m/s^2].', SOL.ACC(2));
%  fprintf(fid,'\n - Z acc:      %g [m/s^2].', SOL.ACC(3));
%  fprintf(fid,'\n - P-DOT:      %g [rad/s^2].', SOL.ACC(4));
%  fprintf(fid,'\n - Q-DOT:      %g [rad/s^2].', SOL.ACC(5));
%  fprintf(fid,'\n - R-DOT:      %g [rad/s^2].\n', SOL.ACC(6));

%  fprintf(fid,'\n - Alpha:      %g [deg].', SOL.Alpha);
%  fprintf(fid,'\n - Sideslip:   %g [deg].', SOL.Betha);
%  fprintf(fid,'\n - Roll rate:  %g [rad/s].', SOL.P);
%  fprintf(fid,'\n - Pitch rate: %g [rad/s].', SOL.Q);
%  fprintf(fid,'\n - Yaw rate:   %g [rad/s].', SOL.R);
  for n=6:size(UX,1)
    nc = nc+1;
    SOL.Control(nc) = rad2deg(UX(n));   
%    fprintf(fid,'\n - Control %s:   %g [deg].', NAME{nc}, SOL.Control(nc));
  end

end
