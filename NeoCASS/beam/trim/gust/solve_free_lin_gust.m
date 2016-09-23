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
%
% Run this solver after a first solution from solve_free_lin_trim
%
function solve_free_lin_gust(TRIM_INDEX, outp)

global beam_model;
%
fid = beam_model.Param.FID; 
%
LOAD_SCALE = 1.0;
EPS = D2R(0.001); % perturbation value to extract aerodynamic derivatives
%

if (~isempty(find(beam_model.Param.MSOL == 144)))

		fprintf(fid,'\nSolving linear static unrestrained trim...\n\n');
    fprintf(outp,'\n\nGUST MANEUVER\n');
    nc = beam_model.Aero.geo.nc;
%
%   select trim case
%
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
%
    NDOF = 1;
    STRIM = 1; 
    FMDOF = [3];
%
%   set flight mechanics trim params
    beam_model.Aero.Trim.Extra = check_extra_param(beam_model.Aero.Trim.NC(index), ...
      beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
		beam_model.Aero.Trim.FM = get_free_body_trim_params(beam_model.Aero.Trim.NC(index), ...
      beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
    [beam_model.Aero.state.alpha, beam_model.Aero.state.betha, beam_model.Aero.state.P, beam_model.Aero.state.Q, beam_model.Aero.state.R] = ...
       get_state_trim_vars(beam_model.Aero.Trim.FM);
    beam_model.Aero.state.alpha = D2R(beam_model.Aero.state.alpha);
    beam_model.Aero.state.betha = D2R(beam_model.Aero.state.betha);
%   make sure angles are converted
    beam_model.Aero.Trim.FM.Value(2) = D2R(beam_model.Aero.Trim.FM.Value(2));
    beam_model.Aero.Trim.FM.Value(3) = D2R(beam_model.Aero.Trim.FM.Value(3));
%   set Tornado state struct
    beam_model.Aero.state.ALT = beam_model.Aero.Trim.ALT(index);
    [beam_model.Aero.state.rho, p, T, a, mu] = ISA_h(beam_model.Aero.state.ALT);
    beam_model.Aero.state.AS = beam_model.Aero.Trim.Mach(index) * a;
    beam_model.Aero.state.Mach(1) = beam_model.Aero.Trim.Mach(index);
		beam_model.Aero.Trim.CS = get_control_surf_trim_params(beam_model.Aero.geo.nc, beam_model.Aero.Trim.NC(index), ...
      beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data, beam_model.Aero.lattice.Control);
    NAME = beam_model.Aero.lattice.Control.Name;
%
    beam_model.Aero.Trim.CS.MPC = [];   % store control surfaces contraint equations
    beam_model.Aero.Trim.CS.Coeff = []; % store control surfaces contraint coefficients
    % set constraints for control surfaces
    [beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Fixed] = ...
    set_constr_eq(nc, beam_model.Aero.lattice.Control, beam_model.Aero.Trim.CS.Fixed, beam_model.Info.nlink, beam_model.Aero.Trim.Link);
%
		ngrid = beam_model.Info.ngrid;
		nbar  =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof  =  beam_model.Info.ndof;
    SPLINE_TYPE = beam_model.Info.spline_type;
%
%   CLEAR PREVIOUS SOLUTION
%
		% create intermediate storage struct
		fprintf(fid, '\n - Setting internal database...');
		% store bar internal forces
		beam_model.Res.Bar.CForces  = zeros(2, 6, nbar);
		beam_model.Res.Beam.CForces = zeros(2, 6, nbeam);
		% store bar internal strains and curvatures
		beam_model.Res.Bar.CStrains  = zeros(2, 6, nbar);
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
    % stiffness matrix   
		fprintf(fid, '- Assemblying stiffness matrix...');
		[K] = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, ...
                                             beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam,...
                                             beam_model.Celas);
    fprintf(fid, 'done.');
    % assembly mass matrix 
		fprintf(fid, '\n- Assemblying mass matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
		fprintf(fid, 'done.'); 
%   RHS 
    if ~isempty(beam_model.RBE2.ID)
        K = RBE2Assembly(beam_model.RBE2,K);
        M = RBE2Assembly(beam_model.RBE2,M);
    end
    fprintf(fid,'\n- Setting system rhs...');
%   set generalized forces 
		Fg = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
                         beam_model.Node.DOF, LOAD_SCALE);
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
      error('Provisory: one suport point with 6 dofs is enabled at the moment.');
    end
    fprintf(fid, 'done.');
%   Mass matrix
    Mll = M(ldof, ldof); Mlr = M(ldof, rdof); Mrr = M(rdof, rdof); Mrl = M(rdof, ldof); 
%   Rigid body mass matrix for the SUPORTED DOFs 
    mr = Mrr + Mrl*D + D'*Mlr + D'*Mll*D;   
    MSRR = mr; 
%-------------------------------------------------------------------------------
%   Restart previus solver
%
%  
    Qaa = beam_model.Res.Aero.Qaa;
    Kax = beam_model.Res.Aero.Kax;
    Kaxl = Kax(ldof,:); 
    Kaxr = Kax(rdof,:);
%
    Qaall = Qaa(ldof, ldof);
    Qaalr = Qaa(ldof, rdof);
    Qaarl = Qaa(rdof, ldof);
    Qaarr = Qaa(rdof, rdof);
%  
  for ntrim = 1:2
%
%   get total aerodynamic force (zero, trim and aeroelastic)
%

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
%
    DUMMY = D'*Mll + Mrl;
    M2RR =  (D'*Mlr + Mrr) - DUMMY * ARLR;
    M3RR = -DUMMY * AMLR;
    K3LX = -DUMMY * ALX;
    DUMMYF = DUMMY;
%
    invM2RR = inv(M2RR);
    M4RR = invM2RR * M3RR;
    K4LX = invM2RR * K3LX;
%
    DUMMY = D'*Kall + Karl;
    K2RR = -DUMMY * ARLR + (D'*Kalr + Karr);
    KAZL = DUMMY;
%    KARZX = KAZL - KAXL * ALX;
    KARZX = (D'*Kaxl + Kaxr) - KAZL * ALX;
%
    M5RR = -K2RR * M4RR + MSRR;
    MIRR = -KAZL * AMLR + M5RR;
    invMIRR = inv(MIRR);
    KR1ZX = -K2RR * K4LX + KARZX;
%
    KR2ZX = -invMIRR * KR1ZX;
    Z1ZX = MSRR * KR2ZX;
%
%   GUST DATA
%
    CREF = beam_model.Aero.ref.C_mgc;
    SREF = beam_model.Aero.ref.S_ref;
    cn_ALPHA = beam_model.Res.Aero.RStab_Der.Alpha.dcl_dalpha;
    MASS = beam_model.WB.MCG(1,1); 
    mu_g = 2 * MASS / (CREF*SREF*cn_ALPHA*beam_model.Aero.state.rho);
    Kg = (0.88 * mu_g) / (5.3 + mu_g);
    KEAS = sqrt(beam_model.Aero.state.rho/1.225);
%
    VEAS = KEAS * beam_model.Aero.state.AS;
    VG   = beam_model.Aero.Trim.Extra.Value(1);
    DELTAN = 0.5 * 1.225 * (SREF/MASS) * cn_ALPHA * VEAS * Kg * VG;
%
%   Determine trim unknows
%
%   Access to previous acceleration values
    UDD(:,ntrim) = beam_model.Aero.Trim.FM.Value(7:12)';
%   Update URDD3
    UDD(FMDOF,ntrim) = UDD(FMDOF,ntrim) + DELTAN;
%   Print data
    if (ntrim==1)
      fprintf(fid,'\n - Flight speed (EAS):    %g [m/s].', VEAS);
      fprintf(fid,'\n - Gust speed (EAS):      %g [m/s].', VG);
      fprintf(fid,'\n - Attenuation factor:    %g [].', Kg);
      fprintf(fid,'\n - Delta load factor:     %g [].', DELTAN/beam_model.Param.G);
      fprintf(fid,'\n - Total load factor:     %g [].\n', UDD(FMDOF,ntrim)/beam_model.Param.G);
    end
%   Load previous solution and fix all trim variables
    beam_model.Aero.Trim.FM.Fixed(1:end) = 1;
%   SET ANGLEA as free
    beam_model.Aero.Trim.FM.Fixed(2) = 0; 
    acc_dof = beam_model.Aero.Trim.FM.Fixed(7:12);
    indexa = find(acc_dof); % fixed acc dofs
%   Move inertial contrib to rhs
%    index_acc = setdiff([1:6], indexa); % free acc dofs 
%    if ~isempty(indexa)
%      IPZF = IPZF - MSRR(:, indexa) * UDD(indexa,ntrim);
%    end
%    DRHS = IPZF; 
    Z1ZX_FM = Z1ZX(:,1:5);
%   find ANGLEA
    TMAT = -Z1ZX_FM(FMDOF,1); % alpha term wrt plunge
    DRHS = -MSRR(:, FMDOF) .* DELTAN;
%
    sol = DRHS(FMDOF)/TMAT;
    UX = zeros(size(K4LX,2),ntrim);
    UX(1,ntrim) = sol;
    beam_model.Res.FM.Value(ntrim, 2) = beam_model.Res.FM.Value(ntrim, 2) + sol;
    beam_model.Res.FM.Value(ntrim, 9) = UDD(FMDOF,ntrim);
%   recover solution fm
    if ntrim==1
      beam_model.Res.Aero.RTrim_sol.ACC = UDD(:,1);
      print_trim_sol(fid, beam_model.Res.FM.Value(ntrim, 9),beam_model.Res.FM.Value(ntrim, 2));
      beam_model.Res.Aero.RTrim_sol.Alpha = beam_model.Res.FM.Value(ntrim, 2) * 57.3;
      fprintf(outp, '\n\nCORRECTIONS TO RIGID TRIM\n');
      print_trim_sol(outp, beam_model.Res.FM.Value(ntrim, 9),beam_model.Res.FM.Value(ntrim, 2));
    else
      beam_model.Res.Aero.DTrim_sol.ACC = UDD(:,2);
      print_trim_sol(fid, beam_model.Res.FM.Value(ntrim, 9),beam_model.Res.FM.Value(ntrim, 2));
      beam_model.Res.Aero.DTrim_sol.Alpha = beam_model.Res.FM.Value(ntrim, 2) * 57.3;
      fprintf(outp, '\n\nCORRECTIONS TO ELASTIC TRIM\n');
      print_trim_sol(outp, beam_model.Res.FM.Value(ntrim, 9),beam_model.Res.FM.Value(ntrim, 2));
   end

  fprintf(fid,'\ndone.');
  end % trim loop%
% Determine divergence dynamic pressure for the restrained aircraft if required
%
% Recover structural displacements and rotations
%
    Fa0 = beam_model.Res.Aero.Fa0tot(:,ntrim);
    F = Fg + F_flw + Fa0 - beam_model.Res.Aero.KaxDOF(:,1) .* sol; 
    if ~isempty(beam_model.RBE2.ID)
       F = RBE2Assembly2(beam_model.RBE2,F);
    end   
%
    UINTL = invKall * F(ldof,1);
    TMP1 = DUMMYF * UINTL;
    TMP2 = invM2RR * TMP1;
%
  UR = -M4RR * UDD(:,ntrim) -TMP2; %- K4LX * UX(:,ntrim) - TMP2; 
  UL = -AMLR * UDD(:,ntrim) - ARLR * UR + UINTL;% - ALX * UX(:,ntrim) + UINTL;
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
  %
  % update final vector
  index = find(beam_model.Aero.Trim.CS.MPC);
  for k=1:length(index)
      beam_model.Res.CS.Value(2,index(k)) = beam_model.Aero.Trim.CS.Coeff(index(k)) * beam_model.Res.CS.Value(2,beam_model.Aero.Trim.CS.MPC(index(k)));
  end
  
  beam_model.Aero.lattice_vlm = rotate_control_surf(beam_model.Aero.ref, beam_model.Aero.state, beam_model.Aero.geo, ...
      beam_model.Aero.lattice_vlm, beam_model.Res.CS.Value(2,:), ...
      beam_model.Aero.lattice_vlm.Control.Hinge,beam_model.Aero.ID);
  if (SPLINE_TYPE==1)
    beam_model.Aero.lattice_defo = update_vlm_mesh1(beam_model.Node, SOL, beam_model.Aero);
  else
    beam_model.Aero.lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord + beam_model.Res.NDispl(:,1:3), ...
        beam_model.Res.NRd, beam_model.Aero);
  end
  beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
  beam_model.Aero.lattice = [];
%
	fprintf(fid, '\n\ncompleted.\n\n');
	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
		
end		
%***********************************************************************************************************************
function print_trim_sol(fid, UDD, AOA)

  fprintf(fid,'\n - Z acc:      %g [m/s^2].', UDD);
  fprintf(fid,'\n - Alpha:      %g [deg].', AOA*180/pi);
 
end
