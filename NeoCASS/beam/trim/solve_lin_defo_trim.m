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
function solve_lin_defo_trim(varargin)

global beam_model;

fid = beam_model.Param.FID;

NSTEP = beam_model.Param.NSTEP;
RES_TOL = beam_model.Param.RES_TOL;
ALPHA = beam_model.Param.REL_FAC;
LOAD_SCALE = 1.0;
GRAV = beam_model.Param.G;
if (~isempty(find(beam_model.Param.MSOL == 144)))

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
    index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
		fprintf(fid,'\nSolving linear deformable aeroelastic trim analysis...');
%
%   determine firstly RIGID trim configuration
%
    solve_rigid_trim(TRIM_INDEX);

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
    % run aeroelastic interpolation
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
    aeroelastic_interface;
%
%   Initialize BEAM database for aeroelastic analysis
%
		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;
    % aerodynamic forces
    Fa = zeros(ndof, 2);
    % inertial forces
    Fi = zeros(ndof, 2);
		% update internal database
		beam_model.Res.SOL = 'Static linear aeroelastic';
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
		NODEPOS = zeros(ngrid, 3);
		% store updated node rotation
		beam_model.Res.NRd = beam_model.Node.R;
		
		fprintf(fid, 'done.');
    % stiffness matrix
		fprintf(fid, '\n - Assemblying stiffness matrix...');
		K = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam);
		fprintf(fid, 'done.');
%   SUPORT DOF
    if (beam_model.Info.cc_nspc)
      error('SPC case control card detected. This solver allows trim analysis for free-free aircraft.');
    end
    % assembly mass matrix
		fprintf(fid, '\n - Assemblying stiffness matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
		fprintf(fid, 'done.');
%   Stiffness matrix
    if (~isempty(beam_model.Param.SUPORT))
      [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, beam_model.Node, beam_model.Param.SUPORT, beam_model.Param.EPS);
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
%   RHS
		fprintf(fid,'\n - Setting system rhs...');
%
%    beam_model.Res.Aero = [];
		% set generalized forces
    fprintf(fid, '\n     External forces...');
		F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
                         beam_model.Node.DOF, LOAD_SCALE);
    fprintf(fid, 'done.');
    sindex = find(beam_model.Param.SUPORT(1,1) == beam_model.Node.ID);
%    beam_model.Aero.geo.ref_point = beam_model.Node.Coord(sindex,:); % set for moments calculations
%    beam_model.Aero.geo.CG = beam_model.Node.Coord(sindex,:); % set for angular velocities
beam_model.Aero.geo.ref_point = beam_model.WB.CG;
beam_model.Aero.geo.CG = beam_model.WB.CG;

%   Total external forces (Applied loads + Inertial + Follower)
    F = F + F_flw;
		fprintf(fid, '\n completed.');

		SOL = zeros(ndof, 1);
		
    RESIDUAL = zeros(NSTEP+1, 1);		
    RESIDUAL(1) = realmax;
    FAERO = zeros(NSTEP, 3);
%***********************************************************************************************************************
		% LOAD STEP

		for N = 2 : NSTEP+1

%***********************************************************************************************************************
			% grid coordinates
      NODEPOS = beam_model.Node.Coord + beam_model.Res.NDispl(:, 1:3);
      %
			fprintf(fid, '\n - Coupled iteration: %4d. ', N-1);
      %
      % run trim solver
      %
      if N>2
	      beam_model.Aero.lattice = update_vlm_mesh(beam_model.Node, NODEPOS, ...
                                         beam_model.Res.NRd, beam_model.Aero);
      % update control surfaces hinge lines
        beam_model.Aero.lattice = update_hinge_lines(beam_model.Aero.geo, ...
                                         beam_model.Aero.lattice);
        %
        restart_rigid_trim(beam_model.Res.Aero.x_hist(:,end), NDOF);
        beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
  %
      end
      % calculate inertial forces + gravitational
      % Direct access to rigid trim results database
      ACC = zeros(6,1);
      ROLL   = beam_model.Res.FM.Value(4); 
      PITCH  = beam_model.Res.FM.Value(5); 
      YAW    = beam_model.Res.FM.Value(6); 
      %
      PSI   = beam_model.Res.FM.Value(13);
      TETA  = beam_model.Res.FM.Value(14);
      PHI   = beam_model.Res.FM.Value(15);
      %
      VB = beam_model.Aero.state.AS.*[cos(TETA) 0 sin(TETA)]'; % rotate only to account for climb
      OMEGAINER = [ROLL; PITCH; YAW];
      OMEGAB = Rrotmat(PSI, TETA, PHI) * OMEGAINER;
      %
      ACC(2) = - (GRAV * cos(TETA) * sin(PHI) - OMEGAB(3)*VB(1) + OMEGAB(1)*VB(3));
      ACC(3) = - (GRAV * cos(TETA) * cos(PHI) - OMEGAB(1)*VB(2) + OMEGAB(2)*VB(1));
      %
      fprintf(fid, '\n     Inertial forces...');
      Fi1 = gf_iner_nodal(ndof, beam_model.Node.DOF, M, ACC);
      Fi2 = gf_iner_centrif(ndof, beam_model.Node.DOF, M,  beam_model.Node.Coord, beam_model.WB.CG, OMEGAB);
      Fi(:,2) = Fi1 + Fi2;
      fprintf(fid, 'done.');

      % deform lattice and rigidly ROTATE control surfaces 
      Fa(:,2) = gf_trim_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, NODEPOS, ...
                                 beam_model.Res.NRd, beam_model.Aero, beam_model.Res.state, beam_model.Res.CS, ...
                                 beam_model.Aero.lattice_defo, beam_model.Res.Aero.results);

      FAERO(N-1,:) = beam_model.Res.Aero.results.FORCES;
      % set external forces
      ALPHA = 0.8;
      Fext = F + (1 - ALPHA) .* (Fa(:,1) + Fi(:,1)) + ALPHA .* (Fa(:,2) + Fi(:,2));
      % store previous aerodynamic and inertial forces for relaxation
      Fa(:,1) = Fa(:,2);
      Fi(:,1) = Fi(:,2);
      %
      SOL = zeros(ndof, 1);
		  SOL(ldof) = Kll \ Fext(ldof,:);

		  gdef = zeros(beam_model.Info.ngrid, 6);

		  for n = 1:beam_model.Info.ngrid

			  dof = beam_model.Node.DOF(n, 1:6);
			  index = find(dof);

				  if ~isempty(index)

					  gdef(n, index) = SOL(dof(index));

				  end
		  end

		% store nodal displacement
		beam_model.Res.NDispl = gdef;
		% set delta Rot
		for n = 1:beam_model.Info.ngrid

			beam_model.Res.NRd(:,:,n) = Rmat(gdef(n, 4:6));

		end
%
%   update aerobeam nodes (if any)    
%
    if (beam_model.Info.nrbe0 > 0)
	       AERO_POS = update_aerobeam_node(beam_model.Info.ngrid, beam_model.Node, beam_model.Res.NDispl(:,1:3,1),...
                                         beam_model.Res.NRd-repmat(eye(3,3),[1,1,beam_model.Info.ngrid]));
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
    RESIDUAL(N)   = sqrt(sum(sum(beam_model.Res.NDispl(:,1:3) .* beam_model.Res.NDispl(:,1:3),2),1));
    DS_RES = abs(RESIDUAL(N) - RESIDUAL(N-1));
    RESIDUAL(N-1) = DS_RES;
		fprintf(fid,'Iter: %4d. Displ. variation: %g.', N-1, DS_RES);
%
    if (DS_RES < RES_TOL)
      fprintf(fid, '\n\t!! Aeroelastic solution converged. !!');
      break;
    end

  end

	% assembly BAR contributions directly in the undeformed position
	[beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
     get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl);

	% assembly BEAM contributions directly in the undeformed position
	[beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
     get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl);

  if (beam_model.Param.AUTOPLOT)
      % plot aerodynamic forces
      figure(100); close; figure(100); hold on;
      plot([1:N-1], FAERO(1:N-1,1),'r.-', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'LineWidth', 1);
      plot([1:N-1], FAERO(1:N-1,2),'b>-', 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'LineWidth', 1);
      plot([1:N-1], FAERO(1:N-1,3),'k^-', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'LineWidth', 1);
      grid; xlabel('Coupled iteration'); ylabel('Force'); title('NEOCASS - Aerodynamic force components plot'); 
      legend('Drag', 'Side', 'Lift', 'Location', 'BestOutside');
      % plot displacements norm
      figure(101); close; figure(101);
      semilogy([2:N-1], RESIDUAL(2:N-1,end), '-ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k', 'LineWidth', 1);
      grid; xlabel('Coupled iteration'); ylabel('Displacements variation norm'); title('NEOCASS - Structural displacement variation'); 

  end
	
	fprintf(fid, '\n\ncompleted.\n\n');
	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
		
end		
%***********************************************************************************************************************
function Fa = gf_trim_aero_nodal(INFO, DOF, NODE, NODEPOS, NODER, AERO, STATE, CS, lattice_defo, results)

% generalized aero forces vector on structural master nodes
Fa = zeros(INFO.ndof, 1);
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

% locally overwrite state struct to correctly update wake
AERO.state = STATE;

if ncaero

 	% update deformed mesh
%	lattice_defo = update_vlm_mesh(NODE, NODEPOS, NODER, AERO);
  % update control hinge lines
%  lattice_defo = update_hinge_lines(AERO.geo, lattice_defo);
  %
%  lattice_defo_noc = lattice_defo;
  % rotate control surfaces
%  lattice_defo = rotate_control_surf(AERO.ref, STATE, AERO.geo, ...
%                                     lattice_defo, CS.Value, lattice_defo.Control.Hinge);
	% set rigid body boundary conditions
	RHS = rigid_body_vlm_rhs(STATE, AERO.geo, lattice_defo);
  np = length(RHS);
  % get panel forces
%  [dwcond, Fxa, Fya, Fza] = get_panel_forces(RHS, AERO.geo, lattice_defo, STATE);

	% store rigid body forces
%	results.dwcond = dwcond;
%	results.FORCES = zeros(1,3);
%	results.MOMENTS = zeros(2,3);
%	results.F = zeros(np, 3);
%	results.M = zeros(np, 3);
%	results.FN = zeros(np, 1);
%  results.lattice_defo = lattice_defo_noc;  
Fxa = results.F(:,1);
Fya = results.F(:,2);
Fza = results.F(:,3);

%  results.FN = sum(lattice_defo.N .* results.F, 2);
%	[results.FORCES, results.MOMENTS, results.M] = get_rigid_body_forces(AERO.geo, lattice_defo, results.F); 

	% get structural nodal forces
	Fxs = zeros(ngrid, 1);	     Fys = zeros(ngrid, 1);       Fzs = zeros(ngrid, 1);
  Imat = (AERO.Interp.Imv)';
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

else

	error('No aerodynamic mesh available for aeroelastic calculation.');

end

end
%***********************************************************************************************************************
function restart_rigid_trim(sol, NDOF)

global beam_model

fid = beam_model.Param.FID;
THRUST_FOUND = false;
nc = beam_model.Aero.geo.nc;

% Newton Raphon iterations
EPS    = beam_model.Param.EPS;
TOLL   = beam_model.Param.RES_TOL;
NREL   = beam_model.Param.NITER;
%
for N=1:beam_model.Param.NSTEP
  % Actual residual and store solution
  fprintf(fid,'\n\nIter: %4d. ', N);
  RES0 = get_rigid_body_res(sol, true, NDOF);
  NORM = norm(RES0);
  % Check convergence
  if (NORM < TOLL)
    fprintf(fid, '\n Convergence criteria satisfied.');
    break;
  end
  % determine numeric jacobian
  %if (N==1 || ~mod(N,NREL)) % modified NR
  if (N==1) % modified NR
    
  fprintf(fid, '\nAssemblying numeric Jacobian...');
    beam_model.Res.Aero.J = zeros(NDOF, NDOF);
    for k = 1:NDOF
      dsol = sol;
      dsol(k) = sol(k) + EPS;
      RES = get_rigid_body_res(dsol, false, NDOF);
      beam_model.Res.Aero.J(:,k) = (RES-RES0)./EPS;
    end
    fprintf(fid, 'done.');
  end
  sol = sol -beam_model.Res.Aero.J\RES0;
end
%
% update solution status
beam_model.Res.Aero.x_hist = [beam_model.Res.Aero.x_hist, sol];
beam_model.Res.Aero.resid  = [beam_model.Res.Aero.resid, NORM];
%
fprintf(fid, '\ndone.\n\n');
%beam_model.Aero.lattice_undefo = beam_model.Aero.lattice;
%beam_model.Aero.lattice = beam_model.Aero.lattice_defo;
%clear beam_model.Aero.lattice_defo;
%
end
%***********************************************************************************************************************
function R = Rrotmat(PSI, TETA, PHI)

  R = [cos(TETA)*cos(PSI) cos(TETA)*sin(PSI) -sin(TETA);
       sin(PHI)*sin(TETA)*cos(PSI)-cos(PHI)*sin(PSI) sin(PHI)*sin(TETA)*sin(PSI)+cos(PHI)*cos(PSI) sin(PHI)*cos(TETA)
       cos(PHI)*sin(TETA)*cos(PSI)+sin(PHI)*sin(PSI) cos(PHI)*sin(TETA)*sin(PSI)-sin(PHI)*cos(PSI) cos(PHI)*cos(TETA)];
end
%***********************************************************************************************************************
function lattice_defo = update_hinge_lines(geo, lattice_defo)
  for nc = 1:geo.nc

    pind = lattice_defo.Control.Patch(nc);

    n1 = lattice_defo.Control.DOF(nc).data(1);
    ncptot = length(lattice_defo.Control.DOF(nc).data);
    n2 = lattice_defo.Control.DOF(nc).data(ncptot-geo.fnx(pind)+1);
    i1 = 1;
    i2 = 2;
    if (geo.b(pind)<0)
      i1 = 2;
      i2 = 1;
    end  
    %
    lattice_defo.Control.Hinge(nc,1,:) = lattice_defo.XYZ(n1,i1,:); 
    lattice_defo.Control.Hinge(nc,2,:) = lattice_defo.XYZ(n2,i2,:); 
    %
  end  

end
