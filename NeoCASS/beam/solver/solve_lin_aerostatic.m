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
function solve_lin_aerostatic(varargin)

global beam_model;

fid = beam_model.Param.FID;

NSTEP = beam_model.Param.NSTEP;
RES_TOL = beam_model.Param.RES_TOL;
ALPHA = beam_model.Param.REL_FAC;
LOAD_SCALE = 1.0;

if (~isempty(find(beam_model.Param.MSOL == 144)))

		fprintf(fid,'\nSolving linear static aeroelastic analysis...');
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
    % run aeroelastic interpolation
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
   aeroelastic_interface;

%
    nc = beam_model.Aero.geo.nc;
%
%   select trim case
%
		index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);

		if ~isempty(index)		

	    beam_model.Res.SOL = 'Static linear constrained aeroelastic';
      beam_model.Res.Aero = [];

    solve_vlm_rigid(TRIM_INDEX);

    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
    beam_model.Aero.lattice_vlm = beam_model.Aero.lattice_defo;

    state = beam_model.Res.state;
    %   Run aerodynamic data for undeformed configuration

    RSTAB = beam_model.Res.Aero.RStab_Der;
    F0 = beam_model.Res.Aero.results.FORCES; M0 = beam_model.Res.Aero.results.MOMENTS;
    ALPHA0 = beam_model.Aero.state.alpha;
    BETA0 = beam_model.Aero.state.betha;
    %
		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;
    % aerodynamic forces
    Fa = zeros(ndof, 2);
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
		K_rank = 1/condest(K);
		if (K_rank < 10*eps)
			fprintf(fid, '\n !! Stiffness matrix is nearly singular. Make sure enough contraints are set to avoid null-energy mechanisms. !!\n');
		end
    % assembly mass matrix
		fprintf(fid, '\n - Assemblying stiffness matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
		fprintf(fid, 'done.');
%   RHS
		fprintf(fid,'\n - Setting system rhs...');
%
    beam_model.Res.Aero = [];
		% set generalized forces
    fprintf(fid, '\n     External forces...');
		F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
                         beam_model.Node.DOF, LOAD_SCALE);
    fprintf(fid, 'done.');
    % calculate inertial forces
    ACC = zeros(6,1); ACC = beam_model.Aero.Trim.FM.Value(7:12);
    fprintf(fid, '\n     Inertial forces...');
    Fi = gf_iner_nodal(ndof, beam_model.Node.DOF, M, ACC);
    fprintf(fid, 'done.');
%   Total external forces (Applied loads + Inertial + Follower)
    F = F + Fi + F_flw;
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
			% grid rotation matrices

			fprintf(fid, '\n - Coupled iteration: %4d. ', N-1);

      % run aerodynamic solver

      [Fa(:,2), beam_model.Res.Aero] = gf_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, ...
                                         NODEPOS, beam_model.Res.NRd, beam_model.Aero);
      FAERO(N-1,:) = beam_model.Res.Aero.FORCES;
      % set external forces
      Fext = F + (1 - ALPHA) .* Fa(:,1) + ALPHA .* Fa(:,2);
      % store previous aerodynamic forces for relaxation
      Fa(:,1) = Fa(:,2);
      
		  SOL = K \ Fext;

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

    if (DS_RES < RES_TOL)
      fprintf(fid, '\n\t!! Aeroelastic solution converged. !!');
      break;
    end

  end

	% assembly BAR contributions directly in the undeformed position
	[beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
     get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, ...
     beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

	% assembly BEAM contributions directly in the undeformed position
	[beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
    get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, ...
    beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

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
	
  % calculate new stability derivatives for the deformed case
  %
  beam_model.Res.Aero.RStab_Der = [];
  % store rigid stability derivatives
  beam_model.Res.Aero.RStab_Der = RSTAB;
  beam_model.Res.Aero.DStab_Der = [];
  fprintf(fid, '\n');
  % calculate and store deformable stability derivatives
%  beam_model.Res.Aero.DStab_Der = stab_der_calc(fid, ALPHA0, BETA0, beam_model.Aero.state, ...
%                                  beam_model.Aero.ref, beam_model.Aero.geo, ...
%                                  beam_model.Res.Aero.lattice_defo, F0, M0);

  beam_model.Aero.lattice_defo = beam_model.Res.Aero.lattice_defo;
% 	beam_model.Aero.lattice_defo = update_vlm_mesh(beam_model.Node, NODEPOS, beam_model.Res.NRd, beam_model.Aero);
  beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
%  beam_model.Aero.lattice = [];
	fprintf(fid, '\n\ncompleted.\n\n');
	
	else

		error('SOL 144 must be given in input file to run linear static analysis.');

	end		
		
end		
