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
function solve_nlin_aerostatic(varargin)

global beam_model;

fid = beam_model.Param.FID;

NSTEP = beam_model.Param.NSTEP;
NITER = beam_model.Param.NITER;
RES_TOL = beam_model.Param.RES_TOL;
ALPHA = beam_model.Param.REL_FAC;
LOAD_SCALE = 1.0;
% load step amplitude
DSCALE = 1 / NSTEP; 
 
if (~isempty(find(beam_model.Param.MSOL == 644)))

	fprintf(fid,'\nSolving linear static aeroelastic analysis...');
	if nargin == 1
		DUMMY_INDEX = varargin{1};
		if (length(DUMMY_INDEX)>1)
			fprintf(fid, '\nWarning: only the first trim case will be run. The following will be ignored.');
			fprintf(fid, '\n         Rerun the solver.')
		end
		TRIM_INDEX = DUMMY_INDEX(1);
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
		%
		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;
		% aerodynamic forces
		Fa = zeros(ndof, 2);

		% update interal database
		beam_model.Res.SOL = 'Static non-linear aeroelastic';

		% create intermediate storage struct
		fprintf(fid, '\n - Setting internal database...');
		% delta NODE rotation
		DR = zeros(3, 3, ngrid);
		% intermediate updated rotations R = DR * R0
		NR = zeros(3, 3, ngrid);
		BARR = zeros(3, 3, 5, nbar);
		BEAMR = zeros(3, 3, 5, nbeam);
		% store beam initial curvatures (assumed always as straight) and updated values
		BARKR = zeros(2, 3, nbar);
		BEAMKR = zeros(2, 3, nbeam);
		% store beam initial local strains
		BARPO = set_initial_PO(nbar, beam_model.Bar, beam_model.Node);
		BEAMPO = set_initial_PO(nbeam, beam_model.Beam, beam_model.Node);
		% store bar internal forces
		beam_model.Res.Bar.CForces = zeros(2, 6, nbar);
		beam_model.Res.Beam.CForces = zeros(2, 6, nbeam);
		% store bar internal strains and curvatures
		beam_model.Res.Bar.CStrains = zeros(2, 6, nbar);
		beam_model.Res.Beam.CStrains = zeros(2, 6, nbeam);

		% NODAL DISPLACEMENTS
		% store nodal displacement
		beam_model.Res.NDispl = zeros(ngrid, 6);

		% UPDATED ROTATIONS
		% store updated node rotation
		beam_model.Res.NRd = beam_model.Node.R;
		% store updated bar rotations
		beam_model.Res.Bar.R = beam_model.Bar.R;
		% store updated beam rotations
		beam_model.Res.Beam.R = beam_model.Beam.R;

		fprintf(fid, 'done.');

		fprintf(fid,'\n - Setting system rhs...');
%
		beam_model.Res.Aero = [];
		% set generalized forces
		fprintf(fid, '\n     External forces...');
		F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		fprintf(fid, 'done.');
		% assembly mass matrix
		M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
		% calculate inertial forces
		ACC = zeros(6,1); ACC = beam_model.Aero.Trim.FM.Value(7:12);
		fprintf(fid, '\n     Inertial forces...');
		Fi = gf_iner_nodal(ndof, beam_model.Node.DOF, M, ACC);
		fprintf(fid, 'done.');
%
		fprintf(fid, '\n completed.');

		NODEPOS = beam_model.Node.Coord; % current node coordinates
		NR = beam_model.Res.NRd;         % current node angular pos
		BARR = beam_model.Res.Bar.R;     % current bar angular pos
		BEAMR = beam_model.Res.Beam.R;   % current beam angular pos

		SOL = zeros(ndof, 1);

		% create array to store element forces in global reference frame
		BARF = []; BEAMF = [];

		BARF =  allocate_barf(nbar);
		BEAMF = allocate_barf(nbeam);

		RESIDUAL = zeros(NSTEP+1, NITER+2);
		RESIDUAL(1, end) = realmax;
		FAERO = zeros(NSTEP+1, 3);
%***********************************************************************************************************************
		% LOAD STEP

		for N = 2 : NSTEP+1

%***********************************************************************************************************************
			% erase rotations
			beam_model.Res.NDispl(:, 4:6) = zeros(ngrid, 3);
			
			fprintf(fid, '\n - Coupled iteration: %4d. ', N-1);

			% run aerodynamic solver
			[Fa(:,2), beam_model.Res.Aero] = gf_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, ...
                                         NODEPOS, NR, beam_model.Aero);
			FAERO(N,:) = beam_model.Res.Aero.FORCES;
			% set external forces
			Fext = Fi + F + (1 - ALPHA) .* Fa(:,1) + ALPHA .* Fa(:,2);
			% store previous aerodynamic forces for relaxation
			Fa(:,1) = Fa(:,2);
      
			CONV = 0;



			for K = 1 : NITER

				F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, NR, beam_model.Node.DOF, LOAD_SCALE);

				% assembly BAR jacobian
				[ibar, jbar, vbar] =       j_assembly(nbar, beam_model.Bar, BARR, ...
				                                      beam_model.Node.Coord, beam_model.Node.R, NODEPOS, NR, ...
				                                      beam_model.Node.DOF, beam_model.Res.Bar.CForces, BARF, beam_model.Res.NDispl, ...
				                                      beam_model.F_FLW, LOAD_SCALE, beam_model.Param.LOAD, beam_model.Info);


				% assembly BEAM jacobian
				[ibeam, jbeam, vbeam] =    j_assembly(nbeam, beam_model.Beam, BEAMR, ...
				                                      beam_model.Node.Coord, NR, NODEPOS, NR, ...
				                                      beam_model.Node.DOF, beam_model.Res.Beam.CForces, BARF, beam_model.Res.NDispl, ...
				                                      beam_model.F_FLW, LOAD_SCALE, beam_model.Param.LOAD, beam_model.Info);

				J = sparse([ibar; ibeam], [jbar; jbeam], [vbar; vbeam], ndof, ndof);

				% element forces
				FN = zeros(ngrid, 6);

				Fbar = get_actual_nodal_forces(ngrid, nbar, beam_model.Bar, BARR, beam_model.Node.DOF, NODEPOS, NR, beam_model.Res.Bar.CForces); % ok
				Fbeam = get_actual_nodal_forces(ngrid, nbeam, beam_model.Beam, BEAMR, beam_model.Node.DOF, NODEPOS, NR, beam_model.Res.Beam.CForces); % ok

				FN = Fbar + Fbeam;
				Fel = get_internal_forces(ndof, beam_model.Node.DOF, FN); % ok

				R = Fext + F_flw - Fel;

				RNORM = norm(R);
				RESIDUAL(N, K) = RNORM;

				SOL = J \ R; % solve Newton's problem
				% get DELTA s and DELTA g
				[DS, DR] = get_nodal_displ(ngrid, beam_model.Node.DOF, SOL); % ok
				% find displacements and rotations
				beam_model.Res.NDispl = beam_model.Res.NDispl + DS; % add to previous newton step solution

				% Total Lagrangian node coords
				NODEPOS = beam_model.Node.Coord(:, 1:3) + beam_model.Res.NDispl(:,1:3);
				% update node rotations
				NR = update_node_rot(ngrid, DR, NR);
				% update BAR rotations
				BARR = update_bar_rot(nbar, DR, beam_model.Bar.Conn, BARR, DS); % ok
				% update BEAM rotations
				BEAMR = update_bar_rot(nbeam, DR, beam_model.Beam.Conn, BEAMR, DS); % ok

				% assembly BAR internal forces
				[beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
				          get_bar_force_strain_NL(nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl, NR, BARR, BARPO, BARKR); % ok

				% assembly BEAM internal forces
				[beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
				          get_bar_force_strain_NL(nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl, NR, BEAMR, BEAMPO, BEAMKR); % ok

				if RNORM < RES_TOL

					CONV = 1;
					break;

				end

				beam_model.Res.NRd = NR;
				beam_model.Res.Bar.R = BARR;
				beam_model.Res.Beam.R = BEAMR;

			end
      
			RESIDUAL(N, end-1) = K;
			RESIDUAL(N, end)   = sqrt(sum(sum(beam_model.Res.NDispl(:,1:3) .* beam_model.Res.NDispl(:,1:3),2),1));
			DS_RES = abs(RESIDUAL(N, end) - RESIDUAL(N-1, end));
			RESIDUAL(N-1, end) = DS_RES;
			fprintf(fid,'Iter: %4d. Residual: %g. Displ. variation: %g. Conv: %2d.', K, RNORM, DS_RES, CONV);

			% store curvatures
			BARKR = beam_model.Res.Bar.CStrains(:, 4:6, :);			
			BEAMKR = beam_model.Res.Beam.CStrains(:, 4:6, :);

			% store global element forces 
			BARF =  store_global_elem_force(nbar, BARR, beam_model.Res.Bar.CForces);
			BEAMF = store_global_elem_force(nbeam, BEAMR, beam_model.Res.Beam.CForces);

			if (DS_RES < RES_TOL)
				fprintf(fid, '\n\t!! Aeroelastic solution converged. !!');
				break;
			end

		end

	% delete nodal rotations
	
	% NODEPOS = beam_model.Res.NDispl(:,1:3);
	% beam_model.Res.NDispl = zeros(ngrid, 3);
	% beam_model.Res.NDispl(:,1:3) = NODEPOS;

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

	if (beam_model.Param.AUTOPLOT)
		% plot residual
		figure(100); close; figure(100);
		x = [];
		y = [];
		cont = 0;
		for n = 2:N
			niter = RESIDUAL(n, end-1);
			x = [x, [cont+1: (cont + niter)]];
			y = [y, RESIDUAL(n, 1:niter)./RESIDUAL(2,1)];
			cont = cont + niter;
		end
		semilogy(x, y, '-ko',  'MarkerSize', 4, 'MarkerFaceColor', 'k', 'LineWidth', 1);
		grid;
		xlabel('Iteration'); ylabel('Scaled residual');
		title('NEOCASS - Residual plot'); 
		% plot aerodynamic forces
		figure(101); close; figure(101); hold on;
		plot([1:N-1], FAERO(2:N,1),'r.-', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'LineWidth', 1);
		plot([1:N-1], FAERO(2:N,2),'b>-', 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'LineWidth', 1);
		plot([1:N-1], FAERO(2:N,3),'k^-', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'LineWidth', 1);
		grid; xlabel('Coupled iteration'); ylabel('Force'); title('NEOCASS - Aerodynamic force components plot'); 
		legend('Drag', 'Side', 'Lift', 'Location', 'BestOutside');
		% plot displacements norm
		figure(102); close; figure(102);
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
		beam_model.Res.Aero.DStab_Der = stab_der_calc(fid, ALPHA0, BETA0, beam_model.Aero.state, ...
		                                beam_model.Aero.ref, beam_model.Aero.geo, ...
		                                beam_model.Res.Aero.lattice_defo, F0, M0);
%
	fprintf(fid, '\n\ncompleted.\n\n');

	else

		error('SOL 644 must be given in input file to run non-linear static analysis.');

	end
end
