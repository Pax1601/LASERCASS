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
function solve_nlin_static

global beam_model;

fid = beam_model.Param.FID;

NSTEP = beam_model.Param.NSTEP;
NITER = beam_model.Param.NITER;
RES_TOL = beam_model.Param.RES_TOL;

% load step amplitude
DSCALE = 1 / NSTEP;

if (~isempty(find(beam_model.Param.MSOL == 600)))

		fprintf(fid,'\nSolving non-linear static analysis...');

		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;

		% update interal database
		beam_model.Res.SOL = 'Static non-linear';

		% create intermediate storage struct
		fprintf(fid, '\n\tSetting internal database...');
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

		fprintf(fid,'\n\tSetting system rhs...');

		% set generalized forces
		F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
    %
    if (norm(beam_model.Param.GRAV) > 0.0)
      M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
      ACC = zeros(6,1); 
      ACC(1:3, 1) = beam_model.Param.GRAV;
      Fi = gf_iner_nodal(beam_model.Info.ndof, beam_model.Node.DOF, M, ACC);
      F = F + Fi;
    end

		fprintf(fid, 'done.');

		NODEPOS = beam_model.Node.Coord; % current node coordinates
		NR = beam_model.Res.NRd;         % current node angular pos
		BARR = beam_model.Res.Bar.R;     % current bar angular pos
		BEAMR = beam_model.Res.Beam.R;   % current beam angular pos

		SOL = zeros(ndof, 1);
		
		% create array to store element forces in global reference frame
		BARF = []; BEAMF = [];
		
		BARF =  allocate_barf(nbar);
		BEAMF = allocate_barf(nbeam);
		
    RESIDUAL = zeros(NSTEP, NITER+1);

%***********************************************************************************************************************
		% LOAD STEP

		for N = 1 : NSTEP

%***********************************************************************************************************************
			% erase rotations
			beam_model.Res.NDispl(:, 4:6) = zeros(ngrid, 3);
			
			fprintf(fid, '\n\tLoad step: %4d. ', N);
			LOAD_SCALE = N * DSCALE;
			Fext = LOAD_SCALE .* F;
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
     
      RESIDUAL(N, end) = K;

			beam_model.Res.NRd = NR;
			beam_model.Res.Bar.R = BARR;
			beam_model.Res.Beam.R = BEAMR;

			end
			
			fprintf(fid,'Iter: %4d. Residual: %.9f. Conv: %2d.', K, RNORM, CONV);
			% store curvatures
			BARKR = beam_model.Res.Bar.CStrains(:, 4:6, :);			
			BEAMKR = beam_model.Res.Beam.CStrains(:, 4:6, :);

			% store global element forces 
			BARF =  store_global_elem_force(nbar, BARR, beam_model.Res.Bar.CForces);
			BEAMF = store_global_elem_force(nbeam, BEAMR, beam_model.Res.Beam.CForces);

		end
		
	% delete nodal rotations
	
%	NODEPOS = beam_model.Res.NDispl(:,1:3);
%	beam_model.Res.NDispl = zeros(ngrid, 3);
%	beam_model.Res.NDispl = NODEPOS;
	
  % plot residual
  if (beam_model.Param.AUTOPLOT)
      figure(100); close; figure(100);
      niter = sum(RESIDUAL(:,end));
      x = [];
      y = [];
      cont = 0;
      for n = 1:N
        niter = RESIDUAL(n, end);
        x = [x, [cont+1: (cont + niter)]];
        y = [y, RESIDUAL(n, 1:niter)./RESIDUAL(n, 1)];
        cont = cont + niter;
      end
      semilogy(x, y, '-ko',  'MarkerSize', 4, 'MarkerFaceColor', 'k', 'LineWidth', 1);
      grid;
      xlabel('Iteration'); ylabel('Scaled residual');
      title('NEOCASS - Residual plot'); 
  end

	fprintf(fid, '\ndone.\n\n');
	
	else

		error('SOL 600 must be given in input file to run non-linear static analysis.');

	end		
		
end		
%***********************************************************************************************************************
