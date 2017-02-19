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
function solve_lin_static

global beam_model;

fid = beam_model.Param.FID;

if (~isempty(find(beam_model.Param.SOL == 101)))

		fprintf(fid,'\nSolving linear static analysis...');

		% assembly stifness matrix
		fprintf(fid, '\n\tAssemblying stiffness matrix...');

		%K = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam);

        [K] = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam, beam_model.Celas);

        
		fprintf(fid, 'done.');

		K_rank = 1/condest(K);

		if (K_rank < 10*eps)

			fprintf(fid, '\n !! Stiffness matrix is nearly singular. Make sure enough contraints are set to avoid null-energy mechanisms. !!\n');

		end

		fprintf(fid,'\n\tSetting system rhs...');

    % set generalized forces
		F_lin = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
		% set follower forces
		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, beam_model.Node.DOF, 1.0);

		F = F_lin + F_flw;

    % look for GRAV load
    if (norm(beam_model.Param.GRAV) > 0.0)
      M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
      ACC = zeros(6,1); 
      ACC(1:3, 1) = beam_model.Param.GRAV;
      Fi = gf_iner_nodal(beam_model.Info.ndof, beam_model.Node.DOF, M, ACC);
      F = F + Fi;
    end

		fprintf(fid, 'done.');

		% use UMFPACK for sparse system solution
		fprintf(fid,'\n\tSolving system...');

		SOL = K \ F;
		fprintf(fid, 'done.');

		gdef = zeros(beam_model.Info.ngrid, 6);

		for n = 1:beam_model.Info.ngrid

			dof = beam_model.Node.DOF(n, 1:6);
			index = find(dof);

				if ~isempty(index)

					gdef(n, index) = SOL(dof(index));

				end
		end
save p2.mat SOL

		fprintf(fid,'\n\tStoring analysis results..');

		% update interal database
		beam_model.Res.SOL = 'Static linear';
		% store nodal displacement
		beam_model.Res.NDispl = gdef;
		beam_model.Res.NRd = zeros(3, 3, beam_model.Info.ngrid);
		% set delta Rot
		for n = 1:beam_model.Info.ngrid

			beam_model.Res.NRd(:,:,n) = Rmat(gdef(n, 4:6));

		end

		% store bar internal forces
		beam_model.Res.Bar.CForces = [];
		% store beam internal forces
		beam_model.Res.Beam.CForces = [];
		% store bar strains and curvatures
		beam_model.Res.Bar.CStrains = [];
		% store beam strains and curvatures
		beam_model.Res.Beam.CStrains = [];
		% store bar stresses
		beam_model.Res.Bar.CStresses = [];
		beam_model.Res.Bar.CSM = [];
		% store beam stresses
		beam_model.Res.Beam.CStresses = [];
		beam_model.Res.Beam.CSM = [];

		% assembly BAR contributions directly in the undeformed position
		[beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
      get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, ...
      beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

		% assembly BEAM contributions directly in the undeformed position
		[beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
      get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, ...
      beam_model.Res.NDispl, beam_model.Param.FUSE_DP);

		fprintf(fid, 'done.');

		fprintf(fid,'\ndone.\n\n');

save sol.mat SOL


	else

		error('SOL 101 must be given in input file to run linear static analysis.');

	end
end
