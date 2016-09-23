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
function solve_buckling

global beam_model;

fid = beam_model.Param.FID;

%if (~isempty(find(beam_model.Param.MSOL == 105)))

		fprintf(fid,'\nSolving non-linear static analysis...');

		ngrid = beam_model.Info.ngrid;
		nbar =  beam_model.Info.nbar;
		nbeam =  beam_model.Info.nbeam;
		ndof =  beam_model.Info.ndof;

		% update interal database
		beam_model.Res.SOL = 'Static linear buckling';

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
%***********************************************************************************************************************
		% erase rotations
		beam_model.Res.NDispl(:, 4:6) = zeros(ngrid, 3);

		K = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, ...
                    beam_model.Bar, beam_model.Beam);

		SOL = K \ F; % solve linear solution
		% get DELTA s and DELTA g
		[DS, DR] = get_nodal_displ(ngrid, beam_model.Node.DOF, SOL); % ok
		% find displacements and rotations
		beam_model.Res.NDispl = DS; 

		% Total Lagrangian node coords
		NODEPOS = beam_model.Node.Coord(:, 1:3) + beam_model.Res.NDispl(:,1:3);
		% update node rotations
		NR = update_node_rot(ngrid, DR, NR);
		% update BAR rotations
%		BARR = update_bar_rot(nbar, DR, beam_model.Bar.Conn, BARR, DS); % ok
		% update BEAM rotations
%		BEAMR = update_bar_rot(nbeam, DR, beam_model.Beam.Conn, BEAMR, DS); % ok


		[beam_model.Res.Bar.CForces, beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CStresses, beam_model.Res.Bar.CSM] = ...
      get_bar_force_strain(beam_model.Info.nbar, beam_model.Bar, beam_model.PBar, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl,0);

		% assembly BEAM contributions directly in the undeformed position
		[beam_model.Res.Beam.CForces, beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CStresses, beam_model.Res.Beam.CSM] = ...
      get_bar_force_strain(beam_model.Info.nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Mat, beam_model.Node, beam_model.Res.NDispl,0);


		BARF =  store_global_elem_force(nbar, BARR, beam_model.Res.Bar.CForces);
		BEAMF = store_global_elem_force(nbeam, BEAMR, beam_model.Res.Beam.CForces);


		% assembly BAR internal forces
%		[beam_model.Res.Bar.CStrains, beam_model.Res.Bar.CForces]   = get_bar_strain_force(nbar, beam_model.Bar, beam_model.Node, ...
%				                                                          beam_model.Res.NDispl, NR, BARR, BARPO, BARKR); % ok
%beam_model.Res.Bar.CStrains(:,:,end)
		% assembly BEAM internal forces
%		[beam_model.Res.Beam.CStrains, beam_model.Res.Beam.CForces] = get_bar_strain_force(nbeam, beam_model.Beam, beam_model.Node, ...
%				                                                          beam_model.Res.NDispl, NR, BEAMR, BEAMPO, BEAMKR); % ok

		F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, NR, beam_model.Node.DOF, 1.0);
		% assembly BAR jacobian
		[ibar, jbar, vbar] =       sg_j_assembly(1,nbar, beam_model.Bar, BARR, ...
				                                  beam_model.Node.Coord, beam_model.Node.R, NODEPOS, NR, ...
												  beam_model.Node.DOF, beam_model.Res.Bar.CForces, BARF, beam_model.Res.NDispl, ...
												beam_model.F_FLW, 1.0, beam_model.Param.LOAD, beam_model.Info);


		% assembly BEAM jacobian
		[ibeam, jbeam, vbeam] =    sg_j_assembly(1,nbeam, beam_model.Beam, BEAMR, ...
				                                  beam_model.Node.Coord, NR, NODEPOS, NR, ...
												  beam_model.Node.DOF, beam_model.Res.Beam.CForces, BARF, beam_model.Res.NDispl, ...
												beam_model.F_FLW, 1.0, beam_model.Param.LOAD, beam_model.Info);

		JA = sparse([ibar; ibeam], [jbar; jbeam], [vbar; vbeam], ndof, ndof);
%save pippo.mat JA
		% assembly BAR jacobian
		[ibar, jbar, vbar] =       sg_j_assembly(2,nbar, beam_model.Bar, BARR, ...
				                                  beam_model.Node.Coord, beam_model.Node.R, NODEPOS, NR, ...
												  beam_model.Node.DOF, beam_model.Res.Bar.CForces, BARF, beam_model.Res.NDispl, ...
												beam_model.F_FLW, 1.0, beam_model.Param.LOAD, beam_model.Info);

		% assembly BEAM jacobian
		[ibeam, jbeam, vbeam] =    sg_j_assembly(2,nbeam, beam_model.Beam, BEAMR, ...
				                                  beam_model.Node.Coord, NR, NODEPOS, NR, ...
												  beam_model.Node.DOF, beam_model.Res.Beam.CForces, BARF, beam_model.Res.NDispl, ...
												beam_model.F_FLW, 1.0, beam_model.Param.LOAD, beam_model.Info);

		JB = sparse([ibar; ibeam], [jbar; jbeam], [vbar; vbeam], ndof, ndof);
%
    [SHAPES, Pcr] = eig(full(JB), -full(JA));
% 
    Pcr = diag(Pcr);
    index = find(abs(Pcr)~=Inf);
    Pcr = Pcr(index);
    SHAPES = SHAPES(:,index);
    %[Pcr, index] = sort(abs(Pcr));
    %SHAPES = SHAPES(:,index);

    beam_model.Param.NROOTS=size(SHAPES,2);
    beam_model.Res.Pcr = Pcr(1:beam_model.Param.NROOTS);

    MODES = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, beam_model.Param.NROOTS, SHAPES);
		beam_model.Res.NDispl = MODES(:,:,:);
		beam_model.Res.NRd = zeros(3, 3, beam_model.Info.ngrid, beam_model.Param.NROOTS);
		crossR = zeros(3, 3, beam_model.Info.ngrid, beam_model.Param.NROOTS);
		% set MODAL delta Rot
		for n = 1:beam_model.Info.ngrid
      for m = 1:beam_model.Param.NROOTS
			  beam_model.Res.NRd(:,:,n,m) = Rmat(MODES(n, 4:6, m));
        crossR(:,:,n,m) = crossm(MODES(n, 4:6, m));
      end
		end
%   update elements
    for m = 1:beam_model.Param.NROOTS
		  beam_model.Res.Bar.R(:,:,:,:,m) = update_bar_rot(beam_model.Info.nbar, beam_model.Res.NRd(:,:,:,m), ...
                                           beam_model.Bar.Conn, beam_model.Bar.R, beam_model.Res.NDispl(:,:,m));
		  beam_model.Res.Beam.R(:,:,:,:,m) = update_bar_rot(beam_model.Info.nbeam, beam_model.Res.NRd(:,:,:,m), ...
                                           beam_model.Beam.Conn, beam_model.Beam.R, beam_model.Res.NDispl(:,:,m));

    end





    fprintf('\ndone.\n\n');
%	else

	%	error('SOL 600 must be given in input file to run non-linear static analysis.');

%	end		

end
%***********************************************************************************************************************
function [gdef, DELTAROT] = get_nodal_displ(ngrid, DOF, SOL)

	gdef = zeros(ngrid, 6);
	DELTAROT = zeros(3,3, ngrid);
	
	for n = 1:ngrid

		dof = DOF(n, 1:6);
		index = find(dof);

		if ~isempty(index)

			gdef(n, index) = SOL(dof(index));

		end		
		
		DELTAROT(:,:,n) = Rmat(gdef(n, 4:6));
		
	end

end
%***********************************************************************************************************************
function [i, j, v] = sg_j_assembly(FLAG, nbar, BAR, BARR, NODE0, NODER0, COORD, NODER, DOF, FORCES, BARF, SOL, FLW, SCALE, LOADS, INFO)
				
i = [];
j = [];
v = [];

switch(FLAG)
  case 1

  % assembly rotation contribution
  [ir, jr, vr] = set_DR_mat(nbar, BAR, BARR, COORD, NODER, DOF, BARF, SOL);
  % assembly arm contribution
  [ia, ja, va] = set_DA_mat(nbar, BAR, BARR, DOF, NODER, FORCES, SOL);
  %[it, jt, vt] = set_DT_mat_nl([1:nbar], BAR, BARR, DOF, NODE0, COORD, NODER0, NODER, SOL);
  % assembly internal forces contribution

  i = [ir; ia];
  j = [jr; ja];
  v = [vr;-va];

  case 2

  [it, jt, vt] = set_DT_mat([1:nbar], BAR, BARR, DOF, NODE0, COORD, NODER0, NODER);
  % assembly follower forces contributions
  [iflw, jflw, vflw] = set_DFLW_mat(LOADS, INFO, FLW, NODER, DOF, SOL, SCALE);

  i = [it; iflw];
  j = [jt; jflw];
  v = [vt; vflw];

  end

end
