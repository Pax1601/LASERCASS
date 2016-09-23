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
function TOT_DEF = solve_defo_mean_axes(DISPL, SUPORT)

global beam_model;

NSHAPES = size(DISPL,3);
TOT_DEF = zeros(beam_model.Info.ngrid, 6, NSHAPES);

fid = 1;
EPS = 1.0e-4;

% update interal database
beam_model.Res.SOL = 'Mean axes converter';
%
UDOF = int32(zeros(beam_model.Info.ngrid, 6));
% apply SPC set if required by the user
ndof = 0;
% count dof
for n=1:beam_model.Info.ngrid
	if beam_model.Node.Index(n) % structural node
		UDOF(n, :) = int32(zeros(1,6));
		UDOF(n, :) = int32([ndof+1 : ndof+6]);	
		ndof = ndof + 6;
	end
end	

bkndof = beam_model.Info.ndof
beam_model.Info.ndof = ndof;
ndof

K = st_lin_matrix(beam_model.Info, UDOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam);
M = ms_matrix(beam_model.Info, UDOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
%
[RMODE, KEPS] = get_suport_shapes(K, beam_model.Node, UDOF, SUPORT, 1.0);
nrsup = size(RMODE,2);
%
nr = find(KEPS < EPS);
if (~isempty(nr))
  fprintf(fid, '\nWarning: %d SUPORT rigid modes exceed deformation energy tolerance %g.', length(nr), EPS);
end
%

MRR = zeros(nrsup);
for k=1:nrsup
  for m = 1:nrsup
    MRR(k,m) = RMODE(:,k)'*M*RMODE(:,m);
  end
end

TOT_DEF = zeros(beam_model.Info.ngrid, 6, NSHAPES);
NRd = zeros(3, 3, beam_model.Info.ngrid, NSHAPES); 

for N = 1:NSHAPES

  NDEFO = zeros(beam_model.Info.ndof, 1);

  for l=1:beam_model.Info.ngrid
    for m=1:6 
      if (UDOF(l,m)~=0)
        NDEFO(UDOF(l,m),1) = DISPL(l,m,N);
      end
    end
  end
%

  qr = inv(MRR)*RMODE'*M*NDEFO
  fprintf(fid, '\n Suport amplitudes: %f.', qr);
%
  MEAN_DEFO = NDEFO - RMODE*qr;
%
  for l=1:beam_model.Info.ngrid
    for m=1:6 
      if (UDOF(l,m)~=0)
        TOT_DEF(l,m,N) = MEAN_DEFO(UDOF(l,m),1);
      end
    end
  end

		crossR = zeros(3, 3, beam_model.Info.ngrid);
		% set MODAL delta Rot
		for n = 1:beam_model.Info.ngrid
			  NRd(:,:,n,N) = Rmat(TOT_DEF(n, 4:6, N));
        crossR(:,:,n) = crossm(TOT_DEF(n, 4:6, N));
		end
    Res = [];

		Res.Bar.R(:,:,:,:,N) = update_bar_rot(beam_model.Info.nbar, NRd(:,:,:,N), beam_model.Bar.Conn, beam_model.Bar.R, TOT_DEF(:,:,N));
		Res.Beam.R(:,:,:,:,N) = update_bar_rot(beam_model.Info.nbeam, NRd(:,:,:,N), beam_model.Beam.Conn, beam_model.Beam.R, TOT_DEF(:,:,N));


    if (beam_model.Info.nrbe0 > 0)
      ngrid = beam_model.Info.ngrid;
		  fprintf(fid,'\n - Updating modal shapes for slave nodes...');

        AERO_POS = update_aerobeam_node(beam_model.Info.ngrid, beam_model.Node, TOT_DEF(:,1:3,N), crossR(:,:,:));
        % update coord database with slave nodes position
        for n=1:beam_model.Info.ngrid
      	  ne = length(beam_model.Node.Aero.Index(n).data);
      	  if ne
		        TOT_DEF(beam_model.Node.Aero.Index(n).data, 1:3, N) = AERO_POS(n).data';
      	  end
        end
        clear AERO_POS;
      end
  		fprintf(fid, 'done.');


  beam_model.Res.WB = [];
  % recover solution in matrix form
  [DS, DR] = get_nodal_displ(beam_model.Info.ngrid, UDOF, MEAN_DEFO); %
	% update BAR rotations
  beam_model.Res.Bar.R = update_bar_rot(beam_model.Info.nbar, DR, beam_model.Bar.Conn, beam_model.Bar.R , DS); % ok
	% update BEAM rotations
	beam_model.Res.Beam.R = update_bar_rot(beam_model.Info.nbeam, DR, beam_model.Beam.Conn, beam_model.Beam.R , DS);
  %
  COORD = beam_model.Node.Coord + TOT_DEF(:,1:3,N);

  beam_model.Res.Bar.Colloc = bar_defo_colloc(beam_model.Info.nbar, beam_model.Bar, UDOF, COORD, NRd(:,:,:,N));
  beam_model.Res.Beam.Colloc = bar_defo_colloc(beam_model.Info.nbeam, beam_model.Beam, beam_model.Node.DOF, COORD, NRd(:,:,:,N));
  beam_model.Res.WB = [];


  % calculate new mass matrix
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] = ...
     wb_set_conm_mass(beam_model.Info.nconm, beam_model.Node.Index, COORD, NRd, beam_model.Param.GRDPNT, beam_model.ConM);
  % set bar mass CG
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] =...
     wb_add_bar_mass(beam_model.Info.nbar, COORD, NRd(:,:,:,N), beam_model.Res.WB.CG, beam_model.Param.GRDPNT, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP, beam_model.Bar);
  % set beam mass CG
  [beam_model.Res.WB.CG, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP] =...
     wb_add_bar_mass(beam_model.Info.nbeam, COORD, NRd(:,:,:,N), beam_model.Res.WB.CG, beam_model.Param.GRDPNT, beam_model.Res.WB.MCG, beam_model.Res.WB.MRP, beam_model.Beam);
  % get principal axes
  [beam_model.Res.WB.MCG_pa, beam_model.Res.WB.R_pa] = wb_principal_axis(beam_model.Res.WB.MCG);

end
   beam_model.Info.ndof = bkndof;

end

function [RMODE, EPS] = get_suport_shapes(K, NODE, DOF, SUPORT, MODE_AMPL)
%
  nnodes = size(SUPORT,1);
  index = [];
  for i=1:nnodes
    m = find(SUPORT(i,1) == [NODE.ID]);
    dof = num2str(SUPORT(i,2));
    for k=1:length(dof)
      index = [index, DOF(m, str2num(dof(k)))];
    end
  end

  nc = size(K,2);
  col = setdiff([1:nc], index);
  %
  Kll = K(col, col);
  Klv = K(col, index);
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
end

function COLLOC = bar_defo_colloc(nbar, BAR, DOF, COORD, NRd)

  COLLOC = BAR.Colloc;
  for n=1:nbar
		n1 = BAR.Conn(n, 1);
		n2 = BAR.Conn(n, 2);
		n3 = BAR.Conn(n, 3);
		dof = [DOF(n1,1:6), DOF(n2,1:6), DOF(n3,1:6)];
		index = find(dof); % look for constraints
		% global offset	
		f1 = NRd(:,:,n1) * BAR.Offset(n, 1:3)';
		f2 = NRd(:,:,n2) * BAR.Offset(n, 4:6)';
		f3 = NRd(:,:,n3) * BAR.Offset(n, 7:9)';
		% global nodes
		c1 = f1 + COORD(n1, :)'; 
		c2 = f2 + COORD(n2, :)'; 
		c3 = f3 + COORD(n3, :)';
		COLLOC(:,:,n) = interp_colloc_pos(c1, c2, c3);
  end

end
