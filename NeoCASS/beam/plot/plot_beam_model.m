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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function fig = plot_beam_model(nfig, varargin)
%
%   DESCRIPTION: Plot beam model struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                nfig           integer    figure index to be opened
%                'aero_param'   OPTIONAL   Enable extra aerodynamic plots
%                                          'aero_param', [P_WAKE P_N]
%                                          P_WAKE boolean to enable plot of wake
%                                          P_N boolean to enable plot of normals
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                fig            integer    figure handler
%
%    REFERENCES:
%
%*******************************************************************************

function fig = plot_beam_model(nfig, varargin)

global beam_model;

WAKE_PLOT = 0;
NORMAL_PLOT = 0;
CHORD_FACTOR = 0.25;
PLOT_SYMM = 0;

if nargin > 1
	PARAM = varargin;
  if (strcmp(PARAM(1), 'aero_param')) 
	  AERO_P = PARAM{2};
	  WAKE_PLOT = AERO_P(1);
	  NORMAL_PLOT = AERO_P(2);
    PLOT_SYMM = AERO_P(3);
  end
end

fig = figure(nfig);
close(nfig);
% nfig = nfig+1;
% fig = figure(nfig);
fig = figure(nfig); hold on;
set(fig, 'Visible','on', 'Name', 'NeoCASS - Model plot', 'NumberTitle','off'); 
grid;

% plot NODES
ngrid = beam_model.Info.ngrid;

for n=1:ngrid
	if beam_model.Node.Index(n)
		plot3(beam_model.Node.Coord(n, 1), beam_model.Node.Coord(n, 2), ...
    beam_model.Node.Coord(n, 3), 'kx', 'MarkerSize', 4, 'MarkerFaceColor','k');
	end
end

% plot beams / bars

nbar = beam_model.Info.nbar;

if nbar > 0

	plot_3n_line_elem(nbar, beam_model.Bar, beam_model.PBar, beam_model.Node);
	
end
	
nbeam = beam_model.Info.nbeam;

if nbeam > 0

	plot_3n_line_elem(nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Node);
	
end

if beam_model.Info.amesh_av_vlm
  if (PLOT_SYMM)
	  plot_aero_model(beam_model.Aero.lattice_vlm, beam_model.Aero.ref, ...
      WAKE_PLOT, NORMAL_PLOT,  CHORD_FACTOR * beam_model.Aero.ref.C_mgc, 0);
  else
	  plot_aero_model(beam_model.Aero.lattice_vlm, beam_model.Aero.ref, ...
      WAKE_PLOT, NORMAL_PLOT,  CHORD_FACTOR * beam_model.Aero.ref.C_mgc, ...
      beam_model.Aero.state.SIMXZ);
  end
  plot_body(beam_model.Aero.body);
elseif beam_model.Info.amesh_av_dlm
  if (PLOT_SYMM)
	  plot_aero_model(beam_model.Aero.lattice_dlm, beam_model.Aero.ref, ...
      WAKE_PLOT, NORMAL_PLOT,  CHORD_FACTOR * beam_model.Aero.ref.C_mgc, 0);
  else
	  plot_aero_model(beam_model.Aero.lattice_dlm, beam_model.Aero.ref, ...
      WAKE_PLOT, NORMAL_PLOT,  CHORD_FACTOR * beam_model.Aero.ref.C_mgc, ...
      beam_model.Aero.state.SIMXZ);
  end
else
  plot_body(beam_model.Aero.body);
end

if beam_model.Info.nrbe0

	plot_beam_aero_nodes(beam_model.Info.ngrid, beam_model.Node);

end

if beam_model.Info.nconm

  plot_conm(beam_model.Info.nconm, beam_model.Node.Coord, beam_model.ConM.Node, beam_model.ConM.Offset);

end

if beam_model.Info.nspc

  plot_spc(beam_model.Info.nspc, beam_model.Node.Coord, beam_model.SPC);

end

if beam_model.Info.nrbe2

	plot_beam_RBE2(beam_model.Info.nrbe2, beam_model.Node, beam_model.RBE2);

end

axis equal;
view([37.5 30]);

end

%*******************************************************************************
function plot_3n_line_elem(nbar, Bar, Pbar, Node)

linec = '-ko';
X1 = zeros(2,3);

	for n=1:nbar

		n1 = Bar.Conn(n, 1);
		n2 = Bar.Conn(n, 2);
		n3 = Bar.Conn(n, 3);

		offset1 = Node.R(:,:,n1) * Bar.Offset(n, 1:3)';
		offset2 = Node.R(:,:,n2) * Bar.Offset(n, 4:6)';
		offset3 = Node.R(:,:,n3) * Bar.Offset(n, 7:9)';

		% nodes	

		plot3([Node.Coord(n1,1) + offset1(1); Node.Coord(n2,1) + offset2(1)], ...
		  [Node.Coord(n1,2) + offset1(2); Node.Coord(n2,2) + offset2(2)], ...
		  [Node.Coord(n1,3) + offset1(3); Node.Coord(n2,3) + offset2(3)], ...
      linec,'LineWidth',2, 'MarkerSize', 6, 'MarkerFaceColor','r');

		plot3([Node.Coord(n2,1) + offset2(1); Node.Coord(n3,1) + offset3(1)], ...
		  [Node.Coord(n2,2) + offset2(2); Node.Coord(n3,2) + offset3(2)], ...
		  [Node.Coord(n2,3) + offset2(3); Node.Coord(n3,3) + offset3(3)], ...
      linec,'LineWidth',2, 'MarkerSize', 6, 'MarkerFaceColor','r');

		% offset points	

		if norm(offset1) > 0

			plot3([Node.Coord(n1,1) + offset1(1); Node.Coord(n1,1)], ...
				  [Node.Coord(n1,2) + offset1(2); Node.Coord(n1,2)], ...
				  [Node.Coord(n1,3) + offset1(3); Node.Coord(n1,3)], '--k');

		end	
		if norm(offset2) > 0

			plot3([Node.Coord(n2,1) + offset1(1); Node.Coord(n2,1)], ...
				  [Node.Coord(n2,2) + offset1(2); Node.Coord(n2,2)], ...
				  [Node.Coord(n2,3) + offset1(3); Node.Coord(n2,3)], '--k');

		end	
		if norm(offset3) > 0

			plot3([Node.Coord(n3,1) + offset1(1); Node.Coord(n3,1)], ...
				  [Node.Coord(n3,2) + offset1(2); Node.Coord(n3,2)], ...
				  [Node.Coord(n3,3) + offset1(3); Node.Coord(n3,3)], '--k');

		end	
		% collocation points	
		plot3([Bar.Colloc(:, 1, n)]', [Bar.Colloc(:, 2, n)]', [Bar.Colloc(:, 3, n)]',...
      'ks', 'MarkerSize', 4, 'MarkerFaceColor','r');
	
    % stress points
    for k=1:4
      X1(1,:) = Bar.Colloc(1, :, n) + (Bar.R(:,:,4,n)*[0 Pbar.Str_point(k,:,Bar.PID(n))]')';
      X1(2,:) = Bar.Colloc(1, :, n);
		  plot3(X1(:,1),X1(:,2),X1(:,3),...
      '--cv','LineWidth',1, 'MarkerSize', 2, 'MarkerFaceColor','c');
    end
%
    for k=1:4
      X1(1,:) = Bar.Colloc(2, :, n) + (Bar.R(:,:,5,n)*[0 Pbar.Str_point(k,:,Bar.PID(n))]')';
      X1(2,:) = Bar.Colloc(2, :, n);
		  plot3(X1(:,1),X1(:,2),X1(:,3), ...
      '--cv','LineWidth',1, 'MarkerSize', 2, 'MarkerFaceColor','c');
    end

	end
end
%*******************************************************************************
function plot_aero_model(lattice, ref, WAKE_PLOT, NORMAL_PLOT, C_mac, symm)

  try

	[np, nv, dim] = size(lattice.VORTEX);

	plot3(lattice.XYZ(:,:,1)',lattice.XYZ(:,:,2)',lattice.XYZ(:,:,3)','-mo',...
    'MarkerSize', 1, 'MarkerFaceColor','m');
	plot3(lattice.COLLOC(:,1),lattice.COLLOC(:,2),lattice.COLLOC(:,3),'mx',...
    'MarkerSize', 2, 'MarkerFaceColor','m');

	switch nv
	
		case 6
			c = [3 4];

		case 8
			c = [4 5];

		otherwise
			error('Wrong VORTEX dimension.');
	
	end
	
	x1 = zeros(np,2); 
	x2 = zeros(np,2);
	x3 = zeros(np,2);
	v1 = zeros(np,1);
	v2 = zeros(np,1);
	v3 = zeros(np,1);
	
	x1 = lattice.VORTEX(:,c,1);
	x2 = lattice.VORTEX(:,c,2);
	x3 = lattice.VORTEX(:,c,3);
	
	v1 = mean(x1,2);
	v2 = mean(x2,2);
	v3 = mean(x3,2);

	plot3(v1, v2, v3,'mo',  'MarkerSize', 1, 'MarkerFaceColor','m');

if WAKE_PLOT

	for s = 1:np	

		w=0;
		for u = 1:(nv-1)

			w = w + 1;

			VX(w,:) = [lattice.VORTEX(s, u, 1) lattice.VORTEX(s, u+1, 1)];
			VY(w,:) = [lattice.VORTEX(s, u, 2) lattice.VORTEX(s, u+1, 2)];
			VZ(w,:) = [lattice.VORTEX(s, u, 3) lattice.VORTEX(s, u+1, 3)];

		end   

		plot3(VX, VY, VZ,'m.-.');

	end

end
cref = 1.0;

catch

  cref = ref.C_mgc;
	[np, nv, dim] = size(lattice.DOUBLET);

  if (symm)
    np = np - lattice.npsymm;
  end

	plot3(lattice.XYZ(1:np,:,1)'.*cref,lattice.XYZ(1:np,:,2)'.*cref,...
    lattice.XYZ(1:np,:,3)'.*cref, '-mo',  'MarkerSize', 1, 'MarkerFaceColor','m');
	plot3(lattice.COLLOC(1:np,1).*cref,lattice.COLLOC(1:np,2).*cref,...
    lattice.COLLOC(1:np,3).*cref, 'mx',  'MarkerSize', 2, 'MarkerFaceColor','m');

  if (nv==2)

			c = [1 2];
  else
			error('Wrong DOUBLET dimension.');
	end
	
	x1 = zeros(np,2);
	x2 = zeros(np,2);
	x3 = zeros(np,2);
	v1 = zeros(np,1);
	v2 = zeros(np,1);
	v3 = zeros(np,1);
	
	x1 = lattice.DOUBLET(1:np,c,1).*cref;
	x2 = lattice.DOUBLET(1:np,c,2).*cref;
	x3 = lattice.DOUBLET(1:np,c,3).*cref;
	
	v1 = mean(x1,2);
	v2 = mean(x2,2);
	v3 = mean(x3,2);

	plot3(v1, v2, v3,'mo',  'MarkerSize', 1, 'MarkerFaceColor','m');
end

if NORMAL_PLOT

	for s = 1:np	

    N = lattice.N(s, :) .* (C_mac/norm(lattice.N(s, :)));

	  plot3([lattice.N(s,1) + lattice.COLLOC(s,1).*cref; lattice.COLLOC(s,1).*cref],...
      [lattice.N(s,2) + lattice.COLLOC(s,2).*cref; lattice.COLLOC(s,2).*cref],...
		  [lattice.N(s,3) + lattice.COLLOC(s,3).*cref; lattice.COLLOC(s,3).*cref],...
      'mv--', 'LineWidth',1);

	end
end

end
%*******************************************************************************
function plot_beam_aero_nodes(ngrid, Node)

	lineaer = '--k+';
	nodepos = zeros(3,1);
	
	for n=1:ngrid
	
		if ~isempty(Node.Aero.Coord(n).data)

			for i=1:size(Node.Aero.Coord(n).data,2)

			   nodepos = Node.R(:,:,n) * Node.Aero.Coord(n).data(:,i);
			   
			   plot3([Node.Coord(n,1); Node.Coord(n,1) + nodepos(1)], ...
			     [Node.Coord(n,2); Node.Coord(n,2) + nodepos(2)], ...
			     [Node.Coord(n,3); Node.Coord(n,3) + nodepos(3)], lineaer,...
           'LineWidth',1, 'MarkerSize', 2, 'MarkerFaceColor','k');

			end
		end		
	end
end
%*******************************************************************************
function plot_conm(nconm, Coord, conm_node, offset)

  for n=1:nconm
    
    cx = [Coord(conm_node(n), 1) + offset(n,1), Coord(conm_node(n), 1)];
    cy = [Coord(conm_node(n), 2) + offset(n,2), Coord(conm_node(n), 2)];
    cz = [Coord(conm_node(n), 3) + offset(n,3), Coord(conm_node(n), 3)];

		plot3(cx, cy, cz, '--k', 'MarkerSize', 2, ...
    'MarkerFaceColor','k');
		plot3(cx(1), cy(1), cz(1), '--ko', 'MarkerSize', 2, ...
    'MarkerFaceColor','k');

  end

end
%*******************************************************************************
function plot_spc(nspc, Coord, SPC)

  for n=1:nspc
    index = SPC.Nodes(n).list;
		plot3(Coord(index, 1), Coord(index, 2), Coord(index, 3), 'k^', 'MarkerSize', ...
      4, 'MarkerFaceColor','k');
  end
end

function plot_beam_RBE2( nrbe2, Node, RBE2)
for i = 1 : nrbe2
    ns = length(RBE2.IDS(i).data); 
    mast = find(Node.ID == RBE2.IDM(i));
    for j = 1 : ns
        slv = find(Node.ID == RBE2.IDS(i).data(j));
        plot3([Node.Coord(mast,1) , Node.Coord(slv,1)] , [Node.Coord(mast,2) , Node.Coord(slv,2)] , [Node.Coord(mast,3) , Node.Coord(slv,3)],'r' , 'linewidth',1.5)
    end
    
end
end
