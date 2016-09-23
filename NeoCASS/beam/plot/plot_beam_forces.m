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

function plot_beam_forces(nfig, scale, set)

global beam_model;

CHORD_FACTOR = 0.25;
PLOT_SYMM = 0;

fig = figure(nfig);
close(nfig);
fig = figure(nfig); hold on;
%set(fig, 'Visible','on', 'Name', 'NeoCASS - Model plot', 'NumberTitle','off'); 
grid;

% plot NODES
ngrid = beam_model.Info.ngrid;

for n=1:ngrid
	if beam_model.Node.Index(n)
		plot3(beam_model.Node.Coord(n, 1), beam_model.Node.Coord(n, 2), ...
    beam_model.Node.Coord(n, 3), 'kx', 'MarkerSize', 8, 'MarkerFaceColor','k');
	end
end

% plot beams / bars

nbar = beam_model.Info.nbar;

if nbar > 0

	plot_3n_line_elem(nbar, beam_model.Bar, beam_model.PBar, beam_model.Node);
%
  maxv = 0.0;
  for n=1:beam_model.Info.nbar
    for k=1:2
      if (abs(beam_model.Res.Bar.CForces(k,set,n))>maxv)
        maxv = abs(beam_model.Res.Bar.CForces(k,set,n));
      end
    end
  end
  for n=1:beam_model.Info.nbar

    % determine main axis orientation
    [v, index] = max(abs(beam_model.Bar.R(:,2,1,n)));
    F = zeros(1,3);
    for k=1:2
      F(index) = scale*beam_model.Res.Bar.CForces(k,set,n)/maxv; 

    cx = [beam_model.Bar.Colloc(k,1,n), beam_model.Bar.Colloc(k,1,n) + F(1)];
    cy = [beam_model.Bar.Colloc(k,2,n), beam_model.Bar.Colloc(k,2,n) + F(2)];
    cz = [beam_model.Bar.Colloc(k,3,n), beam_model.Bar.Colloc(k,3,n) + F(3)];

		plot3(cx, cy, cz, '--r', 'MarkerSize', 6, ...
    'MarkerFaceColor','r');
		plot3(cx(2), cy(2), cz(2), '--ro', 'MarkerSize', 6, ...
    'MarkerFaceColor','r');

    end
  end

end

nbeam = beam_model.Info.nbeam;

if nbeam > 0

	plot_3n_line_elem(nbeam, beam_model.Beam, beam_model.PBeam, beam_model.Node);
	
end

if beam_model.Info.nspc

  plot_spc(beam_model.Info.nspc, beam_model.Node.Coord, beam_model.SPC);

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
	
	end
end
%*******************************************************************************
function plot_conm(nconm, Coord, conm_node, offset)

  for n=1:nconm
    
    cx = [Coord(conm_node(n), 1) + offset(n,1), Coord(conm_node(n), 1)];
    cy = [Coord(conm_node(n), 2) + offset(n,2), Coord(conm_node(n), 2)];
    cz = [Coord(conm_node(n), 3) + offset(n,3), Coord(conm_node(n), 3)];

		plot3(cx, cy, cz, '--k', 'MarkerSize', 10, ...
    'MarkerFaceColor','k');
		plot3(cx(1), cy(1), cz(1), '--ko', 'MarkerSize', 10, ...
    'MarkerFaceColor','k');

  end

end
%*******************************************************************************
function plot_spc(nspc, Coord, SPC)

  for n=1:nspc
    index = SPC.Nodes(n).list;
		plot3(Coord(index, 1), Coord(index, 2), Coord(index, 3), 'k^', 'MarkerSize', ...
      10, 'MarkerFaceColor','k');
  end
end
