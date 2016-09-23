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
% function animate_flutt_mode(mode_ampl, scale, np, periods, varargin)
%
%   DESCRIPTION: Create animation for structural vibration modes in .avi format 
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                mode_ampl      complex    mode amplitudes from fl_model struct
%                scale          real array scale factor
%                np             integer    number of points in the period
%                periods        integer    number of periods to plot
%                'aero_param'   OPTIONAL   Enable extra aerodynamic plots
%                                          'aero_param', [P_WAKE P_N]
%                                          P_WAKE boolean to enable plot of wake
%                                          P_N boolean to enable plot of normals
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                avi_filename   file       neocass_headname_m[set].avi file
%
%    REFERENCES:
%
%*******************************************************************************

function animate_flutt_mode(mode_ampl, scale, np, periods, varargin)

global beam_model;
global dlm_model;

warning off;
figure = 100;

fid = beam_model.Param.FID;

NMODES = length(mode_ampl)-1;
DISPL = zeros(size(beam_model.Res.NDispl,1),6);

for j=1:NMODES

  DISPL = DISPL + beam_model.Res.NDispl(:,:,j) .* mode_ampl(j);

end
DISPLR = real(DISPL);
DISPLI = imag(DISPL);

%
outname = beam_model.Param.FILE;
dtpos = find('.' == outname);
%
if (isempty(dtpos))
  headname = outname;
else
  headname = outname(1:dtpos(end)-1);
end
%
fileout = strcat(headname, '_fl_mode.avi');
%
WAKE_PLOT = 0;
NORMAL_PLOT = 0;
CONTOUR_PLOT = 0;
CHORD_FACTOR = 0.25;
%
if nargin > 2

	PARAM = varargin;
  for n=1:2:length(PARAM);
    if (strcmp(PARAM(n), 'aero_param')) 
      % array with 
	    AERO_P = PARAM{n+1};
	    WAKE_PLOT = AERO_P(1);
	    NORMAL_PLOT = AERO_P(2);
	    CONTOUR_PLOT = AERO_P(3);
      break;
    end
  end
end

switch(beam_model.Res.SOL)

  case {'Linear flutter', 'Vibration modes'}
    ft = ['NEOCASS - Flutter ', ' - Freq: ', ...
      num2str(real(mode_ampl(end))/2/pi), ' Hz'];

  otherwise
  error('Wrong solution results.');
end

fprintf(fid, '\nAnimating flutter mode from "%s" solver...\n', ...
      beam_model.Res.SOL);

% Open video object
mov = VideoWriter(fileout);
open(mov);

%
twop = 2*pi;
dt = twop / np;

if periods > 1
  scaler = cos([0:dt:twop-dt]) .* scale;
  scalei = sin([0:dt:twop-dt]) .* scale;
  scaleR = repmat(scaler, 1, periods);
  scaleI = repmat(scalei, 1, periods);
else
  scaleR = cos([0:dt:twop]) .* scale;
  scaleI = sin([0:dt:twop]) .* scale;
end

fprintf(fid, 'Total number of frames: %d.',length(scaleR));

for I=1:length(scaleR)

  fprintf(fid, '\n - Frame: %d. Amplitude: %g.', I, scaleR(I));

  H = plot_beam_model(figure);
  % get axis scale;
  V = axis;

  R = zeros(3,3,beam_model.Info.ngrid);

  % update nodal rotation matrix
  for n = 1:beam_model.Info.ngrid

	  R(:,:,n) =  Rmat(scaleR(I).*DISPLR(n,4:6) - scaleI(I).*DISPLI(n,4:6)) * beam_model.Node.R(:,:,n);

  end

  NODEPOS = beam_model.Node.Coord + (scaleR(I).*DISPLR(:,1:3) - scaleI(I).*DISPLI(:,1:3));

  post_plot_beam_model(beam_model.Info, R, NODEPOS, beam_model.Bar, beam_model.Beam);

  if (beam_model.Info.amesh_av_dlm)

      if (strcmp(beam_model.Res.SOL, 'Linear flutter'))
        % plot deformed doublet lattice mesh
        nptot = size(dlm_model.data.c_displ,1);
        XYZ = zeros(nptot,5,3);
        NDISP = XYZ;
        for j=1:NMODES
        count  = 0;
          for k = 1:nptot
            for m = 1:4
              count = count +1;
              NDISP(k, m, 1:3) = NDISP(k, m, 1:3) + mode_ampl(j) .* reshape(dlm_model.data.n_displ(count, 1:3, j),[ 1 1 3]);
            end
            NDISP(k, 5, 1:3) = NDISP(k, 1, 1:3);
          end

        end

        NDISPR = real(NDISP);
        NDISPI = imag(NDISP);
        count = 0;
        for k = 1:nptot
          for m = 1:4
            count = count +1;
            XYZ(k, m, 1:3) = scaleR(I).*NDISPR(k, m, 1:3) - scaleI(I).*NDISPI(k, m, 1:3);
          end
          XYZ(k, 5, 1:3) = XYZ(k, 1, 1:3);
        end
        XYZ = XYZ + dlm_model.aero.cref .* beam_model.Aero.lattice_dlm.XYZ(1:nptot,:,:);
        plot3(XYZ(:,:,1)',XYZ(:,:,2)',XYZ(:,:,3)','-bo',  'MarkerSize', 2, 'MarkerFaceColor','b');
        axis equal;
      end
  end
  if beam_model.Info.nrbe0

	  plot_beam_aero_nodes(beam_model.Info.ngrid, R, NODEPOS, beam_model.Node);

  end

  if beam_model.Info.nconm

    plot_conm(beam_model.Info.nconm, NODEPOS, beam_model.ConM.Node, R, beam_model.ConM.Offset);

  end

  axis(V);
  title(ft);
  view([37.5 30]);

  % Save frame in video object
  writeVideo(mov, getframe(H));

end

fprintf(fid, '\n - Exported animation to %s file.', fileout);
fprintf(fid, '\n\ncompleted.\n\n');

% Close video object
close(mov);

warning on;
close(H);

end
%*******************************************************************************
function post_plot_beam_model(INFO, NODER, COORD, BAR, BEAM)

linec = '-ro';

% plot NODES
ngrid = INFO.ngrid;

for n=1:ngrid

	plot3(COORD(n, 1), COORD(n, 2), COORD(n, 3),'kx', 'MarkerSize', 8, 'MarkerFaceColor','k');
	
end

% plot beams / bars

nbar  = INFO.nbar;
nbeam = INFO.nbeam;

if nbar > 0

	plot_3n_defo_line_elem(nbar, NODER, COORD, BAR);

end

if nbeam > 0

	plot_3n_defo_line_elem(nbeam, NODER, COORD, BEAM);

end

end
%*******************************************************************************
function plot_3n_defo_line_elem(nbar, NODER, COORD, BAR)

linec = '-ro';

for n=1:nbar

	n1 = BAR.Conn(n, 1);
	n2 = BAR.Conn(n, 2);
	n3 = BAR.Conn(n, 3);

	offset1 = NODER(:,:,n1) * BAR.Offset(n, 1:3)';
	offset2 = NODER(:,:,n2) * BAR.Offset(n, 4:6)';
	offset3 = NODER(:,:,n3) * BAR.Offset(n, 7:9)';

	% collocation points	
	N1 = COORD(BAR.Conn(n, 1),1:3) + offset1';
	N2 = COORD(BAR.Conn(n, 2),1:3) + offset2';
	N3 = COORD(BAR.Conn(n, 3),1:3) + offset3';

	COLLOC = interp_colloc_pos(N1, N2, N3);

	% nodes	
	
	plot3([COORD(BAR.Conn(n, 1),1) + offset1(1); COLLOC(1,1)], ...
	  [COORD(BAR.Conn(n, 1),2) + offset1(2); COLLOC(1,2)], ...
	  [COORD(BAR.Conn(n, 1),3) + offset1(3); COLLOC(1,3)], '-r','LineWidth',2,...
    'MarkerSize', 6, 'MarkerFaceColor','r');

	plot3([COORD(BAR.Conn(n, 2),1) + offset2(1); COLLOC(1,1)], ...
    [COORD(BAR.Conn(n, 2),2) + offset2(2); COLLOC(1,2)], ...
    [COORD(BAR.Conn(n, 2),3) + offset2(3); COLLOC(1,3)], '-r','LineWidth',2,...
     'MarkerSize', 6, 'MarkerFaceColor','r');
	
	plot3([COORD(BAR.Conn(n, 2),1) + offset1(1); COLLOC(2,1)], ...
    [COORD(BAR.Conn(n, 2),2) + offset1(2); COLLOC(2,2)], ...
    [COORD(BAR.Conn(n, 2),3) + offset1(3); COLLOC(2,3)], '-r','LineWidth',2,...
     'MarkerSize', 6, 'MarkerFaceColor','r');

	plot3([COORD(BAR.Conn(n, 3),1) + offset2(1); COLLOC(2,1)], ...
    [COORD(BAR.Conn(n, 3),2) + offset2(2); COLLOC(2,2)], ...
    [COORD(BAR.Conn(n, 3),3) + offset2(3); COLLOC(2,3)], '-r','LineWidth',2,...
    'MarkerSize', 6, 'MarkerFaceColor','r');
	
	
	% offset points	

	if norm(offset1) > 0

		plot3([COORD(BAR.Conn(n, 1),1) + offset1(1); COORD(BAR.Conn(n, 1),1)], ...
			  [COORD(BAR.Conn(n, 1),2) + offset1(2); COORD(BAR.Conn(n, 1),2)], ...
			  [COORD(BAR.Conn(n, 1),3) + offset1(3); COORD(BAR.Conn(n, 1),3)], '--k');

	end	
	if norm(offset2) > 0

		plot3([COORD(BAR.Conn(n, 2),1) + offset2(1); COORD(BAR.Conn(n, 2),1)], ...
			  [COORD(BAR.Conn(n, 2),2) + offset2(2); COORD(BAR.Conn(n, 2),2)], ...
			  [COORD(BAR.Conn(n, 2),3) + offset2(3); COORD(BAR.Conn(n, 2),3)], '--k');

	end	
	if norm(offset3) > 0

		plot3([COORD(BAR.Conn(n, 3),1) + offset1(1); COORD(BAR.Conn(n, 3),1)], ...
			  [COORD(BAR.Conn(n, 3),2) + offset1(2); COORD(BAR.Conn(n, 3),2)], ...
			  [COORD(BAR.Conn(n, 3),3) + offset1(3); COORD(BAR.Conn(n, 3),3)], '--k');

	end	

	plot3([COLLOC(:, 1)]', [COLLOC(:, 2)]', [COLLOC(:, 3)]', 'rs', 'MarkerSize', 4,...
     'MarkerFaceColor','r');

	plot3([N1(1) N2(1) N3(1)]', [N1(2) N2(2) N3(2)]', [N1(3) N2(3) N3(3)]', 'ro', ...
    'MarkerSize', 4, 'MarkerFaceColor','r');

end

%axis equal;

end
%*******************************************************************************
function plot_vlm_model(lattice, ref, WAKE_PLOT, NORMAL_PLOT, C_mac)

	plot3(lattice.XYZ(:,:,1)',lattice.XYZ(:,:,2)',lattice.XYZ(:,:,3)','-bo',...
    'MarkerSize', 2, 'MarkerFaceColor','b');
	
	plot3(lattice.COLLOC(:,1),lattice.COLLOC(:,2),lattice.COLLOC(:,3),'bx',...
    'MarkerSize', 4, 'MarkerFaceColor','b');

	[np, nv, dim] = size(lattice.VORTEX);

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

	plot3(v1, v2, v3,'bo',  'MarkerSize', 2, 'MarkerFaceColor','b');

if WAKE_PLOT

	for s = 1:np	

		w=0;
		for u = 1:(nv-1)

			w = w + 1;

			VX(w,:) = [lattice.VORTEX(s, u, 1) lattice.VORTEX(s, u+1, 1)];
			VY(w,:) = [lattice.VORTEX(s, u, 2) lattice.VORTEX(s, u+1, 2)];
			VZ(w,:) = [lattice.VORTEX(s, u, 3) lattice.VORTEX(s, u+1, 3)];

		end   

		plot3(VX, VY, VZ,'b.-.');

	end

end

if NORMAL_PLOT

	for s = 1:np	

    N = lattice.N(s, :) .* (C_mac/norm(lattice.N(s, :)));

	  plot3([N(1) + lattice.COLLOC(s,1); lattice.COLLOC(s,1)],...
    	    [N(2) + lattice.COLLOC(s,2); lattice.COLLOC(s,2)],...
		      [N(3) + lattice.COLLOC(s,3); lattice.COLLOC(s,3)], 'bv--', 'LineWidth', 1);

	end
end

axis equal;

end
%*******************************************************************************
function plot_beam_aero_nodes(ngrid, R, Coord, Node)

	lineaer = '--r+';
	nodepos = zeros(3,1);
	
	for n=1:ngrid
	
		if ~isempty(Node.Aero.Coord(n).data)

			for i=1:size(Node.Aero.Coord(n).data,2)

			   nodepos = R(:,:,n) * Node.Aero.Coord(n).data(:,i);
			   plot3([Coord(n,1); Coord(n,1) + nodepos(1)], ...
				   [Coord(n,2); Coord(n,2) + nodepos(2)], ...
				   [Coord(n,3); Coord(n,3) + nodepos(3)], lineaer,'LineWidth',1, ...
           'MarkerSize', 5, 'MarkerFaceColor','r');

			end
		end		
	end

axis equal;

end
%*******************************************************************************
function plot_conm(nconm, Coord, conm_node, R, offset)

  for n=1:nconm
    offsetr = (R(:,:,conm_node(n)) * offset(n,:)')';    
    cx = [Coord(conm_node(n), 1) + offsetr(1), Coord(conm_node(n), 1)];
    cy = [Coord(conm_node(n), 2) + offsetr(2), Coord(conm_node(n), 2)];
    cz = [Coord(conm_node(n), 3) + offsetr(3), Coord(conm_node(n), 3)];

		plot3(cx, cy, cz, '--r', 'MarkerSize', 10, ...
    'MarkerFaceColor','r');
		plot3(cx(1), cy(1), cz(1), '--ro', 'MarkerSize', 10, ...
    'MarkerFaceColor','r');

  end

end
