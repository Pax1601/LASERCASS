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
% function plot_beam_defo(nfig, scale, varargin)
%
%   DESCRIPTION: Plot beam model result struct (structural and aerodynamic model)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                nfig           integer    figure index to be opened
%                scale          real       amplitude scale
%                'aero_param'   OPTIONAL   Enable extra aerodynamic plots
%                                          'aero_param', [P_WAKE P_N]
%                                          P_WAKE boolean to enable plot of wake
%                                          P_N boolean to enable plot of normals
%                'title'        OPTIONAL   Specify figure title ftitle
%                                          'title', ftitle
%                'set'          OPTIONAL   Plot only result set N (index)
%                                          'set', N
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                               figure     opened figure nfig
%
%    REFERENCES:
%
%*******************************************************************************

function plot_beam_defo(nfig, scale, varargin)

global beam_model;
global dlm_model;

% scale = -scale;
WAKE_PLOT = 0;
NORMAL_PLOT = 0;
CHORD_FACTOR = 0.25;
CONTOUR_PLOT = 0;
UNDEFO_PLOT = true;
fig_title = '';
set = [];
lattice_defo = [];

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
    for n=1:2:length(PARAM);
        if (strcmp(PARAM(n), 'title'))
            fig_title = PARAM{n+1};
            break;
        end
    end
    for n=1:2:length(PARAM);
        if (strcmp(PARAM(n), 'set'))
            set = PARAM{n+1};
            break;
        end
    end
    for n=1:2:length(PARAM);
        if (strcmp(PARAM(n), 'undefo'))
            UNDEFO_PLOT = PARAM{n+1};
            break;
        end
    end
    
end

fid = beam_model.Param.FID;

fprintf(fid, '\nPlotting analysis results from "%s" solver...\n\n', ...
    beam_model.Res.SOL);

NSHAPES = size(beam_model.Res.NDispl,3);
if isempty(set)
    SHAPE_INDEX = [1:NSHAPES];
else
    if (set > NSHAPES)
        error('Required output set %d not available.', set);
    end
    SHAPE_INDEX = set;
    NSHAPES = length(set);
end

amesh_av_dlm_bk = beam_model.Info.amesh_av_dlm;

for nsh = 1:NSHAPES
    
    fprintf(fid, ' - Output set %d.\n', SHAPE_INDEX(nsh));
    figure(nfig); close; figure(nfig);
    ft = '';
    %
    % overwrite figure title
    
    switch(beam_model.Res.SOL)
        
        case {'Linear flutter'}
            ft = ['Vibration mode ', num2str(SHAPE_INDEX(nsh)), ' - Freq: ', ...
                num2str(beam_model.Res.Omega(SHAPE_INDEX(nsh))/2/pi), ' Hz'];
            
        case {'Vibration modes'}
            ft = ['Vibration mode ', num2str(SHAPE_INDEX(nsh)), ' - Freq: ', ...
                num2str(beam_model.Res.Omega(SHAPE_INDEX(nsh))/2/pi), ' Hz'];
            beam_model.Info.amesh_av_dlm = 0;
        case {'Static linear aeroelastic'}
            ft = ['Static linear aeroelastic analysis ', num2str(SHAPE_INDEX(nsh))];
            
        case {'Static non-linear aeroelastic'}
            ft = ['Static non linear aeroelastic analysis ', num2str(SHAPE_INDEX(nsh))];
            
    end
    
    if (UNDEFO_PLOT)
        plot_beam_model(nfig);
    end
    
    if (isempty(ft))
        ft = ['Deformed model plot ', num2str(SHAPE_INDEX(nsh))];
    end
    
    title(ft);
    %
    R = zeros(3,3,beam_model.Info.ngrid);
    
    % update nodal rotation matrix
    for n = 1:beam_model.Info.ngrid
        
        R(:,:,n) =  Rmat(scale.*beam_model.Res.NDispl(n,4:6, SHAPE_INDEX(nsh))) * beam_model.Node.R(:,:,n);
        
    end
    
    
    NODEPOS = beam_model.Node.Coord + scale .* beam_model.Res.NDispl(:,1:3, SHAPE_INDEX(nsh));
    
    BARR =  update_bar_rot(beam_model.Info.nbar, R, ...
        beam_model.Bar.Conn, beam_model.Bar.R, scale.*beam_model.Res.NDispl(:,:,SHAPE_INDEX(nsh)));
    
    BEAMR =  update_bar_rot(beam_model.Info.nbeam, R, ...
        beam_model.Beam.Conn, beam_model.Beam.R, scale.*beam_model.Res.NDispl(:,:,SHAPE_INDEX(nsh)));
    
    post_plot_beam_model(nfig, beam_model.Info, R, NODEPOS, beam_model.Bar, beam_model.PBar,...
        beam_model.Beam, beam_model.PBeam, BARR, BEAMR);
    
    if (beam_model.Info.amesh_av_dlm || beam_model.Info.amesh_av_vlm)
        
        if (strcmp(beam_model.Res.SOL, 'Linear flutter') || strcmp(beam_model.Res.SOL, 'Linear Dynamic Derivatives'))
            % plot deformed doublet lattice mesh
            nptot = beam_model.Aero.lattice_dlm.np;
            XYZ = zeros(nptot,5,3);
            count  = 0;
            for k = 1:nptot
                for m = 1:4
                    count = count +1;
                    XYZ(k, m, 1:3) = scale .* dlm_model.data.n_displ(count, 1:3, SHAPE_INDEX(nsh));
                end
                XYZ(k, 5, 1:3) = XYZ(k, 1, 1:3);
            end
            COLLOC = zeros(nptot,3);
            COLLOC = dlm_model.aero.cref .* beam_model.Aero.lattice_dlm.MID_DPOINT(1:nptot,:,:) + ...
                scale .* dlm_model.data.c_displ(1:nptot,:,SHAPE_INDEX(nsh));
            XYZ = XYZ + dlm_model.aero.cref .* beam_model.Aero.lattice_dlm.XYZ(1:nptot,:,:);
            plot3(XYZ(:,:,1)',XYZ(:,:,2)',XYZ(:,:,3)','-bo',  'MarkerSize', 1, 'MarkerFaceColor','b');
            plot3(COLLOC(:,1),COLLOC(:,2),COLLOC(:,3),'bx',  'MarkerSize', 2, 'MarkerFaceColor','b');
            axis equal;
        else
            if isempty(beam_model.Aero.lattice_defo)
                ntrim = size(beam_model.Res.CS.Value,1);
                beam_model.Aero.lattice = beam_model.Aero.lattice_vlm;
                beam_model.Aero.lattice_vlm = rotate_control_surf(beam_model.Aero.ref, beam_model.Aero.state, beam_model.Aero.geo, ...
                    beam_model.Aero.lattice_vlm, beam_model.Res.CS.Value(ntrim,:), ...
                    beam_model.Aero.lattice_vlm.Control.Hinge,beam_model.Aero.ID);
            
            
                if (beam_model.Info.spline_type==1)
                    lattice_defo = update_vlm_mesh1(beam_model.Node, scale.*ndispl2dof(beam_model.Node.DOF, beam_model.Res.NDispl), beam_model.Aero);
                else
                    Nrd = zeros(size(beam_model.Res.NRd,1),size(beam_model.Res.NRd,2));
                    for n = 1:beam_model.Info.ngrid
                        NRd(:,:,n) = Rmat(scale.*beam_model.Res.NDispl(n,4:6));
                    end
                    lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord + scale.*beam_model.Res.NDispl(:,1:3), ...
                        NRd, beam_model.Aero);
                end
            else
                lattice_defo = beam_model.Aero.lattice_defo;
            end
            
            beam_model.Aero.lattice_vlm = beam_model.Aero.lattice;
            
            plot_vlm_model(lattice_defo, beam_model.Aero.ref, WAKE_PLOT, NORMAL_PLOT, ...
                CHORD_FACTOR * beam_model.Aero.ref.C_mgc);
            
            if (CONTOUR_PLOT)
                try
                    beam_model.Res.Aero.FN;
                    colormap(hot);
                    fill3(lattice_defo.XYZ(:,:,1)', lattice_defo.XYZ(:,:,2)', lattice_defo.XYZ(:,:,3)', ...
                        beam_model.Res.Aero.FN');
                    colorbar('vert');
                end
                %      end
            end
        end
        
    end
    
    if beam_model.Info.nconm
        
        plot_conm(beam_model.Info.nconm, NODEPOS, beam_model.ConM.Node, R, beam_model.ConM.Offset);
        
    end
    
    if beam_model.Info.nrbe2
        
        plot_beam_RBE2(beam_model.Info.nrbe2, beam_model.Node, NODEPOS, beam_model.RBE2);
        
    end
    
    nfig = nfig +1;
    
end

beam_model.Info.amesh_av_dlm = amesh_av_dlm_bk;

fprintf(fid, '\ncompleted.\n\n');

end
%*******************************************************************************
function post_plot_beam_model(nfig, INFO, NODER, COORD, BAR, PBAR, BEAM, PBEAM, BARR, BEAMR)

fig = figure(nfig); hold on;
linec = '-ro';

% plot NODES
ngrid = INFO.ngrid;

for n=1:ngrid
    
    plot3(COORD(n, 1), COORD(n, 2), COORD(n, 3),'kx', 'MarkerSize', 4, 'MarkerFaceColor','k');
    
end

% plot beams / bars

nbar  = INFO.nbar;
nbeam = INFO.nbeam;

if nbar > 0
    
    plot_3n_defo_line_elem(nbar, NODER, COORD, BAR, BARR, PBAR);
    
end

if nbeam > 0
    
    plot_3n_defo_line_elem(nbeam, NODER, COORD, BEAM, BEAMR, PBEAM);
    
end

end
%*******************************************************************************
function plot_3n_defo_line_elem(nbar, NODER, COORD, BAR, BARR, PBAR)

linec = '-ro';
X1 = zeros(2,3);

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
        [COORD(BAR.Conn(n, 1),3) + offset1(3); COLLOC(1,3)], ...
        '-r','LineWidth',1, 'MarkerSize', 3, 'MarkerFaceColor','r');
    
    plot3([COORD(BAR.Conn(n, 2),1) + offset2(1); COLLOC(1,1)], ...
        [COORD(BAR.Conn(n, 2),2) + offset2(2); COLLOC(1,2)], ...
        [COORD(BAR.Conn(n, 2),3) + offset2(3); COLLOC(1,3)], ...
        '-r','LineWidth',1, 'MarkerSize', 3, 'MarkerFaceColor','r');
    
    plot3([COORD(BAR.Conn(n, 2),1) + offset1(1); COLLOC(2,1)], ...
        [COORD(BAR.Conn(n, 2),2) + offset1(2); COLLOC(2,2)], ...
        [COORD(BAR.Conn(n, 2),3) + offset1(3); COLLOC(2,3)],...
        '-r','LineWidth',1, 'MarkerSize', 3, 'MarkerFaceColor','r');
    
    plot3([COORD(BAR.Conn(n, 3),1) + offset2(1); COLLOC(2,1)], ...
        [COORD(BAR.Conn(n, 3),2) + offset2(2); COLLOC(2,2)], ...
        [COORD(BAR.Conn(n, 3),3) + offset2(3); COLLOC(2,3)],...
        '-r','LineWidth',1, 'MarkerSize', 3, 'MarkerFaceColor','r');
    
    
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
    
    plot3([COLLOC(:, 1)]', [COLLOC(:, 2)]', [COLLOC(:, 3)]', 'rs', 'MarkerSize', ...
        2, 'MarkerFaceColor','r');
    
    plot3([N1(1) N2(1) N3(1)]', [N1(2) N2(2) N3(2)]', [N1(3) N2(3) N3(3)]', 'ro', ...
        'MarkerSize', 2, 'MarkerFaceColor','r');
    
    % stress points
    for k=1:4
        X1(1,:) = COLLOC(1, :) + (BARR(:,:,4,n)*[0 PBAR.Str_point(k,:,BAR.PID(n))]')';
        X1(2,:) = COLLOC(1, :);
        plot3(X1(:,1),X1(:,2),X1(:,3),...
            '--yv','LineWidth',1, 'MarkerSize', 2, 'MarkerFaceColor','y');
    end
    %
    for k=1:4
        X1(1,:) = COLLOC(2, :) + (BARR(:,:,5,n)*[0 PBAR.Str_point(k,:,BAR.PID(n))]')';
        X1(2,:) = COLLOC(2, :);
        plot3(X1(:,1),X1(:,2),X1(:,3),...
            '--yv','LineWidth',1, 'MarkerSize', 2, 'MarkerFaceColor','y');
    end
    
    
end

axis equal;
view([37.5 30]);

end
%*******************************************************************************
function plot_vlm_model(lattice, ref, WAKE_PLOT, NORMAL_PLOT, C_mac)

plot3(lattice.XYZ(:,:,1)',lattice.XYZ(:,:,2)',lattice.XYZ(:,:,3)','-bo',...
    'MarkerSize', 1, 'MarkerFaceColor','b');

plot3(lattice.COLLOC(:,1),lattice.COLLOC(:,2),lattice.COLLOC(:,3),'bx',...
    'MarkerSize', 2, 'MarkerFaceColor','b');

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

plot3(v1, v2, v3,'bo',  'MarkerSize', 6, 'MarkerFaceColor','b');

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
function plot_conm(nconm, Coord, conm_node, R, offset)

for n=1:nconm
    offsetr = (R(:,:,conm_node(n)) * offset(n,:)')';
    cx = [Coord(conm_node(n), 1) + offsetr(1), Coord(conm_node(n), 1)];
    cy = [Coord(conm_node(n), 2) + offsetr(2), Coord(conm_node(n), 2)];
    cz = [Coord(conm_node(n), 3) + offsetr(3), Coord(conm_node(n), 3)];
    
    plot3(cx, cy, cz, '--r', 'MarkerSize', 4, ...
        'MarkerFaceColor','r');
    plot3(cx(1), cy(1), cz(1), '--ro', 'MarkerSize', 4, ...
        'MarkerFaceColor','r');
    
end

end
%*******************************************************************************
function UPDR = update_bar_rot(nbar, DELTAR, BAR_CONN, BARR, SOL)

UPDR = zeros(3, 3, 5, nbar);

% COLLOC 1
eta = -1/sqrt(3);
NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
NID1 = NI(1) .* eye(3); NID2 = NI(2) .* eye(3); NID3 = NI(3) .* eye(3);

% COLLOC 2
eta = +1/sqrt(3);
NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
NIID1 = NII(1) .* eye(3); NIID2 = NII(2) .* eye(3); NIID3 = NII(3) .* eye(3);

for n=1:nbar
    
    n1 = BAR_CONN(n, 1);
    n2 = BAR_CONN(n, 2);
    n3 = BAR_CONN(n, 3);
    
    % update nodal sol
    
    UPDR(:,:, 1, n) = DELTAR(:,:,n1) *  BARR(:,:, 1, n);
    UPDR(:,:, 2, n) = DELTAR(:,:,n2) *  BARR(:,:, 2, n);
    UPDR(:,:, 3, n) = DELTAR(:,:,n3) *  BARR(:,:, 3, n);
    
    % update colloc sol
    % interpolate Gibbs-Rodriguez parameters
    gI  = NID1  * SOL(n1, 4:6)' + NID2  * SOL(n2, 4:6)' + NID3  * SOL(n3, 4:6)';
    gII = NIID1 * SOL(n1, 4:6)' + NIID2 * SOL(n2, 4:6)' + NIID3 * SOL(n3, 4:6)';
    
    UPDR(:,:, 4, n) = Rmat(gI)  *  BARR(:,:, 4, n);
    UPDR(:,:, 5, n) = Rmat(gII) *  BARR(:,:, 5, n);
    
end

end
%*******************************************************************************
function plot_beam_RBE2( nrbe2, Node, Coord, RBE2)
for i = 1 : nrbe2
    ns = length(RBE2.IDS(i).data);
    mast = find(Node.ID == RBE2.IDM(i));
    for j = 1 : ns
        slv = find(Node.ID == RBE2.IDS(i).data(j));
        plot3([Coord(mast,1) , Coord(slv,1)] , [Coord(mast,2) , Coord(slv,2)] , ...
            [Coord(mast,3) , Coord(slv,3)],'r' , 'linewidth',1.5)
    end
    
end
end
