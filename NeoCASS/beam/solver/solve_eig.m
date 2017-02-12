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
function solve_eig

global beam_model;
%
SUP_SHAPE = 0;
fid = beam_model.Param.FID;
EPS = 1.0e-4;

if (~isempty(find(beam_model.Param.MSOL == 103)))
    
    fprintf(fid,'\nSolving vibration modes analysis...');
    %
    %   assembly stifness matrix
    %
    fprintf(fid, '\n\t - Assemblying stiffness matrix...');
    K = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam, beam_model.Celas);
    fprintf(fid, 'done.');
    %
    %   assembly mass matrix
    %
    fprintf(fid, '\n\t - Assemblying mass matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
    fprintf(fid, 'done.');
    if ~isempty(beam_model.RBE2)
        M2 = M;
        K2 = K;
        ndof = size(K,1);
        K = RBE2Assembly(beam_model.RBE2,K);
        M = RBE2Assembly(beam_model.RBE2,M);
    end
    %
    %   extract eigenvalues
    %
    opts.disp = 0;
    %
    fprintf(fid, '\n\t - Extracting eigenvalues...');
    if (beam_model.Param.MAXF == 0)
        beam_model.Param.MAXF = realmax;
    end
    fprintf(fid,'\n\t\tFrequency range: from %g to %g Hz (MINF,MAXF).', beam_model.Param.MINF, beam_model.Param.MAXF);
    fprintf(fid,'\n\t\tNumber of modes to extract (NROOTS): %d.', beam_model.Param.NROOTS);
    %   find all values
    if (beam_model.Param.NROOTS == 0)
        beam_model.Param.NROOTS = int32(beam_model.Info.ndof);
    end
    %
    if (beam_model.Param.NROOTS >= beam_model.Info.ndof)
        %     find  all modes
        beam_model.Param.NROOTS = int32(beam_model.Info.ndof);
        fprintf(fid,'\n\tNROOTS limited to: %d.', beam_model.Param.NROOTS);
        warning off;
        [V, D] = eig(full(K),full(M));
        warning on;
    else
        warning off;
        %     force matrix to be simmetrix due to tolerances for eigs solver on Windows system
        M = (M + M') ./ 2.0;
        [V, D] = eigs(K, M, beam_model.Param.NROOTS, 'SM', opts);
        %     [V, D] = eig(full(K),full(M));
        eigenvalue = diag(D);
        [eigenvalue, indexROTT] = sort(eigenvalue);
        V = V(:,indexROTT);
        D = diag(eigenvalue);
        
        V = V(:,1:beam_model.Param.NROOTS);
        D = D(1:beam_model.Param.NROOTS,1:beam_model.Param.NROOTS);
        warning on;
    end
    %
    if ~isempty(beam_model.RBE2)
        NRoots = size(V,2);
        V2 = zeros(ndof,NRoots);
        for jeig = 1 : NRoots
            V2(:,jeig) = RBE2disp(beam_model.RBE2,V(:,jeig),ndof);
        end
    end
    puls = sqrt(abs(diag(D)));
    [Omega, index] = sort(puls);
    Freq = Omega./(2*pi);
    %
    V = real(V(:, index));
    if ~isempty(beam_model.RBE2)
        V2 = real(V2(:, index));
    end
    
    NAVM = length(index);
    OID = [1:NAVM];
    fprintf(fid, '\n\n\t\tID     Frequency [Hz]');
    for m=1:NAVM
        fprintf(fid, '\n\t\t%d      %g', OID(m), Freq(m));
    end
    %-------------------------------------------------------------------------------
    %   Consider frequency range
    %
    il = find(Freq >= beam_model.Param.MINF);
    if (isempty(il))
        error('No eigenvalue higher than %g found.', beam_model.Param.MINF);
    end
    ih = find(Freq <= beam_model.Param.MAXF);
    if (isempty(ih))
        error('No eigenvalue lower than %g found.', beam_model.Param.MAXF);
    end
    index = intersect(il, ih);
    if (isempty(index))
        error('Unable to extract vibration modes for the required frequency range.');
    end
    %
    %   Consider user-defined modes
    %
    if (~isempty(beam_model.Param.MSELECT))
        [v, ia, ib] = intersect(OID, beam_model.Param.MSELECT);
        index = intersect(index, ia);
    end
    %
    OID = OID(index); NMODES = length(index);
    if (~isempty(beam_model.Param.MSELECT))
        beam_model.Param.MSELECT = OID; % cleaned with f range
    end
    %
    fprintf(fid, '\n\n\t\tNumber of modes retained: %d.\n', NMODES);
    % set eigenvectors
    Omega = Omega(index);
    V = V(:,index);
    if ~isempty(beam_model.RBE2)
        V2 = V2(:,index);
    end
    fprintf(fid, '\n\t\tID     Frequency [Hz]');
    for m=1:NMODES
        fprintf(fid, '\n\t\t%d      %g', OID(m), Omega(m)/2/pi);
    end
    % set eigenvalues
    mscale = zeros(NMODES, 1);
    mm = zeros(NMODES);
    km = zeros(NMODES);
    % set eigenvalues
    meth = 0;
    if (strcmp(beam_model.Param.MSCALE,'MAX'))
        meth = 1;
    end
    %
    switch(beam_model.Param.MSCALE)
        
        case 'MASS'
            fprintf(fid, '\n\n\t\tNormalization: unity generalized mass.');
            for m=1:NMODES
                mscale(m) = V(:,m)' * M * V(:,m);
                if ~isempty(beam_model.RBE2)
                    mscale2 = V2(:,m)' * M2 * V2(:,m);
                end
                V(:,m) = V(:,m) / sqrt(mscale(m));
                if ~isempty(beam_model.RBE2)
                    V2(:,m) = V2(:,m) / sqrt(mscale2);
                end
            end
            if ~isempty(beam_model.RBE2)
                MODES = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, NMODES, V2);
            else
                MODES = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, NMODES, V);
            end
            mm = V' * M * V;
            km = V' * K * V;
            
        case {'MAX', 'POINT'}
            
            if ~isempty(beam_model.RBE2)
                MODES = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, NMODES, V2);
            else
                MODES = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, NMODES, V);
            end
            if (meth == 1)
                fprintf(fid, '\n\n\t\tNormalization: unit value of the largest component in the analysis set.');
                % max
                for m=1:NMODES
                    mscale(m) = max(max(abs(MODES(:,:,m))));
                    MODES(:,:,m) = MODES(:,:,m) ./ mscale(m);
                end
            else
                fprintf(fid, '\n\n\t\tNormalization: unit value of the %d component for node %d.', ...
                    beam_model.Param.MC, beam_model.Param.MG);
                fdof = setdiff([1:6], beam_model.Param.MC);
                % point
                for m = 1:NMODES
                    mscale(m) = MODES(beam_model.Param.MG, beam_model.Param.MC, m);
                    if (mscale(m) == 0)
                        fprintf(fid, '\n\t\t### Warning: zero displacement found. ');
                        for nfdof = 1:5
                            mscale(m) = MODES(beam_model.Param.MG, fdof(nfdof), m);
                            if (mscale(m) ~= 0)
                                break;
                            end
                        end
                        fprintf(fid, '\n\t\tUsed dof %d to normalize modeshape %d. !!', fdof(nfdof), m);
                    end
                    MODES(:,:,m) = MODES(:,:,m) ./ mscale(m);
                end
            end % end if
            
            mm = V' * M * V;
            km = V' * K * V;
            for m=1:NMODES
                V(:,m) = V(:,m)/mscale(m);
                mm(m,m) = mm(m,m) / (mscale(m)^2);
                km(m,m) = km(m,m) / (mscale(m)^2);
            end
    end % end case
    fprintf(fid, '\n\n\t completed.');
    
    %
    %   set output results
    %
    %
    fprintf(fid,'\n\t - Storing analysis results..');
    % update interal database
    beam_model.Res.SOL = 'Vibration modes';
    % store MODAL nodal displacement
    beam_model.Res.NDispl = MODES(:,:,:);
    
    
    
    beam_model.Res.NRd = zeros(3, 3, beam_model.Info.ngrid, NMODES);
    crossR = zeros(3, 3, beam_model.Info.ngrid, NMODES);
    % set MODAL delta Rot
    for n = 1:beam_model.Info.ngrid
        for m = 1:NMODES
            beam_model.Res.NRd(:,:,n,m) = Rmat(MODES(n, 4:6, m));
            crossR(:,:,n,m) = crossm(MODES(n, 4:6, m));
        end
    end
    %   update elements
    for m = 1:NMODES
        beam_model.Res.Bar.R(:,:,:,:,m) = update_bar_rot(beam_model.Info.nbar, beam_model.Res.NRd(:,:,:,m), ...
            beam_model.Bar.Conn, beam_model.Bar.R, beam_model.Res.NDispl(:,:,m));
        beam_model.Res.Beam.R(:,:,:,:,m) = update_bar_rot(beam_model.Info.nbeam, beam_model.Res.NRd(:,:,:,m), ...
            beam_model.Beam.Conn, beam_model.Beam.R, beam_model.Res.NDispl(:,:,m));
        
    end
    
    beam_model.Res.Mmm = diag(diag(mm));
    beam_model.Res.Kmm = diag(diag(km));
    beam_model.Res.M = M;
    beam_model.Res.K = K;
    beam_model.Res.Omega = Omega;
    beam_model.Res.V = V;
    
    fprintf(fid, 'done.');
    
    beam_model.Res.ID = OID;
    if (~isempty(beam_model.Param.SUPORT))
        %
        %   Overwrite rigid body modes using SUPORT data
        %
        
        MINDEX = find(OID<=6);
        %
        if (isempty(MINDEX) )
            fprintf(fid, '\n\t - No rigid mode available.');
        end
        %
        if (~isempty(MINDEX))
            %
            if (~isempty(beam_model.Param.SUPORT)) && SUP_SHAPE
                %       get rigid modes at suport point through K matrix
                [RMODE, RINDEX, KEPS] = get_suport_shapes(K, beam_model.Node, beam_model.Param.SUPORT, beam_model.Param.SUP_MAMPL);
            else
                %       get rigid modes at CG as simple kinematics for DOF and DOF2
                [RMODE, RMODE2, RINDEX, KEPS] = get_CG_shapes(beam_model.Node, beam_model.RBE2, beam_model.WB.CG, beam_model.Param.SUPORT);
                nrsup = length(RINDEX);
                MODES(:,:,1:nrsup) = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, nrsup, RMODE2);
            end
            %
            if (~isempty(beam_model.Param.MSELECT))
                [dummy, id, jd] = intersect(beam_model.Param.MSELECT, RINDEX);
                nrsup = length(jd);
            else
                nrsup = size(RMODE,2);
                jd = [1:nrsup];
            end
            fprintf(fid, '\nSUPORT required: %d rigid body modes overwritten.', nrsup);
            %
            nr = find(KEPS > EPS);
            if (~isempty(nr))
                fprintf(fid, '\n### Warning: %d SUPORT rigid modes exceed deformation energy tolerance %g.', length(nr), EPS);
            end
            %
            nr = find((beam_model.Res.Omega/2/pi) < EPS);
            if (length(nr) < nrsup)
                fprintf(fid, '\n### Warning: number of extracted rigid modes is less than SUPORT dofs.');
            end
            %
            %      MASS matrix
            %
            if SUP_SHAPE
                beam_model.Res.Mmm(1:nrsup,1:nrsup) = RMODE' * M * RMODE;
                if ~isempty(beam_model.RBE2)
                    RMODE2 = zeros(ndof,nrsup);
                    for jeig = 1:nrsup
                        RMODE2(:,jeig) = RBE2disp(beam_model.RBE2,RMODE(:,jd(jeig)),ndof);
                    end
                    MODES(:,:,1:nrsup) = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, nrsup, RMODE2);
                else
                    MODES(:,:,1:nrsup) = get_mode_shapes(beam_model.Info.ngrid, beam_model.Node.DOF, nrsup, RMODE);
                end
            else
                
                beam_model.Res.Mmm(1:nrsup,1:nrsup) = beam_model.WB.MCG(RINDEX,RINDEX);
                beam_model.Res.Mmm(1:nrsup,1:nrsup) = RMODE' * M * RMODE;
                
            end
            %
            beam_model.Res.V(:,1:nrsup) = RMODE(:,jd);
            %
            % store MODAL nodal displacement
            beam_model.Res.NDispl = MODES(:,:,:);
            % set MODAL delta Rot
            for n = 1:beam_model.Info.ngrid
                for m = 1:nrsup
                    beam_model.Res.NRd(:,:,n,m) = Rmat(MODES(n, 4:6, m));
                    crossR(:,:,n,m) = crossm(MODES(n, 4:6, m));
                end
            end
            %
            beam_model.Res.Kmm(1:nrsup,1:nrsup) = 0.0;
            beam_model.Res.Omega(1:nrsup) = 0.0;
            fprintf(fid, 'done.');
            %
        end
        
    end
    %
    %   update aerobeam nodes (if any)
    %
    if (beam_model.Info.nrbe0 > 0)
        ngrid = beam_model.Info.ngrid;
        fprintf(fid,'\n\t - Updating modal shapes for slave nodes...');
        for m = 1:NMODES
            AERO_POS = update_aerobeam_node(ngrid, beam_model.Node, beam_model.Res.NDispl(:,1:3,m), crossR(:,:,:,m));
            % update coord database with slave nodes position
            for n=1:ngrid
                ne = length(beam_model.Node.Aero.Index(n).data);
                if ne
                    beam_model.Res.NDispl(beam_model.Node.Aero.Index(n).data, 1:3, m) = AERO_POS(n).data';
                end
            end
            clear AERO_POS;
        end
        fprintf(fid, 'done.');
    end
    %
    %   Build structural damping matrix
    %
    SDAMP = beam_model.Param.SDAMP;
    if (beam_model.Param.SDAMP)
        fprintf(fid,'\n - Building complex stiffness matrix for damping...');
        DAMP = beam_model.Damp;
        beam_model.Res.Gmm = modal_damp(DAMP.g{SDAMP}, DAMP.Freq{SDAMP}, DAMP.Type(SDAMP), 0, ...
            beam_model.Res.Mmm, beam_model.Res.Omega./(2*pi));
        fprintf(fid, 'done.');
    end
    %
    fprintf(fid, '\n\ndone.\n\n');
    %
    if (beam_model.Param.AUTOPLOT)
        
        plot_beam_defo(100, 1, 'title', 'NEOCASS - vibration mode ');
        
    end
    
else
    
    error('SOL 103 must be given in input file to run eigenmodes solver.');
    
end
end
%***********************************************************************************************************************
function [RMODE, RINDEX, EPS] = get_suport_shapes(K, NODE, SUPORT, MODE_AMPL)
%
nnodes = size(SUPORT,1);
index = [];
DOF2 = NODE.DOF2;
for i=1:nnodes
    m = find(SUPORT(i,1) == [NODE.ID]);
    dof = num2str(SUPORT(i,2));
    for k=1:length(dof)
        index = [index, DOF2(m, str2num(dof(k)))];
        RINDEX(k) = str2num(dof(k));
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
%
