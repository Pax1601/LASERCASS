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
function solve_linflutt
% tic
global beam_model;
global dlm_model;
global fl_model;

fid = beam_model.Param.FID;

if (~isempty(find(beam_model.Param.MSOL == 145)))
    
    fprintf(fid,'\nRunning DLM solver for aerodynamic transfer matrix calculation...\n');
    %
    %   set DLM struct
    %
    beam_model.Res = [];
    dlm_model = set_struct(fid, beam_model.Aero.state.Mach_qhh, beam_model.Aero.state.Kfreq, beam_model.Aero.ref.C_mgc, beam_model.Aero.state.SIMXZ, ...
        beam_model.Param.DLM_ORDER, beam_model.Aero.lattice_dlm.dx, beam_model.Param.DLM_NP);
    %
    %   run eigenvalue solver
    %
    if isempty(beam_model.Param.EIG_FILE)
        %
        solve_eig;
        %
    else
        % load data from external files
        filename = [beam_model.Param.EIG_FILE, '.mas'];  beam_model.Res.Mmm = load(filename);
        filename = [beam_model.Param.EIG_FILE, '.stf'];  beam_model.Res.Kmm = load(filename);
        filename = [beam_model.Param.EIG_FILE, '.mod'];  EXT_MODES = load(filename);
        ngrid = beam_model.Info.ngrid;
        if mod(size(EXT_MODES, 1), ngrid)
            error('Mode shape file %s does not contain displacements for all vibration modes.', filename);
        end
        NMODES = size(beam_model.Res.Mmm, 1);
        ncol = size(EXT_MODES, 2);
        beam_model.Res.NDispl = zeros(ngrid, ncol-1, NMODES);
        id = EXT_MODES(1:ngrid, 1);
        [v, i1, i2] = intersect(beam_model.Node.ID, id);
        counter = 0;
        for n=1:NMODES
            beam_model.Res.NDispl(i1, 1:ncol-1, n) = EXT_MODES(i2 + counter, 2:ncol);
            counter = counter + ngrid;
        end
        beam_model.Res.Omega = sqrt(diag(beam_model.Res.Kmm)./diag(beam_model.Res.Mmm));
        Freq = beam_model.Res.Omega ./ (2*pi);
        beam_model.Res.ID = [1:NMODES];
        fprintf(fid,' \n ### Warning: loaded mode shapes will have sequential ID number.');
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
        
        if (~isempty(beam_model.Param.MSELECT))
            [v, ia, ib] = intersect(beam_model.Res.ID, beam_model.Param.MSELECT);
            index = intersect(index, ia);
            beam_model.Res.Omega = beam_model.Res.Omega(index);
            Freq = Freq(index);
            beam_model.Res.ID = beam_model.Res.ID(index);
            beam_model.Res.NDispl = beam_model.Res.NDispl(:,:,index);
            beam_model.Res.Mmm = beam_model.Res.Mmm(index,index);
            beam_model.Res.Kmm = beam_model.Res.Kmm(index,index);
            NMODES = length(index);
            SDAMP = beam_model.Param.SDAMP;
            if (beam_model.Param.SDAMP)
                fprintf(fid,'\n - Building complex stiffness matrix for damping...');
                DAMP = beam_model.Damp;
                beam_model.Res.Gmm = modal_damp(DAMP.g{SDAMP}, DAMP.Freq{SDAMP}, DAMP.Type(SDAMP), 0, ...
                    beam_model.Res.Mmm, beam_model.Res.Omega./(2*pi));
                fprintf(fid, 'done.');
            end
            
        end
        %
    end
    
    beam_model.Res.SOL = 'Linear flutter';
    NMODES = size(beam_model.Res.NDispl, 3);
    NMACH  = length(dlm_model.aero.M);
    NK     = length(dlm_model.aero.k);
    SPLINE_TYPE = beam_model.Info.spline_type;
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    np = beam_model.Aero.lattice.np;
    %   boundary condition
    
    if (SPLINE_TYPE==1)
        [dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ, DN] = ...
            set_boundary_condition1(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node, ...
            beam_model.Res.NDispl, beam_model.Aero, true, false);
    else
        [dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ, DN] = ...
            set_boundary_condition(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node.Coord, ...
            beam_model.Res.NDispl, beam_model.Aero, true, false);
    end
         
    nptot = beam_model.Aero.lattice_dlm.np;
    x0 = dlm_model.aero.cref .* beam_model.Aero.lattice.XYZ(1:nptot,:,:);
    
    for i = 1: length(x0)
        v1 = [x0(i, 1, 1); x0(i, 1, 2); x0(i, 1, 3)];
        v2 = [x0(i, 2, 1); x0(i, 2, 2); x0(i, 2, 3)];
        v3 = [x0(i, 3, 1); x0(i, 3, 2); x0(i, 3, 3)];
        v4 = [x0(i, 4, 1); x0(i, 4, 2); x0(i, 4, 3)];
        
        b1 = norm(v4 - v1);
        b2 = norm(v3 - v2);
        costheta = dot((v2 - v1) / norm(v2 - v1), (v4 - v1) / norm(v4 - v1));
        h = sqrt(1 - costheta ^ 2) * norm(v2 - v1);
        dS(i) = (b1 + b2) / 2 * h;
        
    end
    for nsh = 1 : NMODES
                
        XYZ = zeros(nptot,5,3);
        count  = 0;
        for k = 1:nptot
            for m = 1:4
                count = count +1;
                XYZ(k, m, 1:3) = dlm_model.data.n_displ(count, 1:3, nsh);
            end
            XYZ(k, 5, 1:3) = XYZ(k, 1, 1:3);
        end
                   
        x0_csi = (x0(:, 2, :) - x0(:, 1, :));
        x0_eta = (x0(:, 4, :) - x0(:, 1, :));
        s_csi = (XYZ(:, 2, :) - XYZ(:, 1, :));
        s_eta = (XYZ(:, 4, :) - XYZ(:, 1, :));
        
        for i = 1: length(x0_csi)
            
            v1 = [x0(i, 1, 1); x0(i, 1, 2); x0(i, 1, 3)];
            v2 = [x0(i, 2, 1); x0(i, 2, 2); x0(i, 2, 3)];
            v3 = [x0(i, 3, 1); x0(i, 3, 2); x0(i, 3, 3)];
            v4 = [x0(i, 4, 1); x0(i, 4, 2); x0(i, 4, 3)];

            b1 = norm(v4 - v1);
            b2 = norm(v3 - v2);
            costheta = dot((v2 - v1) / norm(v2 - v1), (v4 - v1) / norm(v4 - v1));
            d_csi = sqrt(1 - costheta ^ 2) * norm(v2 - v1);
            
            
            v_x0_csi = [x0_csi(i, 1, 1); x0_csi(i, 1, 2); x0_csi(i, 1, 3)];
            v_x0_csi = v_x0_csi / d_csi;
            v_x0_eta = [x0_eta(i, 1, 1); x0_eta(i, 1, 2); x0_eta(i, 1, 3)];
            v_x0_eta = v_x0_eta / b1;
            
            v_x1_csi = [x0_csi(i, 1, 1) + s_csi(i, 1, 1); x0_csi(i, 1, 2) + s_csi(i, 1, 2); x0_csi(i, 1, 3) + s_csi(i, 1, 3)];
            v_x1_csi = v_x1_csi / d_csi;
            v_x1_eta = [x0_eta(i, 1, 1) + s_eta(i, 1, 1); x0_eta(i, 1, 2) + s_eta(i, 1, 2); x0_eta(i, 1, 3) + s_eta(i, 1, 3)];
            v_x1_eta = v_x1_eta / b1;
            
            N1 = cross(v_x0_csi, v_x0_eta);
            N1 = N1 / norm(N1);
            N2 = cross(v_x1_csi, v_x1_eta);
            N2 = N2 / norm(N2);
            deltaN = N2 - N1;
            
            cross_prod_mat{i, nsh} = deltaN; 
            phi_d_mat{nsh} = XYZ(:, 1, :);
        end
    end

    for i = 1: length(x0_csi)
        temp_mat{i} = [];
        temp_mat2{i} = [];
        for nsh = 1 : NMODES
            temp_mat{i} = [temp_mat{i} cross_prod_mat{i, nsh}];
            t_vect = phi_d_mat{nsh};
            temp_mat2{i} = [temp_mat2{i} [t_vect(i, 1, 1); t_vect(i, 1, 2); t_vect(i, 1, 3)]];
        end       
    end

    beam_model.Aero.Nvar = temp_mat;
    beam_model.Aero.Phid = temp_mat2;
    
    dlm_model.data.D = assembly_AIC_matrix(fid, beam_model.Aero.lattice, dlm_model);
    %   solve system and determine Cp for each mode, reduced frequency and Mach number
    %
    dlm_model.data.Cp = solve_system_CP(fid, np, NK, NMODES, NMACH, dlm_model.data.D, dlm_model.data.dwnwash);
    %   build aerodynamic transfer matrix
    dlm_model.data.Qhh = set_generalized_f(fid, NMODES, dlm_model.data.c_displ, NK, NMACH, ...
        beam_model.Aero.lattice.area(1:np), ...
        beam_model.Aero.lattice.N(1:np,:), dlm_model.data.Cp);
    Gmm = zeros(NMODES); % complex stiff
    if (beam_model.Param.SDAMP)
        Gmm = beam_model.Res.Gmm;
    end
    %   export FFA structural model
    export_str_modal_ffa(fid, beam_model.Param.FILE, beam_model.Node.ID, beam_model.Node.Coord, beam_model.Res.NDispl, ...
        beam_model.Res.ID, beam_model.Res.Mmm, beam_model.Res.Kmm, Gmm);
    %   export aerodynamic transfer matrix
    export_qhh_massa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh);
    %   require FFA matlab toolbox
    export_qhh_ffa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh, dlm_model.aero.cref);
    fprintf(fid, '\n\ndone.\n');
    %
    %   run flutter tracking solver
    %
    res = zeros(NMACH, 2);
    counter = 100;
    mbase = beam_model.Res.ID;
    %
    if (isempty(beam_model.Param.FMODES))
        flwbase = mbase;
    else
        flwbase = beam_model.Param.FMODES;
    end
    vrange = [];
    rho = beam_model.Param.RHO_VG;
    vmax =  beam_model.Param.VMAX;
    vstep = vmax/beam_model.Param.NVSTEP;
    %   clean or define Res field
    fl_model.Res = [];
    fl_model.Res.data = [];
    fl_model.Res.Env = [];
    %
    %   Mach number loop
    %
    CHREF = dlm_model.aero.cref;
    nfig = 100;
    for n=1:NMACH

        %
        Hamf = zeros(NMODES,NMODES,NK*2);
        Hamf(:,:,1:2:NK*2) = dlm_model.data.Qhh(:,:,1:NK,n);
        [VF{n}, HF{n}] = run_flutter(fid, beam_model.Res.Mmm, Gmm*beam_model.Res.Kmm, ...
            beam_model.Res.Kmm, beam_model.Res.Omega./(2*pi), ...
            beam_model.Res.ID, Hamf, dlm_model.aero.k, dlm_model.aero.M(n), CHREF, ...
            mbase, flwbase, rho, [vstep, vmax], true, nfig, n);
        nfig = nfig + 1;
        %
        %      [VF{n}, HF{n}] = ...
        %         run_flutter_ffa(strf, aerof, mbase, flwbase, rho, [vstep, vmax], true, counter, n);
        
    end % Mach loop
    %
    if (rho == 1.225)
        for n=1:NMACH
            if (~isempty(VF{n}))
                figure(nfig); close; figure(nfig); hold on;
                figure(nfig+1); close; figure(nfig+1); hold on;
                figure(nfig);
                plot(repmat(beam_model.Aero.state.Mach_qhh(n),1,length(VF{n})), VF{n},'-ro', 'MarkerSize', 6, 'MarkerFaceColor','r');
                figure(nfig+1);
                plot(repmat(beam_model.Aero.state.Mach_qhh(n),1,length(VF{n})), HF{n},'-ro', 'MarkerSize', 6, 'MarkerFaceColor','r');
                plot([0 1], [11 11],'-k');
                plot([0 1], [25 25],'-k');
                plot([1 1], [0 25],'-k');
                figure(nfig); grid;
                axis([0 1.0 0.0 2*max(VF{n})]);
                title('Flutter envelope'); ylabel('VF_{EAS}');   xlabel('Mach');
                figure(nfig+1); grid; title('Flutter envelope'); ylabel('Altitude'); xlabel('Mach');
                nfig = nfig+2;
            else
                fprintf(fid, '\n - No flutter envelope to plot for Mach %g.', beam_model.Aero.state.Mach_qhh(n));
            end
        end
    end
    fprintf(fid, '\ndone.\n\n');
else
    %
    error('SOL 145 must be given in input file to run linear flutter analysis.');
    %
end
fprintf(fid, '\n');
end
