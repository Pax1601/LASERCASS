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
function solve_linflutt_corr
global beam_model;
global dlm_model;
global fl_model;
global fl_model2;

fid = beam_model.Param.FID;

if (~isempty(beam_model.Param.SOL == 150))
    fprintf(fid,'\nSolve trim problems\n');
    beam_model.Res = [];
    beam_model.FreeTrimRes = {};
    ntrim = beam_model.Info.ntrim;
    for nt = 1 :  ntrim
        solve_free_lin_trim(nt)
        beam_model.FreeTrimRes{nt} = beam_model.Res;
        beam_model.FreeTrimRes{nt}.lattice_defo = beam_model.Aero.lattice_defo;
    end
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
        solve_eig;
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
    end
    beam_model.Res.SOL = 'Linear flutter';
    NMODES = size(beam_model.Res.NDispl, 3);
    NMACH  = length(dlm_model.aero.M);
    NK     = length(dlm_model.aero.k);
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    np = beam_model.Aero.lattice.np;
    SPLINE_TYPE = beam_model.Info.spline_type;
    %   boundary condition
    dlm_model.data.D = assembly_AIC_matrix(fid, beam_model.Aero.lattice, dlm_model);
    for nt = 1 : ntrim
        V=beam_model.FreeTrimRes{nt}.state.AS;
        if isfield(beam_model.FreeTrimRes{nt}.Aero,'DTrim_sol')
            state = [beam_model.FreeTrimRes{nt}.Aero.DTrim_sol.Alpha*pi/180 , beam_model.FreeTrimRes{nt}.Aero.DTrim_sol.Betha*pi/180,...
                beam_model.FreeTrimRes{nt}.Aero.DTrim_sol.P, beam_model.FreeTrimRes{nt}.Aero.DTrim_sol.Q, beam_model.FreeTrimRes{nt}.Aero.DTrim_sol.R];
        else
            state = [beam_model.FreeTrimRes{nt}.Aero.RTrim_sol.Alpha*pi/180 , beam_model.FreeTrimRes{nt}.Aero.RTrim_sol.Betha*pi/180,...
                beam_model.FreeTrimRes{nt}.Aero.RTrim_sol.P, beam_model.FreeTrimRes{nt}.Aero.RTrim_sol.Q, beam_model.FreeTrimRes{nt}.Aero.RTrim_sol.R];
        end
        beam_model.Aero.lattice.N = -beam_model.FreeTrimRes{nt}.lattice_defo.N;
        if (SPLINE_TYPE==1)
          [dlm_model.data(nt).c_displ, dlm_model.data(nt).dwnwash, dlm_model.data(nt).n_displ,ROT] = ...
             set_boundary_condition_CORR1(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node, ...
                    beam_model.Res.NDispl, beam_model.Aero, true, false, state, V);%   assembly AIC matrix
        else
          [dlm_model.data(nt).c_displ, dlm_model.data(nt).dwnwash, dlm_model.data(nt).n_displ,ROT] = ...
             set_boundary_condition_CORR(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node.Coord, ...
                    beam_model.Res.NDispl, beam_model.Aero, true, false, state, V);%   assembly AIC matrix
        end
        dlm_model.data(nt).Cp = solve_system_CP(fid, np, NK, NMODES, NMACH, dlm_model.data(1).D, dlm_model.data(nt).dwnwash);
        gam = beam_model.FreeTrimRes{nt}.Gamma;
        VINF = V*([cos(state(1))*cos(state(2)) sin(state(2)) sin(state(1))*cos(state(2))]);
        if norm(state(3:end))
            ARM = (beam_model.Aero.lattice.COLLOC - repmat(beam_model.aero.geo.CG, np, 1));
            OMEGA_P = cross(ARM, repmat(state(3:end), np, 1), 2);
            VINF = VINF+OMEGA_P;
        end
        rhs_zero = dot(repmat(VINF,length(beam_model.Aero.lattice_vlm.N),1),beam_model.Aero.lattice.N,2);
        [gamma0, LK , VIND0] = get_aero_Gamma(rhs_zero,  beam_model.Aero.lattice_vlm, gam.Gamma_P, gam.Gamma_I,0);
        dlm_model.data(nt).Cp_corr = zeros(size(dlm_model.data(nt).Cp));
        for in = 1 : NMODES
            for ik = 1 : NK
                [dummy,dummy, VIND ] = get_aero_Gamma( -dlm_model.data(nt).dwnwash(:,in,ik),  beam_model.Aero.lattice_vlm,gam.Gamma_P, gam.Gamma_I,0);
                Fov = cross(VIND + 1i*dlm_model.aero.k(ik)/dlm_model.aero.cref*dlm_model.data(nt).c_displ(:,:,in)*V,LK.*repmat(gamma0,1,3));
                Fophi = (cross(VIND,LK) + cross(repmat(VINF,np,1),cross(ROT(:,:,in),LK))).*repmat(gamma0,1,3);
                dlm_model.data(nt).Cp_corr(:,in,ik) = conj(dot(Fov+Fophi,beam_model.Aero.lattice.N,2))./(beam_model.Aero.lattice.area)/(0.5*V^2);
            end
        end
        dlm_model.data(nt).Cp = dlm_model.data(nt).Cp + dlm_model.data(nt).Cp_corr;
        %   build aerodynamic transfer matrix
        dlm_model.data(nt).Qhh = set_generalized_f(fid, NMODES, dlm_model.data(nt).c_displ, NK, NMACH, beam_model.Aero.lattice.area(1:np), ...
            beam_model.Aero.lattice.N(1:np,:), dlm_model.data(nt).Cp);
        Gmm = zeros(NMODES); % complex stiff
        if (beam_model.Param.SDAMP)
          Gmm = beam_model.Res.Gmm;
        end
        %   export FFA structural model
        export_str_modal_ffa(fid, beam_model.Param.FILE, beam_model.Node.ID, beam_model.Node.Coord, beam_model.Res.NDispl, ...
                              beam_model.Res.ID, beam_model.Res.Mmm, beam_model.Res.Kmm, Gmm);
        %   export aerodynamic transfer matrix
        export_qhh_massa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data(nt).Qhh);
        %   require FFA matlab toolbox
        export_qhh_ffa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data(nt).Qhh, dlm_model.aero.cref);
        fprintf(fid, '\n\ndone.\n');
        %
        %   run flutter tracking solver
        %
        res = zeros(NMACH, 2);
        counter = 100;
        dtpos = find('.' == beam_model.Param.FILE);
        headname = [];
        if (isempty(beam_model.Param.MSELECT))
            mbase = [1:NMODES];
        else
            mbase = beam_model.Param.MSELECT;
        end
        
        if (isempty(beam_model.Param.FMODES))
            flwbase = mbase;
        else
            flwbase = beam_model.Param.FMODES;
        end
        
        vrange = [];
        rho = beam_model.Param.RHO_VG;
        
        vmax =  beam_model.Param.VMAX;
        vstep = vmax/beam_model.Param.NVSTEP;
        
        if (isempty(dtpos))
            headname = beam_model.Param.FILE;
        else
            headname = beam_model.Param.FILE(1:dtpos(end)-1);
        end
        strf = [headname, '.bmod'];
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
            
          Hamf = zeros(NMODES,NMODES,NK*2);
          Hamf(:,:,1:2:NK*2) = dlm_model.data.Qhh(:,:,1:NK,n);
          [VF{n}, HF{n}] = run_flutter(fid, beam_model.Res.Mmm, Gmm*beam_model.Res.Kmm, ...
                              beam_model.Res.Kmm, beam_model.Res.Omega./(2*pi), ...
                              beam_model.Res.ID, Hamf, dlm_model.aero.k, dlm_model.aero.M(n), CHREF, ...
                              mbase, flwbase, rho, [vstep, vmax], true, nfig, n);
          nfig = nfig + 1;
        end % Mach loop
        fl_model2{nt} = fl_model;
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
    end
    fl_model = fl_model2;
else
    %
    error('SOL 500 must be given in input file to run linear flutter analysis.');
    %
end

end
