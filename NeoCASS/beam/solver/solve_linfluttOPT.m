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
function [OBJ_REF, DOBJ, CSTR_IN, DCSTR_IN, CSTR_EQ, DCSTR_EQ, ...
          OBJ_SET, OUT_IN, OUT_EQ]=solve_linfluttOPT(X, OBJ_SET, IN_CSTR_SET, EQ_CSTR_SET,NOPT)

global beam_model;
global dlm_model;
global fl_model;
clear fl_model
fl_model = [];

fid = beam_model.Param.FID;

orig_beam_model = beam_model;
orig_fl_model = fl_model;

% compute the number of flutter cnst
nfcst = 0;
for i = size(beam_model.Param.follow,1)
    nfcst = nfcst+length(beam_model.Param.follow(i).mode);
end

ndes = length(X);
n_in = beam_model.Optim.Cstr.n_in+nfcst;%+length(beam_model.Optim.Cstr.In.NameF);
n_eq = beam_model.Optim.Cstr.n_eq;%+length(beam_model.Optim.Cstr.Eq.NameF);
% constraints still to be set
MISS_IN = setdiff(1:n_in, IN_CSTR_SET);
MISS_EQ = setdiff(1:n_eq, EQ_CSTR_SET);

OUT_IN = IN_CSTR_SET; %
OUT_EQ = EQ_CSTR_SET;

OBJ_REF = -1;
% constraints value
CSTR_IN = zeros(length(MISS_IN), 1);
CSTR_EQ = zeros(length(MISS_EQ), 1);
% OBJ and contraints gradients
DOBJ = zeros(1, ndes);
DCSTR_IN = zeros(length(MISS_IN), ndes);
DCSTR_EQ = zeros(length(MISS_EQ), ndes);

upd_beam_model = set_approx_model(beam_model, X);
beam_model = upd_beam_model;


% chek if K/p and M/p are passed as imput (from trim solution)
if (~isempty(find(beam_model.Param.MSOL == 150,1)))
    %
    %   set DLM struct
    %
    
    beam_model.Res = [];
    if ~isstruct(dlm_model)
        fprintf(fid,'\nRunning DLM solver for aerodynamic transfer matrix calculation...\n');
        dlm_model = set_struct(fid, beam_model.Aero.state.Mach_qhh, beam_model.Aero.state.Kfreq, beam_model.Aero.ref.C_mgc, beam_model.Aero.state.SIMXZ, ...
            beam_model.Param.DLM_ORDER, beam_model.Aero.lattice_dlm.dx, beam_model.Param.DLM_NP);
    end
    beam_model.Res = beam_model.Optim.Res.Eig;
    Kp = beam_model.Optim.Res.Eig.Kp;
    Mp = beam_model.Optim.Res.Eig.Mp;
    
    for i = 1 : length( beam_model.Param.follow.mode)
        if ~isempty(find(beam_model.Param.follow.mode(i)== beam_model.Param.follow.mode(setdiff(1:5,i)),1))
            beam_model.Param.follow.mode
            fprintf('\n\nOne mode is missing in tracking!!!!!!!!!!!!!!!!!!!!!!!!\n\n')
           pippo = 1;
        end
    end
    Fmode2 = beam_model.Param.follow;
   
    beam_model.Res.SOL = 'Linear flutter';
    NMODES = size(beam_model.Res.NDispl, 3);
    NMACH  = length(dlm_model.aero.M);
    NK     = length(dlm_model.aero.k);
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    np = beam_model.Aero.lattice.np;
    SPLINE_TYPE = beam_model.Info.spline_type;
    if isempty(dlm_model.data(1).D)
        dlm_model.data.D = assembly_AIC_matrix(fid, beam_model.Aero.lattice, dlm_model);
    end
    cont = length(IN_CSTR_SET)+1;
    cont2 = 1;
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    for Ntrim = 1:upd_beam_model.Info.cc_trim
        
        V=beam_model.Optim.Res.Free_trim{Ntrim}.state.AS;
        if isfield(beam_model.Optim.Res.Free_trim{Ntrim}.Aero,'DTrim_sol')
            state = [beam_model.Optim.Res.Free_trim{Ntrim}.Aero.DTrim_sol.Alpha*pi/180 , beam_model.Optim.Res.Free_trim{Ntrim}.Aero.DTrim_sol.Betha*pi/180,...
                beam_model.Optim.Res.Free_trim{Ntrim}.Aero.DTrim_sol.P, beam_model.Optim.Res.Free_trim{Ntrim}.Aero.DTrim_sol.Q, beam_model.Optim.Res.Free_trim{Ntrim}.Aero.DTrim_sol.R];
        else
            state = [beam_model.Optim.Res.Free_trim{Ntrim}.Aero.RTrim_sol.Alpha*pi/180 , beam_model.Optim.Res.Free_trim{Ntrim}.Aero.RTrim_sol.Betha*pi/180,...
                beam_model.Optim.Res.Free_trim{Ntrim}.Aero.RTrim_sol.P, beam_model.Optim.Res.Free_trim{Ntrim}.Aero.RTrim_sol.Q, beam_model.Optim.Res.Free_trim{Ntrim}.Aero.RTrim_sol.R];
        end
        beam_model.Aero.lattice.N = -beam_model.Optim.Res.Free_trim{Ntrim}.lattice_defo.N;
        %   boundary condition
        if (SPLINE_TYPE==1)
          [dlm_model.data(Ntrim).c_displ, dlm_model.data(Ntrim).dwnwash, dlm_model.data(Ntrim).n_displ,ROT] = set_boundary_condition_CORR1(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node, ...
            beam_model.Res.NDispl, beam_model.Aero, true, false,state,V);
        else
          [dlm_model.data(Ntrim).c_displ, dlm_model.data(Ntrim).dwnwash, dlm_model.data(Ntrim).n_displ,ROT] = set_boundary_condition_CORR(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node.Coord, ...
            beam_model.Res.NDispl, beam_model.Aero, true, false,state,V);
        end        
        dlm_model.data(Ntrim).Cp = solve_system_CP(fid, np, NK, NMODES, NMACH, dlm_model.data(1).D, dlm_model.data(Ntrim).dwnwash);
        VINF = V*([cos(state(1))*cos(state(2)) sin(state(2)) sin(state(1))*cos(state(2))]);
        if norm(state(3:end))
            ARM = (beam_model.Aero.lattice.COLLOC - repmat(beam_model.aero.geo.CG, np, 1));
            OMEGA_P = cross(ARM, repmat(state(3:end), np, 1), 2);
            VINF = VINF+OMEGA_P;
        end
        rhs_zero = dot(repmat(VINF,length(beam_model.Aero.lattice.N),1),beam_model.Aero.lattice.N,2);
        gam = beam_model.Optim.Res.Free_trim{Ntrim}.Gamma;
        [gamma0, LK ] = get_aero_Gamma(rhs_zero,  beam_model.Aero.lattice_vlm, gam.Gamma_P, gam.Gamma_I,0);
        
        dlm_model.data(Ntrim).Cp_corr = zeros(size(dlm_model.data(Ntrim).Cp));
        for in = 1 : NMODES
            for ik = 1 : NK
                %            gammaDLM = 0.5*V*beam_model.Aero.lattice_dlm.cord .*dlm_model.data.Cp(:,in,ik);
                [dummy,dummy, VIND ] = get_aero_Gamma( dlm_model.data(Ntrim).dwnwash(:,in,ik),  beam_model.Aero.lattice_vlm, gam.Gamma_P, gam.Gamma_I,0);
                Fov = cross(VIND-1i*dlm_model.aero.k(ik)/dlm_model.aero.cref*dlm_model.data(Ntrim).c_displ(:,:,in)*V,LK.*repmat(gamma0,1,3));
                Fophi = (cross(VIND,LK) + cross(repmat(VINF,np,1),cross(ROT(:,:,in),LK))).*repmat(gamma0,1,3);
                dlm_model.data(Ntrim).Cp_corr(:,in,ik) = dot(Fov+Fophi,beam_model.Aero.lattice.N,2)./(beam_model.Aero.lattice.area)/(0.5*V^2);
            end
        end
        dlm_model.data(Ntrim).Cp = dlm_model.data(Ntrim).Cp + dlm_model.data(Ntrim).Cp_corr;
        
        
        %   solve system and determine Cp for each mode, reduced frequency and Mach
        %   number
        
        %   build aerodynamic transfer matrix
        dlm_model.data(Ntrim).Qhh = set_generalized_f(fid, NMODES, dlm_model.data(Ntrim).c_displ, NK, NMACH, beam_model.Aero.lattice.area(1:np), ...
            beam_model.Aero.lattice.N(1:np,:), dlm_model.data(Ntrim).Cp);
        %   export FFA structural model
        export_str_modal_ffa(fid, beam_model.Param.FILE, beam_model.Node.ID, beam_model.Node.Coord, beam_model.Res.NDispl, beam_model.Res.ID, beam_model.Res.Mmm, beam_model.Res.Kmm);
        %   export aerodynamic transfer matrix
        export_qhh_massa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data(Ntrim).Qhh);
        %   require FFA matlab toolbox
        export_qhh_ffa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data(Ntrim).Qhh, dlm_model.aero.cref);
        fprintf(fid, '\n\ndone.\n');
        
        %
        %   run flutter tracking solver
        %
        %     res = zeros(NMACH, 2);
        counter = 100;
        dtpos = find('.' == beam_model.Param.FILE);
        headname = [];
        if (isempty(beam_model.Param.MSELECT))
            mbase = 1:NMODES;
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
        
        %
        
        
        %   Mach number loop
        
        for n=1:NMACH
            
            aerof = strcat(headname, '_M');
            string = sprintf('%4.3f', dlm_model.aero.M(n));
            aerof =[aerof, string,'.baer'];
            %
            %       [VF{n}, HF{n}] = ...
            %          run_flutter_ffa(strf, aerof, mbase, flwbase, rho, [vstep, vmax], true, counter, n);
            OUT(n,:) = run_flutter_ffaOPT(strf, aerof, mbase, flwbase, rho, [vstep, vmax], true, counter, n);
            counter = counter + 1;
            %beam_model.Param.AUTOPLOT
            if NOPT ==1
                mode = zeros(size(beam_model.Param.follow(n).mode));
                for i = 1 : length(beam_model.Param.follow(n).mode)
                    %                 indmode1 = find(Fmode(n).mode == beam_model.Param.follow(n).mode(i));
                    %                 indmode = find(flwbase == beam_model.Param.follow(n).mode(indmode1));
                    
                    indmode = find(flwbase == beam_model.Param.follow(n).mode(i));
                    mode(i) = trapz(OUT(n,indmode).ris(:,2),abs(OUT(n,indmode).ris(:,3)) )*0.5;
                end
                orig_beam_model.Optim.Res.Flu.Factor(n).mode = mode;
            else
                [dummy,ordine] = sort(Fmode2(n).mode,2);
                orig_beam_model.Optim.Res.Flu.Factor(n).mode = orig_beam_model.Optim.Res.Flu.Factor(n).mode(ordine);
            end
            %         compute constrained
            for i = 1 : length(beam_model.Param.follow(n).mode)
                
                %             indmode1 = find(Fmode(n).mode == beam_model.Param.follow(n).mode(i));
                %             indmode = find(flwbase == beam_model.Param.follow(n).mode(indmode1));
                
                indmode = find(flwbase == beam_model.Param.follow(n).mode(i));
                [CSTR_IN(cont2),v,IND] = flutter_cstr(OUT(n,indmode),beam_model.Param.curve,orig_beam_model.Optim.Res.Flu.Factor(n).mode(i));
                
                % compute derivative
                for j = 1: ndes
                    dummy = flutter_dcstr(OUT(n,indmode),beam_model.Param.curve,Kp(:,:,j),Mp(:,:,j),IND,v,orig_beam_model.Optim.Res.Flu.Factor(n).mode(i), beam_model.Param.DIRDER*beam_model.Optim.X(j)*beam_model.Optim.Xnorm(j));
                    DCSTR_IN(cont2,j) = (dummy-CSTR_IN(cont2))/beam_model.Param.DIRDER;
                end
                OUT_IN = [OUT_IN,cont];
                cont = cont+1;
                cont2 = cont2+1;
            end
            
            
        end % Mach loop
        
        
    end
    beam_model = orig_beam_model;
    
    fl_model = orig_fl_model;
    beam_model.Aero.lattice = [];
elseif (~isempty(find(beam_model.Param.MSOL == 145,1)))
    
    
    %
    %   set DLM struct
    %
    
    beam_model.Res = [];
    if ~isstruct(dlm_model)
        fprintf(fid,'\nRunning DLM solver for aerodynamic transfer matrix calculation...\n');
        dlm_model = set_struct(fid, beam_model.Aero.state.Mach_qhh, beam_model.Aero.state.Kfreq, beam_model.Aero.ref.C_mac, beam_model.Aero.state.SIMXZ, ...
            beam_model.Param.DLM_ORDER);
    end
    %
    %   run eigenvalue solver
    %
    
    
    %     if isempty(beam_model.Param.EIG_FILE)
    % il modello è cambiato gli autovalori e gli autovettori devono
    % essere sempre ricalcolati...per ora
    
    if (isempty(find(beam_model.Param.MSOL == 144,1)))
        dummy = beam_model;
        beam_model.Param.SUPORT = [];
        solve_eig;
        dummy.Res = beam_model.Res;
        beam_model = dummy;
        % if nargin ~= 4 there is trim solution too, therefore OBJ is
        % already set
        if (~OBJ_SET) % if OBJ is not set, try to set it
            [OBJ_REF, OBJ_SET] = beam_model.Param.OBJ(beam_model);
        end
        %    Compute K/p and M/p (global matrices)
        fprintf(fid, '\n - Evaluating perturbations...');
        K = beam_model.Res.Kmm;
        M = beam_model.Res.Mmm;
%         K =  st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, ...
%             beam_model.Node.Coord, beam_model.Bar, beam_model.Beam);
%         M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, ...
%             beam_model.ConM, beam_model.Bar, beam_model.Beam);
        
        Kp = zeros([beam_model.Param.NROOTS,beam_model.Param.NROOTS,length(X)]);
        Mp = Kp;
        for I = 1:ndes
            %
            fprintf(fid, '\n Perturbation n. %d.\n', I);
            XC = X;
            % perturb desvar I
            XC(I) = XC(I) * (1 + beam_model.Param.DIRDER);
            dummy = set_approx_model(upd_beam_model, XC);
            
            K2 = st_lin_matrix(dummy.Info, dummy.Node.DOF, dummy.Node.R, ...
                dummy.Node.Coord, dummy.Bar, dummy.Beam, dummy.Celas);
            M2 = ms_matrix(dummy.Info, dummy.Node.DOF, dummy.Node.R, ...
                dummy.ConM, dummy.Bar, dummy.Beam);
            if ~isempty(beam_model.RBE2)
                K2 = RBE2Assembly(beam_model.RBE2,K2);
                M2 = RBE2Assembly(beam_model.RBE2,M2);
            end
            
            warning off;
            %     force matrix to be simmetrix due to tolerances for eigs solver on Windows system
            M2 = (M2 + M2') ./ 2.0;
            opts.disp = 0;
            [V, D] = eigs(K2, M2, beam_model.Param.NROOTS, 'SM', opts);
            warning on;
            V = real(V);
            NMODES = beam_model.Param.NROOTS;
            %             only normalization unitary
            %             mass!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            
            
            for m=1:NMODES
                mscale = V(:,m)' * M2 * V(:,m);
                V(:,m) = V(:,m) / sqrt(mscale);
            end
             
            mm = diag(diag(V' * M2 * V)); 
            km = diag(diag(V' * K2 * V));
            
            Kp(:,:,I) = (km-K)/(XC(I)*beam_model.Param.DIRDER*beam_model.Optim.Xnorm(I)); 
            Mp(:,:,I) = (mm-M)/(XC(I)*beam_model.Param.DIRDER*beam_model.Optim.Xnorm(I));
            if (OBJ_REF~=-1)
                [OBJ, OBJ_SET] = beam_model.Param.OBJ(dummy);
                DOBJ(I) = (OBJ-OBJ_REF) / (beam_model.Param.DIRDER);
            end
            
        end % param loop 
        fprintf(fid, 'done.');
        % we have to check order of eigenvalues (and eigenvectors)
        % and if needed, change the vector INDfollow to ensure to look at
        % chosen eigenvalues.
        if ~(isfield(orig_beam_model,'Optim') && isfield(orig_beam_model.Optim,'Res') && isfield(orig_beam_model.Optim.Res,'Eig'))
            orig_beam_model.Optim.Res.Eig = beam_model.Res;
        end
        
        
        transform = Check_eigenvector(orig_beam_model.Optim.Res.Eig.V,beam_model.Res.V, beam_model.Res.M,beam_model.Param.MSELECT);
        
        % update eigenvector base
        orig_beam_model.Optim.Res.Eig.V = beam_model.Res.V;
        % update follow mode
        
        Fmode = beam_model.Param.follow;
        Fmode2 = Fmode;
        for i = 1 : length(Fmode) % loop on Mach number
            Fmode2(i).mode = transform(2,Fmode(i).mode);
        end
        beam_model.Param.follow = Fmode2;
        orig_beam_model.Param.follow = Fmode2;
    else
        beam_model.Res = beam_model.Optim.Res.Eig;
        Kp = beam_model.Optim.Res.Eig.Kp;
        Mp = beam_model.Optim.Res.Eig.Mp;
    end
    
%     for i = 1 : length( beam_model.Param.follow.mode)
%         if ~isempty(find(beam_model.Param.follow.mode(i)== beam_model.Param.follow.mode(setdiff(1:5,i)),1))
            beam_model.Param.follow.mode
%             fprintf('\n\nTi sei perso un modo!!!!!!!!!!!!!!!!!!!!!!!!\n\n')
%            pippo = 1;
%         end
%     end
    Fmode2 = beam_model.Param.follow;
   
    beam_model.Res.SOL = 'Linear flutter';
    NMODES = size(beam_model.Res.NDispl, 3);
    NMACH  = length(dlm_model.aero.M);
    NK     = length(dlm_model.aero.k);
    beam_model.Aero.lattice = beam_model.Aero.lattice_dlm;
    SPLINE_TYPE = beam_model.Info.spline_type;
    np = beam_model.Aero.lattice.np;
    %   boundary condition
    if (SPLINE_TYPE==1)
      [dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ] = ...
           set_boundary_condition1(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node, ...
           beam_model.Res.NDispl, beam_model.Aero, true, false);  
    else
      [dlm_model.data.c_displ, dlm_model.data.dwnwash, dlm_model.data.n_displ] = set_boundary_condition(fid, beam_model.Aero.geo, beam_model.Aero.lattice, dlm_model, beam_model.Node.Coord, ...
          beam_model.Res.NDispl, beam_model.Aero, true, false);%   assembly AIC matrix
    end
    %   solve system and determine Cp for each mode, reduced frequency and Mach
    %   number 
    dlm_model.data.Cp = solve_system_CP(fid, np, NK, NMODES, NMACH, dlm_model.data.D, dlm_model.data.dwnwash);
    %   build aerodynamic transfer matrix
    dlm_model.data.Qhh = set_generalized_f(fid, NMODES, dlm_model.data.c_displ, NK, NMACH, beam_model.Aero.lattice.area(1:np), ...
        beam_model.Aero.lattice.N(1:np,:), dlm_model.data.Cp);
    %   export FFA structural model
    export_str_modal_ffa(fid, beam_model.Param.FILE, beam_model.Node.ID, beam_model.Node.Coord, beam_model.Res.NDispl, beam_model.Res.Mmm, beam_model.Res.Kmm);
    %   export aerodynamic transfer matrix
    export_qhh_massa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh);
    %   require FFA matlab toolbox
    export_qhh_ffa(fid, beam_model.Param.FILE, dlm_model.aero.k, dlm_model.aero.M, dlm_model.data.Qhh, dlm_model.aero.cref);
    fprintf(fid, '\n\ndone.\n');
    beam_model.Aero.lattice = [];
    %
    %   run flutter tracking solver
    %
%     res = zeros(NMACH, 2);
    counter = 100;
    dtpos = find('.' == beam_model.Param.FILE);
    headname = [];
    if (isempty(beam_model.Param.MSELECT))
        mbase = 1:NMODES;
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
    
    %
    
    
    %   Mach number loop
    cont = length(IN_CSTR_SET)+1;
    cont2 = 1;
    for n=1:NMACH
        
        aerof = strcat(headname, '_M');
        string = sprintf('%4.3f', dlm_model.aero.M(n));
        aerof =[aerof, string,'.baer'];
        %
        %       [VF{n}, HF{n}] = ...
        %          run_flutter_ffa(strf, aerof, mbase, flwbase, rho, [vstep, vmax], true, counter, n);
        OUT(n,:) = run_flutter_ffaOPT(strf, aerof, mbase, flwbase, rho, [vstep, vmax], true, counter, n);
        counter = counter + 1;
        %beam_model.Param.AUTOPLOT
        if NOPT ==1
            mode = zeros(size(beam_model.Param.follow(n).mode));
            for i = 1 : length(beam_model.Param.follow(n).mode)
%                 indmode1 = find(Fmode(n).mode == beam_model.Param.follow(n).mode(i));
%                 indmode = find(flwbase == beam_model.Param.follow(n).mode(indmode1));

                indmode = find(flwbase == beam_model.Param.follow(n).mode(i));
                mode(i) = trapz(OUT(n,indmode).ris(:,2),abs(OUT(n,indmode).ris(:,3)))*0.5;
            end
            orig_beam_model.Optim.Res.Flu.Factor(n).mode = mode;
        else
            [dummy,ordine] = sort(Fmode2(n).mode,2);
            orig_beam_model.Optim.Res.Flu.Factor(n).mode = orig_beam_model.Optim.Res.Flu.Factor(n).mode(ordine);
        end 
        %         compute constrained 
        for i = 1 : length(beam_model.Param.follow(n).mode)
            
            %             indmode1 = find(Fmode(n).mode == beam_model.Param.follow(n).mode(i));
            %             indmode = find(flwbase == beam_model.Param.follow(n).mode(indmode1));
            
            indmode = find(flwbase == beam_model.Param.follow(n).mode(i));
            [CSTR_IN(cont2),v,IND] = flutter_cstr(OUT(n,indmode),beam_model.Param.curve,orig_beam_model.Optim.Res.Flu.Factor(n).mode(i));
            
            % compute derivative
            for j = 1: ndes
                dummy = flutter_dcstr(OUT(n,indmode),beam_model.Param.curve,Kp(:,:,j),Mp(:,:,j),IND,v,orig_beam_model.Optim.Res.Flu.Factor(n).mode(i), beam_model.Param.DIRDER*XC(j)*beam_model.Optim.Xnorm(j));
                DCSTR_IN(cont2,j) = (dummy-CSTR_IN(cont2))/beam_model.Param.DIRDER;
            end
            OUT_IN = [OUT_IN,cont]; 
            cont = cont+1;
            cont2 = cont2+1;
        end
        
        
    end % Mach loop
    
    beam_model = orig_beam_model;
    
    fl_model = orig_fl_model;
    
    
    
else
    %
    error('SOL 145 must be given in input file to run linear flutter analysis.');
    %
end

end
