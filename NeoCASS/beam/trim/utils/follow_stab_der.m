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
%                      Sergio Ricci           <ricci@aero.polimi.it>
%                      Luca Cavagna           <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari  <degaspari@aero.polimi.it>
%                      Luca Riccobene         <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function follow_stab_der(varargin)

global beam_model;
%
fid = beam_model.Param.FID;
%
if nargin == 2
    DUMMY_INDEX = varargin{1};
    if (length(DUMMY_INDEX)>1)
        fprintf(fid, '\nWarning: only the first trim case will be run. The following will be ignored.');
        fprintf(fid, '\n         Rerun the solver.')
    end
    TRIM_INDEX = DUMMY_INDEX(1);
    range = varargin{2};
    %
else
    TRIM_INDEX = 1;
    range = varargin{1};
end
%

LOAD_SCALE = 1.0;
EPS = D2R(0.001); % perturbation value to extract aerodynamic derivatives

if (~isempty(find(beam_model.Param.MSOL == 144)))
    
    fprintf(fid,'\nSolving linear static unrestrained trim...\n\n');
    
    nc = beam_model.Aero.geo.nc;
    % switch off the pg correction
    beam_model.Aero.state.pgcorr = 0;
    
    %
    %   select trim case
    %
    beam_model.Aero.lattice = beam_model.Aero.lattice_vlm; % already defined for null variables (alpha, beta, p q r)
    index = find(beam_model.Aero.Trim.Select(TRIM_INDEX) == beam_model.Aero.Trim.ID);
    
    
    if ~isempty(index)
        
        switch (beam_model.Aero.Trim.Type(index))
            
            case 0
                NDOF = 6;
                STRIM = 0;
                FMDOF = [];
            case 1 % full simmetric trim
                NDOF = 3;
                STRIM = 1;
                FMDOF = [1,3,5];
            case 2 % pitch spin
                NDOF = 1;
                STRIM = 1;
                FMDOF = [5];
            case -1 % full anti-simmetric trim
                NDOF = 3;
                STRIM = 1;
                FMDOF = [2,4,6];
            case -2 % roll spin
                NDOF = 1;
                STRIM = 1;
                FMDOF = [4];
            case -3 % yaw spin
                NDOF = 1;
                STRIM = 1;
                FMDOF = [6];
                %
        end
        %
        % set flight mechanics trim params
        beam_model.Aero.Trim.FM = get_free_body_trim_params(beam_model.Aero.Trim.NC(index), ...
            beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data);
        [beam_model.Aero.state.alpha, beam_model.Aero.state.betha, beam_model.Aero.state.P, beam_model.Aero.state.Q, beam_model.Aero.state.R] = ...
            get_state_trim_vars(beam_model.Aero.Trim.FM);
        beam_model.Aero.state.alpha = D2R(beam_model.Aero.state.alpha);
        beam_model.Aero.state.betha = D2R(beam_model.Aero.state.betha);
        % make sure angles are converted
        beam_model.Aero.Trim.FM.Value(2) = D2R(beam_model.Aero.Trim.FM.Value(2));
        beam_model.Aero.Trim.FM.Value(3) = D2R(beam_model.Aero.Trim.FM.Value(3));
        % set Tornado state struct
        beam_model.Aero.state.ALT = beam_model.Aero.Trim.ALT(index);
        [beam_model.Aero.state.rho, p, T, a, mu] = ISA_h(beam_model.Aero.state.ALT);
        beam_model.Aero.state.AS = beam_model.Aero.Trim.Mach(index) * a;
        beam_model.Aero.state.Mach(1) = beam_model.Aero.Trim.Mach(index);
        %
    else
        error('Unable to find the required TRIM set %d.', TRIM_INDEX);
    end
    
    if nc
        beam_model.Aero.Trim.CS = get_control_surf_trim_params(beam_model.Aero.geo.nc, beam_model.Aero.Trim.NC(index), ...
            beam_model.Aero.Trim.Param(index).data, beam_model.Aero.Trim.Value(index).data, beam_model.Aero.lattice.Control);
    end
    % check trim variables
    ncs = sum(beam_model.Aero.Trim.CS.Fixed);
    nfm = sum(beam_model.Aero.Trim.FM.Fixed);
    if (ncs + nfm) ~= beam_model.Aero.Trim.NC(index)
        
        error('Unable to fix trim variables. Wrong variable name given in TRIM card %d.', ...
            beam_model.Aero.Trim.ID(index));
        
    end
    %***********************************************************************************************************************
    % check for duplicated AELINK labels
    if (beam_model.Info.nlink)
        
        [labels, i] = unique(beam_model.Aero.Trim.Link.ID);
        if (length(labels) ~= beam_model.Info.nlink)
            
            n = [1 : beam_model.Info.nlink];
            dof = beam_model.Aero.Trim.Link.ID(setdiff(n, i));
            
            for k=1:length(dof)
                
                fprintf(fid, '\n\tWarning: duplicated labels for AELINK card: %d.', ...
                    beam_model.Aero.Trim.Link.ID(dof(k)));
                
            end
            
            error('AELINK entries have duplicated labels.');
            
        end
    end
    
    beam_model.Aero.Trim.CS.MPC = [];   % store control surfaces contraint equations
    beam_model.Aero.Trim.CS.Coeff = []; % store control surfaces contraint coefficients
    
    % set output struct
    beam_model.Res = [];
    beam_model.Res.SOL = 'Static linear unrestrained trim';
    beam_model.Res.FM.Value = repmat(beam_model.Aero.Trim.FM.Value,2,1); % current flight mechanics solution
    beam_model.Res.FM.Fixed = beam_model.Aero.Trim.FM.Fixed;
    beam_model.Res.CS.Value = beam_model.Aero.Trim.CS.Value; % current control surfaces solution
    beam_model.Res.CS.Fixed = zeros(1,nc); % set later
    beam_model.Res.state = beam_model.Aero.state; % Tornado dummy state (changed during nonlinear solution search)
    % set constraints for control surfaces
    [beam_model.Aero.Trim.CS.MPC, beam_model.Aero.Trim.CS.Coeff, beam_model.Res.CS.Fixed] = ...
        set_constr_eq(nc, beam_model.Aero.lattice.Control, beam_model.Aero.Trim.CS.Fixed, beam_model.Info.nlink, beam_model.Aero.Trim.Link);
    % count variables
    NC_TOT = sum(beam_model.Res.FM.Fixed) + sum(beam_model.Res.CS.Fixed);
    NTOT = length(beam_model.Res.FM.Fixed) + length(beam_model.Res.CS.Fixed);
    % free variables
    NF_TOT = NTOT - NC_TOT;
    %
    if NF_TOT ~= NDOF
        if NF_TOT<NDOF
            error('Number of degrees of freedom are less than number of DOF %d (%d).', NDOF, NF_TOT);
        else
            %
            fprintf(fid, '\n - Non linear trim solution...');
        end
    end
    %
    ngrid = beam_model.Info.ngrid;
    nbar =  beam_model.Info.nbar;
    nbeam =  beam_model.Info.nbeam;
    ndof =  beam_model.Info.ndof;
    % create intermediate storage struct
    fprintf(fid, '\n - Setting internal database...');
    % store bar internal forces
    beam_model.Res.Bar.CForces = zeros(2, 6, nbar);
    beam_model.Res.Beam.CForces = zeros(2, 6, nbeam);
    % store bar internal strains and curvatures
    beam_model.Res.Bar.CStrains = zeros(2, 6, nbar);
    beam_model.Res.Beam.CStrains = zeros(2, 6, nbeam);
    % store bar stresses
    beam_model.Res.Bar.CStresses = zeros(2, 4, nbar);
    beam_model.Res.Beam.CStresses = zeros(2, 4, nbeam);
    beam_model.Res.Bar.CSM = [];
    beam_model.Res.Beam.CSM = [];
    % store nodal displacement
    beam_model.Res.NDispl = zeros(ngrid, 6);
    NODEPOS = beam_model.Node.Coord;
    % store updated bar rotations
    beam_model.Res.Bar.R = beam_model.Bar.R;
    beam_model.Res.Bar.Colloc = beam_model.Bar.Colloc;
    % store updated beam rotations
    beam_model.Res.Beam.R = beam_model.Beam.R;
    beam_model.Res.Beam.Colloc = beam_model.Beam.Colloc;
    % store updated node rotation
    beam_model.Res.NRd = beam_model.Node.R;
    fprintf(fid, 'done.');
    %
    % run aeroelastic interpolation
    aeroelastic_interface;
    % stiffness matrix
    fprintf(fid, '- Assemblying stiffness matrix...');
    
    [K] = st_lin_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.Node.Coord, beam_model.Bar, beam_model.Beam, beam_model.Celas);
    
    fprintf(fid, 'done.');
    % assembly mass matrix
    fprintf(fid, '\n- Assemblying mass matrix...');
    M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, beam_model.ConM, beam_model.Bar, beam_model.Beam);
    fprintf(fid, 'done.');
    %   RHS
    if ~isempty(beam_model.RBE2.ID)
        K = RBE2Assembly(beam_model.RBE2,K);
        M = RBE2Assembly(beam_model.RBE2,M);
    end
    fprintf(fid,'\n- Setting system rhs...');
    %
    beam_model.Res.Aero = [];
    % set generalized forces
    fprintf(fid, '\n     External forces...');
    F = gf_lin_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F, beam_model.M, beam_model.Node.DOF);
    F_flw = gf_flw_nodal(beam_model.Param.LOAD, beam_model.Info, beam_model.F_FLW, beam_model.Node.R, ...
        beam_model.Node.DOF, LOAD_SCALE);
    fprintf(fid, 'done.');
    %
    fprintf(fid,'\n- Setting SUPORT matrix...');
    %   SUPORT DOF
    if (beam_model.Info.cc_nspc)
        error('SPC case control card detected. This solver allows trim analysis for free-free aircraft.');
    end
    %   Stiffness matrix
    if (~isempty(beam_model.Param.SUPORT))
        dummy = beam_model.Node;
        if ~isempty(beam_model.RBE2.ID)
            dummy.DOF = dummy.DOF2;
        end
        [D, Kll, Klr, Krr, Krl, rdof, ldof, KEPS] = get_suport_shapes(K, dummy, beam_model.Param.SUPORT, beam_model.Param.EPS);
        nr = find(KEPS < beam_model.Param.EPS);
        if (~isempty(nr))
            fprintf(fid, '\nWarning: %d SUPORT rigid modes exceed deformation energy tolerance %g.', length(nr), beam_model.Param.EPS);
        end
    else
        error('No SUPORT card given.');
    end
    %   Provisory
    if ( (size(beam_model.Param.SUPORT,1)>1) || (length(num2str(beam_model.Param.SUPORT(1,2)))~=6))
        error('Provisory: one one suport point with 6 dofs is enabled at the moment.');
    end
    fprintf(fid, 'done.');
    %   Mass matrix
    Mll = M(ldof, ldof); Mlr = M(ldof, rdof); Mrr = M(rdof, rdof); Mrl = M(rdof, ldof);
    %   Rigid body mass matrix for the SUPORTED DOFs
    mr = Mrr + Mrl*D + D'*Mlr + D'*Mll*D;
    MSRR = mr;
    %
    %   Aerodynamic influence matrix
    fprintf(fid,'\nGenerating aerodynamic database...');
    sindex = find(beam_model.Param.SUPORT(1,1) == beam_model.Node.ID);
    beam_model.Aero.geo.ref_point = beam_model.Node.Coord(sindex,:); % set for moments calculations
    beam_model.Aero.geo.CG = beam_model.Node.Coord(sindex,:); % set for angular velocities
    %***********************************************************************************************************************
    %
    % store stability derivatives
    CREF = beam_model.Aero.ref.C_mgc;
    BREF = beam_model.Aero.ref.b_ref;
    SREF = beam_model.Aero.ref.S_ref;
    VREF = 1;% beam_model.Aero.state.AS;
    RHOREF =1;% beam_model.Aero.state.rho;
    QINF = 0.5 * RHOREF * VREF^2;
    QINFS = QINF * SREF;
    LREF = [1.0 1.0 1.0 1/BREF 1/CREF 1/BREF]./QINFS;
    NDIM = diag(LREF);
    beam_model.Res.Aero.RStab_Der = [];
    beam_model.Res.Aero.RIntercept = [];
    beam_model.Res.Aero.DStab_Der = [];
    beam_model.Res.Aero.DIntercept = [];
    beam_model.Res.Aero.RTrim_sol = [];
    beam_model.Res.Aero.DTrim_sol = [];
    %
    %   1 case: static rigid load at NULL reference condition
    fprintf(fid,'\n - Reference rigid case...');
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    %   Extract all VLM structure
    [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(dummy_aero.geo, dummy_aero.lattice, dummy_aero.state);
    [Fxa0, Fya0, Fza0] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    Fa0 = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, Fxa0, Fya0, Fza0);
%     %   Total external forces (Applied loads + Follower + reference rigid aero condition)
    F = F + F_flw + Fa0;
    fprintf(fid,'done.');
    if ~isempty(beam_model.RBE2.ID)
        F = RBE2Assembly2(beam_model.RBE2,F);
    end
    
    %
    %   2 case: rigid body attitude variations
    %   Alpha
    fprintf(fid,'\n - Alpha perturbation...');
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = EPS; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    Fa_ALPHA = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
    
    Fa_State_aero(:,:,1) = [ Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
    
    Fa_ALPHA = Fa_ALPHA ./ EPS; % get force variation
    Fa_ALPHAl = Fa_ALPHA(ldof,1); Fa_ALPHAr = Fa_ALPHA(rdof,1);
    fprintf(fid,'done.');
    %   Beta
    fprintf(fid,'\n - Sideslip perturbation...');
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = EPS;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    
    Fa_State_aero(:,:,2) = [ Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
    
    Fa_BETA = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
    Fa_BETA = Fa_BETA ./ EPS;
    Fa_BETAl = Fa_BETA(ldof,1); Fa_BETAr = Fa_BETA(rdof,1);
    fprintf(fid,'done.');
    %   P
    fprintf(fid,'\n - Angular velocities...');
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = EPS;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    
    Fa_State_aero(:,:,3) = [ Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
    
    Fa_P = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
    %   set to adimensional angular speed
    Fa_P = (2*VREF)*Fa_P ./ (EPS*BREF);
    Fa_Pl = Fa_P(ldof,1); Fa_Pr = Fa_P(rdof,1);
    %   Q
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = EPS;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    
    Fa_State_aero(:,:,4) = [ Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
    
    Fa_Q = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
    %   set to adimensional angular speed
    Fa_Q = (2*VREF)*Fa_Q ./ (EPS*CREF);
    Fa_Ql = Fa_Q(ldof,1); Fa_Qr = Fa_Q(rdof,1);
    %   R
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = EPS;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    [Fxa, Fya, Fza] = paero_force(dummy_aero, dummy_aero.lattice, GAMMA_P, GAMMA_I);
    
    Fa_State_aero(:,:,5) = [ Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
    
    Fa_R = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
        Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
    %   set to adimensional angular speed
    Fa_R = (2*VREF)*Fa_R ./ (EPS*BREF);
    Fa_Rl = Fa_R(ldof,1); Fa_Rr = Fa_R(rdof,1);
    fprintf(fid,'done.');
    %
    %   Control surfaces
    %
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    
    nr = find(beam_model.Aero.Trim.CS.MPC == 0);
    FCl = [];
    FCr = [];
    %
    if (~isempty(nr))
        fprintf(fid,'\n - Controls...');
        FCtot = zeros(ndof, length(nr));
        FCtot_aero = zeros(length(Fxa0),3, length(nr));
        FCl = zeros(length(ldof), length(nr));
        FCr = zeros(length(rdof), length(nr));
        %
        for k=1:length(nr)
            
            % erase all rotations
            beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
            % set master rotation
            beam_model.Res.CS.Value(nr(k)) = EPS;
            % look for slave
            cdof = find(beam_model.Aero.Trim.CS.MPC == nr(k));
            if (~isempty(cdof))
                beam_model.Res.CS.Value(cdof) = EPS*beam_model.Aero.Trim.CS.Coeff(cdof);
            end
            lattice_defo = rotate_control_surf(beam_model.Aero.ref, dummy_aero.state, beam_model.Aero.geo, ...
                beam_model.Aero.lattice, beam_model.Res.CS.Value, ...
                beam_model.Aero.lattice.Control.Hinge, beam_model.Aero.ID, 1);
            
            [Fxa, Fya, Fza] = paero_force(dummy_aero, lattice_defo, GAMMA_P, GAMMA_I);
            
            FCtot_aero(:,:,k) = [Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
            
            Fa_C = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
                Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
            
            Fa_C = Fa_C ./ EPS;
            FCl(:,k) = Fa_C(ldof,1);
            FCr(:,k) = Fa_C(rdof,1);
            FCtot(:,k) = Fa_C;
            %
        end
        beam_model.Res.CS.Value(1:length(beam_model.Aero.Trim.CS.MPC)) = 0.0;
        fprintf(fid,'done.');
    end
    
    %   Assembly derivatives
    Kax  =  -[Fa_ALPHA, Fa_BETA, Fa_P, Fa_Q, Fa_R, FCtot];
    if ~isempty(beam_model.RBE2.ID)
        Kax = RBE2Assembly2(beam_model.RBE2,Kax);
    end
    %     Kax(beam_model.Node.DOF((45:55),2),9) = Kax(beam_model.Node.DOF((45:55),2),9)*0;
    %     Kaxl =  -[Fa_ALPHAl, Fa_BETAl, Fa_Pl, Fa_Ql, Fa_Rl, FCl];
    Kaxl = 2*Kax(ldof,:);
    Kaxr = Kax(rdof,:);
    
    %     Kaxr =  -[Fa_ALPHAr, Fa_BETAr, Fa_Pr, Fa_Qr, Fa_Rr, FCr];
    %***********************************************************************************************************************
    %
    %   Assembly deformability influence coefficients
    %
    fprintf(fid,'\n - Deformability influence coefficients...');
    Qaa = zeros(ndof, ndof);
    dummy_aero = beam_model.Aero;
    dummy_aero.state.alpha = 0; dummy_aero.state.betha = 0;
    dummy_aero.state.P = 0;     dummy_aero.state.Q = 0;      dummy_aero.state.R = 0;
    dummy_aero.state.AS = 1;
    dummy_aero.state.rho = 1;
    %   Get master nodes connected to aerodynamics
    MNODE = [];
    AERO_DOF = [];
    for n = 1:ngrid
        if (~isempty(beam_model.Node.Aero.Index(n).data))
            MNODE = [MNODE, n];
        end
    end
    %
    CPaeroDef = zeros(size(beam_model.Aero.lattice_vlm.COLLOC,1),3,ndof);
    
    for m = 1:length(MNODE)
        n = MNODE(m);
        % DISPLACEMENT DOF
        for k=1:3
            % loop on node displacements DOFs
            if (beam_model.Node.DOF(n,k))
                AERO_DOF = [AERO_DOF, beam_model.Node.DOF(n,k)];
                NODEPOS = beam_model.Node.Coord;
                NODEPOS(n,k) = beam_model.Node.Coord(n,k) + EPS;
                %         determine updated lattice
                lattice_defo = update_vlm_mesh(beam_model.Node, NODEPOS, beam_model.Node.R, dummy_aero);
                [Fxa, Fya, Fza] = paero_force(dummy_aero, lattice_defo, GAMMA_P, GAMMA_I);
                Fa_D = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
                    Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
                
                CPaeroDef(:,:,beam_model.Node.DOF(n, k)) = [Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
                
                Qaa(:,beam_model.Node.DOF(n, k)) = Fa_D ./ EPS;
            end
        end
        %
        for k=4:6
            % erase displacements
            % erase rotations
            beam_model.Res.NRd = beam_model.Node.R;
            % loop on node displacements DOFs
            if (beam_model.Node.DOF(n,k))
                AERO_DOF = [AERO_DOF, beam_model.Node.DOF(n,k)];
                dummyrot = zeros(1,3);
                dummyrot(k-3) = EPS;
                beam_model.Res.NRd(:,:,n) = Rmat(dummyrot);
                %         determine updated lattice
                lattice_defo = update_vlm_mesh(beam_model.Node, beam_model.Node.Coord, beam_model.Res.NRd, dummy_aero);
                [Fxa, Fya, Fza] = paero_force(dummy_aero, lattice_defo, GAMMA_P, GAMMA_I);
                Fa_D = gf_tranfer_aero_nodal(beam_model.Info, beam_model.Node.DOF, beam_model.Node, dummy_aero, ...
                    Fxa-Fxa0, Fya-Fya0, Fza-Fza0);
                
                CPaeroDef(:,:,beam_model.Node.DOF(n, k)) = [Fxa-Fxa0, Fya-Fya0, Fza-Fza0]./ EPS;
                
                Qaa(:,beam_model.Node.DOF(n, k)) = Fa_D ./ EPS;
                
            end
        end
    end
    %
    fprintf(fid,'done.');
    fprintf(fid,'\ndone.');
    
    beam_model.Res.NDispl = zeros(ngrid, 6);
    beam_model.Res.NRd = beam_model.Node.R;
    %
    if ~isempty(beam_model.RBE2.ID)
        Qaa = RBE2Assembly(beam_model.RBE2,Qaa);
    end
    Qaall = Qaa(ldof, ldof);
    Qaalr = Qaa(ldof, rdof);
    Qaarl = Qaa(rdof, ldof);
    Qaarr = Qaa(rdof, rdof);
    %
    VREF =  beam_model.Aero.state.AS;
    RHOREF = beam_model.Aero.state.rho;
    QINF = 0.5 * RHOREF * VREF^2;
    
%     nfollow = size(IND,1);
%     IND = varargin{2};
%     qinverse = zeros(nfollow,1);
%     options = optimset('TolX',1e-8);
%     for i = 1 : nfollow
%         fun = @(q) stab_der_efficiency( q, Kll, Klr, Krl, Krr, D, Mll, Mrl, Mlr, Mrr, MSRR,...
%     Qaall, Qaalr, Qaarl, Qaarr, Kaxl, Kaxr, NDIM,  IND(i,1), IND(i,2));

%         qinverse(i) = fzero(fun,QINF);
         x = linspace(QINF,range(1),range(2));
         eff = zeros(6,size(Kaxl,2),range(2));
         for j = 1 : range(2)
             eff(:,:,j) = stab_der_efficiency( x(j), Kll, Klr, Krl, Krr, D, Mll, Mrl, Mlr, Mrr, MSRR,...
    Qaall, Qaalr, Qaarl, Qaarr, Kaxl, Kaxr, NDIM);%  IND(i,1), IND(i,2));
         end
%          plot(x,eff)
%     end
   
    
    beam_model.Res.Eff = eff;
    fprintf(fid,'\ndone.');
    
    fprintf(fid, '\n\ncompleted.\n\n');
    
else
    
    error('SOL 144 must be given in input file to run linear static analysis.');
    
end

end
%***********************************************************************************************************************
function [Fxa, Fya, Fza] = paero_force(AERO, lattice_defo, GAMMA_P, GAMMA_I)

% set rigid body boundary conditions
RHS = rigid_body_vlm_rhs(AERO.state, AERO.geo, lattice_defo);
np = length(RHS);
% get panel forces
[dwcond, Fxa, Fya, Fza] = get_aero_forces(RHS, AERO.geo, lattice_defo, AERO.state, GAMMA_P, GAMMA_I);

end
%***********************************************************************************************************************
function [dwcond, GAMMA_P, GAMMA_I] = get_VLM_matrix(geo, lattice, state)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;
dwcond = 0;

[np vor_length dim] = size(lattice.VORTEX);

if vor_length ~= 8
    error('Wrong vortex struct dimension.');
end

[dwcond, GAMMA_P, GAMMA_I] = assembly_vlm_mat(lattice, 1, symmxz, symmxy);
[dwcond, GAMMA_P2, GAMMA_I] = assembly_vlm_mat(lattice, 2, symmxz, symmxy);
%
GAMMA_P = inv(GAMMA_P);

end
%***********************************************************************************************************************
function [dwcond, Fx, Fy, Fz] = get_aero_forces(RHS, geo, lattice, state, GAMMA_P, GAMMA_I)

symmxz = state.SIMXZ;
symmxy = state.SIMXY;
PG = 1.0;

if ( (state.pgcorr) && (state.Mach < 1.0))
    pg = sqrt(1-state.Mach^2);
    PGRHO = state.rho./pg;
else
    PGRHO = state.rho;
end

dwcond = 0;

[np vor_length dim] = size(lattice.VORTEX);

Fx = zeros(np,1); Fy = zeros(np,1); Fz = zeros(np,1);

if vor_length ~= 8
    error('Wrong vortex struct dimension.');
end

gamma = GAMMA_P * RHS;
%
VX = zeros(np, 3); VY = zeros(np, 3); VZ = zeros(np, 3); VD = zeros(np, 3);
%
VX = GAMMA_I(:,:,1) * gamma;
VY = GAMMA_I(:,:,2) * gamma;
VZ = GAMMA_I(:,:,3) * gamma;

b1 = vor_length / 2;
VD(:,:) = (lattice.VORTEX(:, b1+1, :) - lattice.VORTEX(:, b1, :));

wind = state.AS.*([cos(state.alpha)*cos(state.betha) sin(state.betha) sin(state.alpha)*cos(state.betha)]);

VFLOW = repmat(wind, np, 1) - [VX VY VZ];

OMEGA = [state.P state.Q state.R];
if norm(OMEGA)
    VBODY = cross((lattice.COLLOC - repmat(geo.CG, np, 1)),...
        repmat(OMEGA, np, 1), 2);
    VFLOW = VFLOW + VBODY;
end

F = zeros(np, 3);
F = PGRHO .* cross(VFLOW, VD, 2);

Fx = F(:,1).*gamma; Fy = F(:,2).*gamma; Fz = F(:,3).*gamma;

end
%***********************************************************************************************************************
function Fa = gf_tranfer_aero_nodal(INFO, DOF, NODE, AERO, Fxa, Fya, Fza)

% generalized aero forces vector on structural master nodes
Fa = zeros(INFO.ndof, 1);
results = [];
% check

try
    AERO.Interp.Ic;
catch
    error('Collocation points interface matrix not available.');
end

try
    AERO.Interp.In;
catch
    error('Panel nodes interface matrix not available.');
end

try
    AERO.Interp.Iv;
catch
    error('Vortex points interface matrix not available.');
end

try
    AERO.Interp.Imv;
catch
    error('Midpoint vortex points interface matrix not available.');
end

ncaero = INFO.ncaero;
ngrid = INFO.ngrid;

if ncaero
    
    % get structural nodal forces
    Fxs = zeros(ngrid, 1);	     Fys = zeros(ngrid, 1);       Fzs = zeros(ngrid, 1);
    Imat = (AERO.Interp.Imv)';
    Fxs = Imat * Fxa;            Fys = Imat * Fya;            Fzs = Imat * Fza;
    
    % master node forces
    Fxm = zeros(ngrid, 1); 	     Fym = zeros(ngrid, 1); 	    Fzm = zeros(ngrid, 1);
    Mxm = zeros(ngrid, 1); 	     Mym = zeros(ngrid, 1); 	    Mzm = zeros(ngrid, 1);
    
    for n = 1:ngrid
        if (NODE.Index(n)) % if master node
            Fxm(n) = Fxs(n);
            Fym(n) = Fys(n);
            Fzm(n) = Fzs(n);
        end
    end
    % get slave forces
    for n = 1:ngrid
        
        if (~isempty(NODE.Aero.Coord(n).data)) % master
            
            % forces on slaves
            fxs = Fxs(NODE.Aero.Index(n).data);
            fys = Fys(NODE.Aero.Index(n).data);
            fzs = Fzs(NODE.Aero.Index(n).data);
            
            Fxm(n) = sum(fxs) + Fxm(n);
            Fym(n) = sum(fys) + Fym(n);
            Fzm(n) = sum(fzs) + Fzm(n);
            
            M = sum( cross(NODE.Aero.Coord(n).data', [fxs, fys, fzs], 2), 1);
            
            Mxm(n) = Mxm(n) + M(1);
            Mym(n) = Mym(n) + M(2);
            Mzm(n) = Mzm(n) + M(3);
        end
        
    end % end node loop
    
    F = zeros(ngrid, 3);
    M = zeros(ngrid, 3);
    
    F = [Fxm, Fym, Fzm];
    M = [Mxm, Mym, Mzm];
    
    %disp(sum(Fxm))
    %disp(sum(Fym))
    %disp(sum(Fzm))
    %disp(sum(Fxa))
    %disp(sum(Fya))
    %disp(sum(Fza))
    
    % store master node forces and moments in the correct DOF position
    
    for n = 1:ngrid
        
        if (NODE.Index(n)) % if master node
            
            index = find(DOF(n, 1:3)); % get free dofs
            pos = DOF(n, index);
            Fa(pos, 1) = F(n, index);  % assembly
            
            index = find(DOF(n, 4:6)); % get free dofs
            if ~isempty(index)
                
                indexoff = index + 3;
                pos = DOF(n, indexoff);
                Fa(pos, 1) = M(n, index);  % assembly
                
            end
            
        end
    end
    
else
    
    error('No aerodynamic mesh available for aeroelastic calculation.');
    
end

end
%***********************************************************************************************************************
function Stab_Der = get_stab_der(AEROMAT)

Stab_Der = [];
Stab_Der.Alpha = [];
Stab_Der.Beta = [];
Stab_Der.P_rate = [];
Stab_Der.Q_rate = [];
Stab_Der.R_rate = [];
Stab_Der.Control = [];
%  Stab_Der.Alpha.dcd_dalpha = AEROMAT(1,1);
Stab_Der.Alpha.dcs_dalpha = AEROMAT(2,1);
Stab_Der.Alpha.dcl_dalpha = AEROMAT(3,1);
%
Stab_Der.Alpha.dcml_dalpha = AEROMAT(4,1);
Stab_Der.Alpha.dcmm_dalpha = AEROMAT(5,1);
Stab_Der.Alpha.dcmn_dalpha = AEROMAT(6,1);
%
% DELTA BETA
%  Stab_Der.Beta.dcd_dbeta = AEROMAT(1,2);
Stab_Der.Beta.dcs_dbeta = AEROMAT(2,2);
Stab_Der.Beta.dcl_dbeta = AEROMAT(3,2);
%
Stab_Der.Beta.dcml_dbeta = AEROMAT(4,2);
Stab_Der.Beta.dcmm_dbeta = AEROMAT(5,2);
Stab_Der.Beta.dcmn_dbeta = AEROMAT(6,2);
%
% DELTA P rate
%  Stab_Der.P_rate.dcd_dP = AEROMAT(1,3);
Stab_Der.P_rate.dcs_dP = AEROMAT(2,3);
Stab_Der.P_rate.dcl_dP = AEROMAT(3,3);
%
Stab_Der.P_rate.dcml_dP = AEROMAT(4,3);
Stab_Der.P_rate.dcmm_dP = AEROMAT(5,3);
Stab_Der.P_rate.dcmn_dP = AEROMAT(6,3);
%
% DELTA Q rate
%  Stab_Der.Q_rate.dcd_dQ = AEROMAT(1,4);
Stab_Der.Q_rate.dcs_dQ = AEROMAT(2,4);
Stab_Der.Q_rate.dcl_dQ = AEROMAT(3,4);
%
Stab_Der.Q_rate.dcml_dQ = AEROMAT(4,4);
Stab_Der.Q_rate.dcmm_dQ = AEROMAT(5,4);
Stab_Der.Q_rate.dcmn_dQ = AEROMAT(6,4);
%
% DELTA R rate
%  Stab_Der.R_rate.dcd_dR = AEROMAT(1,5);
Stab_Der.R_rate.dcs_dR = AEROMAT(2,5);
Stab_Der.R_rate.dcl_dR = AEROMAT(3,5);
%
Stab_Der.R_rate.dcml_dR = AEROMAT(4,5);
Stab_Der.R_rate.dcmm_dR = AEROMAT(5,5);
Stab_Der.R_rate.dcmn_dR = AEROMAT(6,5);
%
% controls
nc = 0;
for n=6:size(AEROMAT,2)
    nc = nc+1;
    Stab_Der.Control.dcs_dDelta(:,nc)  =  AEROMAT(2,n);
    Stab_Der.Control.dcl_dDelta(:,nc)  =  AEROMAT(3,n);
    Stab_Der.Control.dcml_dDelta(:,nc) =  AEROMAT(4,n);
    Stab_Der.Control.dcmm_dDelta(:,nc) =  AEROMAT(5,n);
    Stab_Der.Control.dcmn_dDelta(:,nc) =  AEROMAT(6,n);
end

end
%***********************************************************************************************************************
function INTER = get_aero_intercept(AEROMAT)

INTER = [];
INTER.cs0  = AEROMAT(2,1);
INTER.cl0  = AEROMAT(3,1);
INTER.cml0 = AEROMAT(4,1);
INTER.cmm0 = AEROMAT(5,1);
INTER.cmn0 = AEROMAT(6,1);

end
%***********************************************************************************************************************
function SOL = store_trim_sol(fid, UDD, UX, NAME)


SOL = [];
SOL.Control = [];
SOL.ACC = UDD;
SOL.Alpha = rad2deg(UX(1));
SOL.Betha = rad2deg(UX(2));
SOL.P = UX(3);
SOL.Q = UX(4);
SOL.R = UX(5);
nc = 0;
fprintf(fid,'\n - X acc:      %g [m/s^2].', SOL.ACC(1));
fprintf(fid,'\n - Y acc:      %g [m/s^2].', SOL.ACC(2));
fprintf(fid,'\n - Z acc:      %g [m/s^2].', SOL.ACC(3));
fprintf(fid,'\n - P-DOT:      %g [rad/s^2].', SOL.ACC(4));
fprintf(fid,'\n - Q-DOT:      %g [rad/s^2].', SOL.ACC(5));
fprintf(fid,'\n - R-DOT:      %g [rad/s^2].\n', SOL.ACC(6));

fprintf(fid,'\n - Alpha:      %g [deg].', SOL.Alpha);
fprintf(fid,'\n - Sideslip:   %g [deg].', SOL.Betha);
fprintf(fid,'\n - Roll rate:  %g [-] (p*BREF/(2VREF)).', SOL.P);
fprintf(fid,'\n - Pitch rate: %g [-] (q*CREF/(2VREF)).', SOL.Q);
fprintf(fid,'\n - Yaw rate:   %g [-] (r*BREF/(2VREF)).', SOL.R);
for n=6:size(UX,1)
    nc = nc+1;
    SOL.Control(nc) = rad2deg(UX(n));
    fprintf(fid,'\n - Control %s:   %g [deg].', NAME{nc}, SOL.Control(nc));
end

end
%***********************************************************************************************************************


%**************************************************************************
%**************************************************************************

function f = stab_der_efficiency( q, Kll, Klr, Krl, Krr, D, Mll, Mrl, Mlr, Mrr, MSRR, ...
    Qaall, Qaalr, Qaarl, Qaarr, Kaxl, Kaxr, NDIM)%,  ind1, ind2)

NDIM = NDIM ./ (2*q);
Qaall = 2*q*Qaall;
Qaalr = 2*q*Qaalr;
Qaarl = 2*q*Qaarl;
Qaarr = 2*q*Qaarr;

Kaxl = 2*q*Kaxl;
Kaxr = 2*q*Kaxr;

f = 1;

for i = 1 : 2
    if i == 1
%         fprintf(fid,'\nSolving rigid aircraft trim condition...');
        Kall = Kll;
        Kalr = Klr;
        Karl = Krl;
        Karr = Krr;
        invKall = inv(Kall);
        RFLEX_MAT = invKall;
    else
%         fprintf(fid,'\nSolving deformable aircraft trim condition...');
        Kall = Kll - Qaall;
        Kalr = Klr - Qaalr;
        Karl = Krl - Qaarl;
        Karr = Krr - Qaarr;
        invKall = inv(Kall);
    end
    %
    
    
    AMLR  = invKall * (Mll*D + Mlr);
    ARLR  = invKall * Kalr;
    ALX   = invKall * Kaxl;
%     UINTL = invKall * F(ldof,1);
    %
    DUMMY = D'*Mll + Mrl;
    M2RR =  (D'*Mlr + Mrr) - DUMMY * ARLR;
    M3RR = -DUMMY * AMLR;
    K3LX = -DUMMY * ALX;
%     TMP1 = DUMMY * UINTL;
    %
    invM2RR = inv(M2RR);
    M4RR = invM2RR * M3RR;
    K4LX = invM2RR * K3LX;
%     TMP2 = invM2RR * TMP1;
    %
    DUMMY = D'*Kall + Karl;
    K2RR = -DUMMY * ARLR + (D'*Kalr + Karr);
    KAZL = DUMMY;
    %    KARZX = KAZL - KAXL * ALX;
    KARZX = (D'*Kaxl + Kaxr) - KAZL * ALX;
%     INTZ = D'*F(ldof,1) + F(rdof,1);
%     IPZ = INTZ - DUMMY * UINTL;
    %
    M5RR = -K2RR * M4RR + MSRR;
    MIRR = -KAZL * AMLR + M5RR;
    invMIRR = inv(MIRR);
    KR1ZX = -K2RR * K4LX + KARZX;
%     IPZF = K2RR * TMP2 + IPZ;
    %
%     IPZF1 = invMIRR * IPZF;
%     IPZF2 = MSRR * IPZF1;
    KR2ZX = -invMIRR * KR1ZX;
    Z1ZX = MSRR * KR2ZX;
    
    FMDER = NDIM * Z1ZX;
    f = FMDER./f; 
 
end
end

%***********************************************************************************************************************


%**************************************************************************
%**************************************************************************
%
% function [c,ceq] = handlecon(x,A,B)
%    c = [];
%    ceq = A*x-B;
% end
