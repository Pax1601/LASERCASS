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
%     080308      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
%
function [OBJ_REF, DOBJ, CSTR_IN, DCSTR_IN, CSTR_EQ, DCSTR_EQ, ...
          OBJ_SET, OUT_IN, OUT_EQ] = Dfree_trim(X, OBJ_SET, IN_CSTR_SET, EQ_CSTR_SET, AERO_EVAL)
%
global beam_model; % actual initial approximate model
fid = beam_model.Param.FID;
if ~isfield(beam_model.Optim,'Res')
    beam_model.Optim.Res.Free_trim = {};
end

orig_beam_model = beam_model;

ndes = length(X);
n_in = beam_model.Optim.Cstr.n_in;
n_eq = beam_model.Optim.Cstr.n_eq;
% constraints still to be set
MISS_IN = setdiff([1:n_in], IN_CSTR_SET);
MISS_EQ = setdiff([1:n_eq], EQ_CSTR_SET);

OUT_IN = []; % nothing is set yet
OUT_EQ = [];

OBJ_REF = -1;
% constraints value
CSTR_IN = zeros(n_in, 1);
CSTR_EQ = zeros(n_eq, 1);
% OBJ and contraints gradients
DOBJ = zeros(1, ndes);
DCSTR_IN = zeros(n_in, ndes);
DCSTR_EQ = zeros(n_eq, ndes);
%
if (~isempty(find(beam_model.Param.MSOL == 144)))

  % update model to current solution
  upd_beam_model = set_approx_model(beam_model, X);

  upd_beam_model.Optim.Res.Free_trim = {};
  upd_beam_model.Optim.Res.Free_trim = beam_model.Optim.Res.Free_trim;
  beam_model = upd_beam_model; % restart from updated model, put it in the global variable then overwritten again
  beam_model.Optim.Res.Free_trim = {};
  % 
  % Solve free aeroelastic trim problem
  %
  for N = 1:upd_beam_model.Info.cc_trim  
  %

    
    % Determine reference trim condition
    if (AERO_EVAL)
      solve_free_lin_trim(N);
      beam_model.Optim.Res.Free_trim{N} = beam_model.Res;
      beam_model.Optim.Res.Free_trim{N}.lattice_defo = beam_model.Aero.lattice_defo;
      Fa0 = beam_model.Res.Aero.Fa0;
      Kax =  beam_model.Res.Aero.Kax;
      Qaa = beam_model.Res.Aero.Qaa;
      command = ['save amat_trim',num2str(N),'.mat Fa0 Kax Qaa;'];
      eval(command);
      DUMMY_RES = beam_model.Res;
      DUMMY_RES.lattice_defo = beam_model.Aero.lattice_defo;
      orig_beam_model.Aero.Interp.Ic = beam_model.Aero.Interp.Ic;
      orig_beam_model.Aero.Interp.In = beam_model.Aero.Interp.In;
      orig_beam_model.Aero.Interp.Iv = beam_model.Aero.Interp.Iv;
      orig_beam_model.Aero.Interp.Imv = beam_model.Aero.Interp.Imv;
      upd_beam_model.Aero.Interp.Ic = beam_model.Aero.Interp.Ic;
      upd_beam_model.Aero.Interp.In = beam_model.Aero.Interp.In;
      upd_beam_model.Aero.Interp.Iv = beam_model.Aero.Interp.Iv;
      upd_beam_model.Aero.Interp.Imv = beam_model.Aero.Interp.Imv;
      
    else
      command = ['load amat_trim', num2str(N),'.mat Fa0 Kax Qaa;'];
      eval(command);
	    K = st_lin_matrixT(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, ...
                        beam_model.Node.Coord, beam_model.Bar, beam_model.Beam, beam_model.Celas);
      M = ms_matrix(beam_model.Info, beam_model.Node.DOF, beam_model.Node.R, ...
                    beam_model.ConM, beam_model.Bar, beam_model.Beam);
      if ~isempty(beam_model.RBE2)
          K = RBE2Assembly(beam_model.RBE2,K);
          M = RBE2Assembly(beam_model.RBE2,M);
      end
      
      RES = restart_solve_free_lin_trim(beam_model, N, M, K, Fa0, Kax, Qaa);
      DUMMY_RES = RES;
      DUMMY_RES.lattice_defo = RES.lattice_defo;
      DUMMY_RES.Gamma = upd_beam_model.Optim.Res.Free_trim{N}.Gamma;
      beam_model.Optim.Res.Free_trim{N} = DUMMY_RES;
    end 
    %
    [CSTR_IN, CSTR_EQ, SET_IN, SET_EQ] = get_cstr_value(beam_model, CSTR_IN, CSTR_EQ, MISS_IN, MISS_EQ);

    % newly set constraints 
    OUT_IN = [OUT_IN , SET_IN];
    OUT_EQ = [OUT_EQ , SET_EQ];
    % update missing constraints
    MISS_IN = setdiff(MISS_IN, SET_IN);
    MISS_EQ = setdiff(MISS_EQ, SET_EQ);
%
    if (~OBJ_SET) % if OBJ is not set, try to set it
      [OBJ_REF, OBJ_SET] = beam_model.Param.OBJ(beam_model);
    end
    %
    %      
    % Evaluate parturbations
    %
    fprintf(fid, '\n - Evaluating perturbations...');
    if (~isempty(find(beam_model.Param.MSOL == 145,1)))
        dummy = beam_model;
        beam_model.Param.SUPORT = [];
        solve_eig;
        dummy.Res = beam_model.Res;
        beam_model = dummy;
        Kr = beam_model.Res.Kmm;
        Mr = beam_model.Res.Mmm;
        
        % we have to check order of eigenvalues (and eigenvectors)
        % and if needed, change the vector INDfollow to ensure to look at
        % chosen eigenvalues.
        if ~(isfield(orig_beam_model,'Optim') && isfield(orig_beam_model.Optim,'Res') && isfield(orig_beam_model.Optim.Res,'Eig'))
            orig_beam_model.Optim.Res.Eig = beam_model.Res;
        end
        
        
        transform = Check_eigenvector(orig_beam_model.Optim.Res.Eig.V,beam_model.Res.V, beam_model.Res.M,beam_model.Param.MSELECT);
        
        % update eigenvector base
%         orig_beam_model.Optim.Res.Eig = beam_model.Res;
        % update follow mode
        
        Fmode = beam_model.Param.follow;
        Fmode2 = Fmode;
        for i = 1 : length(Fmode) % loop on Mach number
            Fmode2(i).mode = transform(2,Fmode(i).mode);
        end
        beam_model.Param.follow = Fmode2;
        orig_beam_model.Param.follow = Fmode2;
        
        orig_beam_model.Optim.Res.Eig = beam_model.Res;
        orig_beam_model.Optim.Res.Eig.Kp = zeros([beam_model.Param.NROOTS,beam_model.Param.NROOTS,length(X)]);
        orig_beam_model.Optim.Res.Eig.Mp = orig_beam_model.Optim.Res.Eig.Kp;
    end
    
    for I = 1:length(X);
%
      fprintf(fid, '\n Perturbation n. %d.\n', I);
      XC = X;
      % perturb desvar I
      XC(I) = XC(I) * (1 + beam_model.Param.DIRDER);
      dummy = set_approx_model(upd_beam_model, XC);

	    K = st_lin_matrixT(dummy.Info, dummy.Node.DOF, dummy.Node.R, ...
                        dummy.Node.Coord, dummy.Bar, dummy.Beam, dummy.Celas);
      M = ms_matrix(dummy.Info, dummy.Node.DOF, dummy.Node.R, ...
                    dummy.ConM, dummy.Bar, dummy.Beam);
      if ~isempty(beam_model.RBE2)
          K = RBE2Assembly(beam_model.RBE2,K);
          M = RBE2Assembly(beam_model.RBE2,M);
      end
                
      if (~isempty(find(beam_model.Param.MSOL == 145,1)))
          warning off;
          %     force matrix to be simmetrix due to tolerances for eigs solver on Windows system
          M2 = (M + M') ./ 2.0;
          K2 = K;
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
          
          orig_beam_model.Optim.Res.Eig.Kp(:,:,I) = (km-Kr)/(XC(I)*beam_model.Param.DIRDER*beam_model.Optim.Xnorm(I));
          orig_beam_model.Optim.Res.Eig.Mp(:,:,I) = (mm-Mr)/(XC(I)*beam_model.Param.DIRDER*beam_model.Optim.Xnorm(I));
          
      end
      if (~isempty(SET_IN)) || (~isempty(SET_EQ))       
          RES = restart_solve_free_lin_trim(dummy, N, M, K, Fa0, Kax, Qaa); 
          beam_model.Optim.Res.Free_trim{N} = RES;
          %     evaluate perturbed constraints (if any)
          [IN_C, EQ_C] = get_dcstr_value(beam_model, SET_IN, SET_EQ);
          if (~isempty(SET_IN)) 
              DCSTR_IN(SET_IN,I) = (IN_C(SET_IN) - CSTR_IN(SET_IN)) ./ (beam_model.Param.DIRDER);
          end
          %
          if (~isempty(SET_EQ))
              DCSTR_EQ(SET_EQ,I) = (EQ_C(SET_EQ) - CSTR_EQ(SET_EQ)) ./ (beam_model.Param.DIRDER);
          end
      end


      
%
      if (OBJ_REF~=-1)
        [OBJ, OBJ_SET] = beam_model.Param.OBJ(dummy);
        DOBJ(I) = (OBJ-OBJ_REF) / (beam_model.Param.DIRDER);
      end      
    end % param loop
    fprintf(fid, 'done.');

    beam_model.Optim.Res.Free_trim{N} = DUMMY_RES;

  end % trim loop
% DA SISTEMARE
  upd_beam_model.Optim.Res.Free_trim = beam_model.Optim.Res.Free_trim;
  orig_beam_model.Optim.Res.Free_trim = beam_model.Optim.Res.Free_trim;
%  beam_model = upd_beam_model;
  % set to previous value
  if (OBJ_SET == -1)
    OBJ_REF = beam_model.Optim.OBJ;
  end

  beam_model = orig_beam_model;

end

end
