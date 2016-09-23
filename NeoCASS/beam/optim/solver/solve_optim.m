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
function solve_optim()

global beam_model;

fid = beam_model.Param.FID; 
OPTDVAR = beam_model.Param.OPTDVAR;
%
OPTIONS =optimset('Algorithm','interior-point','MaxFunEvals',25000,'TolCon', 0.005, 'LargeScale', 'off', 'Display', 'iter','tolfun',1e-4);


                       

NSTEP = beam_model.Param.NSTEP;
% initialize the model to guess value X0 set by the user-defined function
beam_model = set_approx_model(beam_model);
% 
fprintf('\n\n\n NORMALIZZO MIN MAX!!!!!!!!!!!!!')  
beam_model.Optim.XL = beam_model.Optim.XL./beam_model.Optim.Xnorm;  
beam_model.Optim.XU = beam_model.Optim.XU./beam_model.Optim.Xnorm;   
%
% now the model is updated to guess values, set current solution to the initial one
beam_model.Optim.X = beam_model.Optim.X0;        
%
n_in = beam_model.Optim.Cstr.n_in;
n_eq = beam_model.Optim.Cstr.n_eq;
%
beam_model.Optim.CSTR_IN =[]; 
beam_model.Optim.DCSTR_IN = [];

beam_model.Optim.CSTR_EQ =[];
beam_model.Optim.DCSTR_EQ = [];
TRIM_AERO_EVAL = true;

fprintf('\n\n\ntarocco attivo!!!!!');

SAVE_SOL = [];
SAVE_OBJVAL = [];

SAVE_OBJ = [];
SAVE_CSTR = [];

if (exist('stop_optim'))
    fprintf(fid, '\nFound optimization stop from a previous run. Delete stop_optim file.');
    return
end

index = find(beam_model.Param.FILE == '.');
filename = [beam_model.Param.FILE(1:index(end)-1),'_optim_OBJ.out'];
fp1 = fopen(filename,'w');
filename = [beam_model.Param.FILE(1:index(end)-1),'_optim_VAR.out'];
fp2 = fopen(filename,'w');
filename = [beam_model.Param.FILE(1:index(end)-1),'_optim_CST.out'];
fp3 = fopen(filename,'w'); 

for N = 1: NSTEP% optimization loop
    
    fprintf(fid, '\n Optimization iteration: %d', N);
    
    IN_CSTR_SET = [];
    EQ_CSTR_SET = [];
    OBJ_SET = 0;
    beam_model.Optim.OBJ = 0;
    OUT_IN = [];
    OUT_EQ = [];
    %
    % Run all the available solvers
    %
    % 1) Free mean axes rigid/aeroelastic trim
    %
    if (~isempty(find(beam_model.Param.MSOL == 144,1)))
        [beam_model.Optim.OBJ, beam_model.Optim.DOBJ, ...
            beam_model.Optim.CSTR_IN, beam_model.Optim.DCSTR_IN, ...
            beam_model.Optim.CSTR_EQ, beam_model.Optim.DCSTR_EQ, OBJ_SET, OUT_IN, OUT_EQ] = ...
            Dfree_trim(beam_model.Optim.X, OBJ_SET, IN_CSTR_SET, EQ_CSTR_SET, TRIM_AERO_EVAL);
    end
    %
    %
    % 2) Flutter
    %
    if (~isempty(find(beam_model.Param.MSOL == 150,1)))
           [dum, dum, CSTR_IN, DCSTR_IN, CSTR_EQ, DCSTR_EQ, ... 
                dum, OUT_IN, OUT_EQ]=solve_linfluttOPT(beam_model.Optim.X, OBJ_SET, OUT_IN, OUT_EQ,N);
            beam_model.Optim.CSTR_IN = [beam_model.Optim.CSTR_IN ; CSTR_IN];
            beam_model.Optim.CSTR_EQ = [beam_model.Optim.CSTR_EQ ; CSTR_EQ];
            beam_model.Optim.DCSTR_IN = [beam_model.Optim.DCSTR_IN ; DCSTR_IN];
            beam_model.Optim.DCSTR_EQ = [beam_model.Optim.DCSTR_EQ ; DCSTR_EQ];
            n_in = beam_model.Optim.Cstr.n_in;
            nfcst = 0;
            for j = 1 : beam_model.Info.cc_trim 
                for i = size(beam_model.Param.follow,1)
                    nfcst = nfcst+length(beam_model.Param.follow(i).mode);
                end
            end
            n_in = n_in+nfcst;
    
    elseif (~isempty(find(beam_model.Param.MSOL == 145,1)))
        if (~isempty(find(beam_model.Param.MSOL == 144,1)))
            [dum, dum, CSTR_IN, DCSTR_IN, CSTR_EQ, DCSTR_EQ, ... 
                dum, OUT_IN, OUT_EQ]=solve_linfluttOPT(beam_model.Optim.X, OBJ_SET, OUT_IN, OUT_EQ,N);
            beam_model.Optim.CSTR_IN = [beam_model.Optim.CSTR_IN ; CSTR_IN];
            beam_model.Optim.CSTR_EQ = [beam_model.Optim.CSTR_EQ ; CSTR_EQ];
            beam_model.Optim.DCSTR_IN = [beam_model.Optim.DCSTR_IN ; DCSTR_IN];
            beam_model.Optim.DCSTR_EQ = [beam_model.Optim.DCSTR_EQ ; DCSTR_EQ];
            
        else
            [beam_model.Optim.OBJ, beam_model.Optim.DOBJ, beam_model.Optim.CSTR_IN, beam_model.Optim.DCSTR_IN,...
                beam_model.Optim.CSTR_EQ, beam_model.Optim.DCSTR_EQ, ...
                OBJ_SET, OUT_IN, OUT_EQ]=solve_linfluttOPT(beam_model.Optim.X, OBJ_SET, OUT_IN, OUT_EQ,N);
        end
        n_in = beam_model.Optim.Cstr.n_in;
        nfcst = 0;
        for i = size(beam_model.Param.follow,1)
            nfcst = nfcst+length(beam_model.Param.follow(i).mode);
        end
        n_in = n_in+nfcst;
    end
    %beam_model.Optim.OBJ
    %OBJ_SET
    %beam_model.Optim.DOBJ
    %OUT_IN
    %beam_model.Optim.CSTR_IN
    %beam_model.Optim.DCSTR_IN
    
    %
    % Before running optimization check all constraints and objective were detected
    %
    % ...
    if (length(unique(OUT_IN)) ~= n_in)
        error('Some IN constraints are missing');
    end
    
    if (length(unique(OUT_EQ)) ~= n_eq)
        error('Some EQ constraints are missing');
    end
    
    if (~OBJ_SET)
        error('OBJ non evaluated.');
    end
    %-------------------------------------------------------------------------------------
    % Solve approximate problem and get locally to convergence
    % % locally restrict design variable variation
    fprintf('\n\n\nLimitazione Variazioen massima DESVAR ATTIVA!!!!!');
%       loc_XU = beam_model.Optim.X .* (1.0 + OPTDVAR);
%       loc_XL = beam_model.Optim.X ./ (1.0 + OPTDVAR);
%       iu = find( beam_model.Optim.XU < loc_XU);
%       loc_XU(iu) = beam_model.Optim.XU(iu);
%       il = find( beam_model.Optim.XL > loc_XL);
%       loc_XL(il) = beam_model.Optim.XL(il);
%     fprintf('\n\n\nLimitazione Variazioen massima DESVAR NON ATTIVA!!!!!');
    %   loc_XU = beam_model.Optim.X .* (1.0 + OPTDVAR);
    %   loc_XL = beam_model.Optim.X ./ (1.0 + OPTDVAR);
    %   iu = find( beam_model.Optim.XU < loc_XU);
    loc_XU = beam_model.Optim.X + (beam_model.Optim.XU - beam_model.Optim.XL)*0.015;
    %   il = find( beam_model.Optim.XL > loc_XL);
    loc_XL = beam_model.Optim.X - (beam_model.Optim.XU - beam_model.Optim.XL)*0.015; 
    iu = find( beam_model.Optim.XU < loc_XU);
    loc_XU(iu) = beam_model.Optim.XU(iu); 
    il = find( beam_model.Optim.XL > loc_XL);
    loc_XL(il) = beam_model.Optim.XL(il);    
%     % 
        [SOL, fun, flag] = fmincon(@(X)get_optim_obj(X, beam_model), beam_model.Optim.X, ...
            [],[],[],[], ... 
            loc_XL, loc_XU, @(X)get_optim_cstr(X, beam_model), OPTIONS);
    
%     OPTIONS = gaoptimset('PopulationSize',[40,100,75],'SelectionFcn',@selectionremainder,...
%                      'MutationFcn',@mutationadaptfeasible,...
%                      'CrossoverFcn',{@crossoverintermediate,rand(1,length(loc_XU))},... % alternativa {@crossoverheuristic,1.2}
%                      'MigrationInterval',20,'MigrationFraction',0.1,...
%                      'TolFun',0.0001);% 'InitialPopulation',x,'PopInitRange',[x*0.15;x+(1-x)*0.2],...
    
    
%     [SOL, fun, flag] = ga(@(X)get_optim_obj(X, beam_model), length(beam_model.Optim.X), ...
%         [],[],[],[], ...
%         loc_XL, loc_XU, @(X)get_optim_cstr(X, beam_model), OPTIONS);
      
    fprintf(fid, '\n Optimization flag: %d', flag);    
    beam_model.Optim.X = SOL; % new solution to evaluate 
    
    SAVE_SOL = [SAVE_SOL; SOL];
    SAVE_OBJVAL = [SAVE_OBJVAL; beam_model.Optim.OBJ];
    TRIM_AERO_EVAL = false;
    SAVE_OBJ = [SAVE_OBJ, fun];
    SAVE_CSTR = [SAVE_CSTR, beam_model.Optim.CSTR_IN-beam_model.Optim.Cstr.In.Value'];
    
    % check convergence
    if (N > 1)
        ABSR = fun - SAVE_OBJ(N-1);
        CONV = abs((ABSR)/SAVE_OBJ(N-1));
        fprintf(fid, '\nTolerance on objective function: %e.', CONV);
        if ((CONV < 1e-3))% || ((ABSR<1.0e-6) || (ABSR<0.001*MIOOBJ(1))))
            fprintf(fid, ' Tolerance reached.');
            break;
        end
    end
    
    if (exist('stop_optim'))
        fprintf(fid, '\nOptimization process stopped by the user.');
        break
    end
    
    if N >1 %(beam_model.Param.AUTOPLOT)
        figure(100)
        plot(SAVE_OBJ, '-ko'); title ('Objective function'); xlabel('Iteration number')
        %
        figure(101);
        close(gcf);
        figure(101);
        plot(1:size(SAVE_SOL,2),SAVE_SOL(1:end-1,:),'-','linewidth',0.5);
        hold on
        plot(1:size(SAVE_SOL,2),SAVE_SOL(end,:),'-','linewidth',2)
        title ('Design variables');  xlabel('Iteration number')
        %
        figure(102)
        plot(SAVE_CSTR','-'); title ('Constraint value'); xlabel('Iteration number'); grid on
        ylim([-5,5])
        %
        figure(103);
        plot(SAVE_OBJVAL','-'); title ('Design variables');  xlabel('Iteration number')
    end
    
    fprintf(fp1, '%.6f ', fun);
    fprintf(fp1,'\n');
    fprintf(fp2, '%.6f ', SOL');
    fprintf(fp2,'\n');
    fprintf(fp3, '%.6f ', beam_model.Optim.CSTR_IN');
    fprintf(fp3,'\n');
    
    
end
%
% Export the model in the current status X
%
beam_model = set_approx_model(beam_model, beam_model.Optim.X);
%
beam_model.Res.Optim.Histo = [];
beam_model.Res.Optim.Histo.Sol = SAVE_SOL;
beam_model.Res.Optim.Histo.OBJ = SAVE_OBJ;
beam_model.Res.Optim.Histo.CSTR = SAVE_CSTR;
fclose(fp1);
fclose(fp2);
fclose(fp3);

end
