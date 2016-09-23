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
%     DATE        VERS    PROGRAMMER               DESCRIPTION
%     080101      1.0     L.Cavagna - A. Da Ronch  Creation
%
%*******************************************************************************
%
% function AIC = assembly_AIC_matrix(fid, lattice, dlm_model)
%
%   DESCRIPTION: Assembly interference coefficients for DLM
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fid            integer    Output unit
%                lattice        struct     DLM mesh struct
%                dlm_model      struct     DLM struct
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                AIC            complex    Influence matrix for different Mach
%                                          and reduced frequency (4D)
%
%    REFERENCES:
%
%*******************************************************************************

function AIC = assembly_AIC_matrix(fid, lattice, dlm_model)

fprintf(fid, '\n - Assemblying Aerodynamic Influence Coefficients matrix...');
AIC = zeros(lattice.np, lattice.np, length(dlm_model.aero.k), length(dlm_model.aero.M));

ORDER = dlm_model.param.order;
kc = length(dlm_model.aero.k);
% Interference Sets
iset = unique(lattice.INT);
nset = length(iset);
% Set pointer to function
switch (ORDER)
    case 1
        Fhandle = @Quadratic_K_Appr;
    case 2
        Fhandle = @Quartic_K_ApprT;
end


if (dlm_model.param.symm)
    
    if (dlm_model.param.symm > 0)
        
        for m = 1: length(dlm_model.aero.M)
            
            fprintf(fid, '\n\tAssemblying modelled panels for Mach %3f...', dlm_model.aero.M(m));
            
            D1 = zeros(lattice.np, lattice.np);
            D2 = zeros(lattice.np, lattice.np);
            D1s = zeros(lattice.np, lattice.np);
            D2s = zeros(lattice.np, lattice.np);
            
            for ISET=1:nset
                fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
%                 sset = find(lattice.INT(1:lattice.np) == iset(ISET));
%                 msset = length(sset);
                
                sset = find(lattice.INT(1:lattice.np) == iset(ISET));
                msset = length(sset);
                for s = 1 : msset                 % sending box loop
                    
                    DELTA_P = (lattice.COLLOC(sset,:) - meshgrid(lattice.MID_DPOINT(sset(s), :),sset));
                    DELTA_P = (lattice.ref_sys(:,:,sset(s)) * DELTA_P')';
                    [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset, dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                    % Store matrix elements
                    D1(:, sset(s), 1:kc) = reshape(D1rs,msset,1,kc);
                    D2(:, sset(s), 1:kc) = reshape(D2rs,msset,1,kc);
                end
                
                
%                 for s = 1 : msset                 % sending box loop
%                     for r = 1 : msset             % receiving box loop
%                         % distance between collocation and sending point in LOCAL
%                         % coord.
%                         DELTA_P = (lattice.COLLOC(sset(r), :) - lattice.MID_DPOINT(sset(s), :))';
%                         DELTA_P = lattice.ref_sys(:,:,sset(s)) * DELTA_P;
%                         [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset(r), dlm_model.aero.k, lattice, dlm_model, DELTA_P);
%                         % Store matrix elements
%                         D1(sset(r), sset(s), 1:kc) = D1rs;
%                         D2(sset(r), sset(s), 1:kc) = D2rs;
%                     end
%                 end
                fprintf(fid, 'done.');
            end
            
            fprintf(fid, '\n\tdone.');
            fprintf(fid, '\n\tSetting symmetry boundary conditions for Mach %3f...', dlm_model.aero.M(m));
            offset = lattice.np;
            
            
            for ISET=1:nset
                
                fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
                symset = find(lattice.INT(lattice.symm_col) == iset(ISET));
                msymset = length(symset);
                sset = find(lattice.INT(1:lattice.np) == iset(ISET));
                msset = length(sset);
                
                for s = 1 : msymset  % sending box loop
                    
                    DELTA_P = (lattice.COLLOC(sset,:) - meshgrid(lattice.MID_DPOINT(symset(s) + offset, :),sset));
                    DELTA_P = (lattice.ref_sys(:,:,symset(s)+offset) * DELTA_P')';
                    [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), symset(s) + offset, sset, dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                    % Store matrix elements
                    D1s(:, lattice.symm_col(symset(s)), 1:kc) = reshape(D1rs,msset,1,kc);
                    D2s(:, lattice.symm_col(symset(s)), 1:kc) = reshape(D2rs,msset,1,kc);
                    
                end
                
                fprintf(fid, 'done.');
            end
            
            
%             for ISET=1:nset
%                 
%                 fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
%                 symset = find(lattice.INT(lattice.symm_col) == iset(ISET));
%                 msymset = length(symset);
%                 sset = find(lattice.INT(1:lattice.np) == iset(ISET));
%                 msset = length(sset);
%                 
%                 for s = 1 : msymset  % sending box loop
%                     for r = 1 : msset             % receiving box loop
%                         % distance between collocation and sending point in LOCAL
%                         % coord.
%                         coord = lattice.COLLOC(sset(r), :);
%                         DELTA_P = (coord - lattice.MID_DPOINT(symset(s) + offset, :))';
%                         DELTA_P = lattice.ref_sys(:,:,symset(s) + offset) * DELTA_P;
%                         [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), symset(s) + offset, sset(r), dlm_model.aero.k, lattice, dlm_model, DELTA_P);
%                         % Store matrix elements
%                         D1s(sset(r), lattice.symm_col(symset(s)), 1:kc) = D1rs;
%                         D2s(sset(r), lattice.symm_col(symset(s)), 1:kc) = D2rs;
%                     end
%                 end
%                 fprintf(fid, 'done.');
%             end
            
            D1 = D1 + D1s;
            D2 = D2 + D2s;
            fprintf(fid, 'done.');
            
            AIC(:,:,:, m) = D1 + D2;
            
        end % end Mach loop
        
    else % Antisymmetry
        
        for m = 1: length(dlm_model.aero.M)
            
            fprintf(fid, '\n\tAssemblying modelled panels for Mach %3f...', dlm_model.aero.M(m));
            
            D1 = zeros(lattice.np, lattice.np);
            D2 = zeros(lattice.np, lattice.np);
            D1s = zeros(lattice.np, lattice.np);
            D2s = zeros(lattice.np, lattice.np);
            
            for ISET=1:nset
                
                fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
                sset = find(lattice.INT(1:lattice.np) == iset(ISET));
                msset = length(sset);
                for s = 1 : msset                 % sending box loop
                    
                    DELTA_P = (lattice.COLLOC(sset,:) - meshgrid(lattice.MID_DPOINT(sset(s), :),sset));
                    DELTA_P = (lattice.ref_sys(:,:,sset(s)) * DELTA_P')';
                    [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset, dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                    % Store matrix elements
                    D1(:, sset(s), 1:kc) = reshape(D1rs,msset,1,kc);
                    D2(:, sset(s), 1:kc) = reshape(D2rs,msset,1,kc);
                    
                    
%                     for r = 1 : msset             % receiving box loop
%                         % distance between collocation and sending point in LOCAL
%                         % coord.
%                         DELTA_P = (lattice.COLLOC(sset(r), :) - lattice.MID_DPOINT(sset(s), :))';
%                         DELTA_P = lattice.ref_sys(:,:,sset(s)) * DELTA_P;
%                         [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset(r), dlm_model.aero.k, lattice, dlm_model, DELTA_P);
%                         % Store matrix elements
%                         D1(sset(r), sset(s), 1:kc) = D1rs;
%                         D2(sset(r), sset(s), 1:kc) = D2rs;
%                     end
                end
                fprintf(fid, 'done.');
            end
            fprintf(fid, 'done.');
            fprintf(fid, '\n\tSetting antisymmetry boundary conditions for Mach %3f...', dlm_model.aero.M(m));
            offset = lattice.np;
            
            for ISET=1:nset
                
                fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
                symset = find(lattice.INT(lattice.symm_col) == iset(ISET));
                msymset = length(symset);
                sset = find(lattice.INT(1:lattice.np) == iset(ISET));
                msset = length(sset);
                
                for s = 1 : msymset  % sending box loop
                    
                    DELTA_P = (lattice.COLLOC(sset,:) - meshgrid(lattice.MID_DPOINT(symset(s) + offset, :),sset));
                    DELTA_P = (lattice.ref_sys(:,:,symset(s)+offset) * DELTA_P')';
                    [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), symset(s) + offset, sset, dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                    
                    D1s(:, lattice.symm_col(symset(s)), 1:kc) = -reshape(D1rs,msset,1,kc);
                    D2s(:, lattice.symm_col(symset(s)), 1:kc) = -reshape(D2rs,msset,1,kc);
%                     for r = 1 : msset             % receiving box loop
%                         % distance between collocation and sending point in LOCAL
%                         % coord.
%                         coord = lattice.COLLOC(sset(r), :);
%                         DELTA_P = (coord - lattice.MID_DPOINT(symset(s) + offset, :))';
%                         DELTA_P = lattice.ref_sys(:,:,symset(s) + offset) * DELTA_P;
%                         [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), symset(s) + offset, sset(r), dlm_model.aero.k, lattice, dlm_model, DELTA_P);
%                         % Store matrix elements
%                         D1s(sset(r), lattice.symm_col(symset(s)), 1:kc) = -D1rs;
%                         D2s(sset(r), lattice.symm_col(symset(s)), 1:kc) = -D2rs;
%                     end
                end
                fprintf(fid, 'done.');
            end
            
            D1 = D1 + D1s;
            D2 = D2 + D2s;
            
            fprintf(fid, 'done.');
            
            AIC(:,:,:, m) = D1 + D2;
            
        end % end Mach loop
        
    end % end symmetry check
    
else % No symmetry enabled
    
    for m = 1: length(dlm_model.aero.M)
        
        fprintf(fid, '\n\tAssemblying modelled panels for Mach %3f...', dlm_model.aero.M(m));
        
        D1 = zeros(lattice.np, lattice.np);
        D2 = zeros(lattice.np, lattice.np);
        
        for ISET=1:nset
            
            fprintf(fid, '\n\t\tAssemblying interference set %d...', iset(ISET));
            sset = find(lattice.INT(1:lattice.np) == iset(ISET));
            msset = length(sset);
            for s = 1 : msset                 % sending box loop
                %                 tic
                %           for r = 1 : msset             % receiving box loop
                %               % distance between collocation and sending point in LOCAL
                %               % coord.
                %               DELTA_P = (lattice.COLLOC(sset(r), :) - lattice.MID_DPOINT(sset(s), :))';
                %               DELTA_P = lattice.ref_sys(:,:,sset(s)) * DELTA_P;
                %               [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset(r), dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                %               % Store matrix elements
                %               D1(sset(r), sset(s), 1:kc) = D1rs;
                %               D2(sset(r), sset(s), 1:kc) = D2rs;
                
                DELTA_P = (lattice.COLLOC - meshgrid(lattice.MID_DPOINT(sset(s), :),sset));
                DELTA_P = (lattice.ref_sys(:,:,sset(s)) * DELTA_P')';
                [D1rs, D2rs] = Fhandle(dlm_model.aero.M(m), dlm_model.aero.beta(m), sset(s), sset, dlm_model.aero.k, lattice, dlm_model, DELTA_P);
                % Store matrix elements
                D1(:, sset(s), 1:kc) = reshape(D1rs,msset,1,kc);
                D2(:, sset(s), 1:kc) = reshape(D2rs,msset,1,kc);
                
                %           end
                %             toc
            end
            fprintf(fid, 'done.');
        end
        fprintf(fid, 'done.');
        
        AIC(:,:,:, m) = D1 + D2;
        
    end % end Mach loop
end

fprintf(fid, '\n   done.\n');
