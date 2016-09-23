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

%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function       Script called by weight_xml main function
%
%
%   DESCRIPTION:  Execute the balance and inertia prediction routines
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%         
%                
%    REFERENCES:
%
%**************************************************************************


if CALCOGX(DRWSELN) < 0.0001

    CALCOGX(DRWSELN) = CMPCOGX(DRWSELN);  % default cg is max payload at MTOW

end

if CALCOGZ(DRWSELN) < 0.0001

    CALCOGZ(DRWSELN) = CMPCOGZ(DRWSELN);  % default cg is max payload at MTOW

end

fprintf('\n\tInertia estimation [Kgm^2]\n');

% check if the user requests a prediction for inertias
if IXXINER(DRWSELN) < 0.0001 && IYYINER(DRWSELN) < 0.0001 && IZZINER(DRWSELN) < 0.0001 && IXZINER(DRWSELN) < 0.0001

    % Modified scindat function
    [LB,LW,LF,LT,LE1,LE2,IXI,IYI,IZI,IXZI,WB,WW,WF,WTP,WE,WFUEL,WP,LFUEL] ...
        = scindat(DRWSELN, IXXINER, IYYINER, IZZINER, IXZINER, ...
        kdrang, fuselgt, aftslgt, fusevma, forelgt, fusehma,...
        RAMPWEI, WINGWEI, VTALWEI, HTALWEI, PYLNWEI, POWPWEI,...
        NACEWEI, DMFWWEI, PLOTCGS,...
        WSPNMTX, wingspn, REFWLSW, wingplc, REFWARE, REFWTAP, REFWTHB, wingdih,...
        REFWAPX, engeylc, NACELGT, nacemdi,...
        vtalapx, vtalspn, VTAWLSW, vtalver, VTAWARE, VTAWTAP, VTALTHB, VSPNMTX,...
        htallsw, htalapx, htalver, htalspn, HTAWLSW, HTAWARE, HTAWTAP, HSPNMTX, PMFWWEI);

    LCG = REFWYBR(DRWSELN)*wingspn(DRWSELN)/2*tan(kdrang*REFWLSW(DRWSELN)) + ...
        CALCOGX(DRWSELN)/100*REFWMAC(DRWSELN);      % length to longitudinal cg from datum

    HCG = CALCOGZ(DRWSELN)/100*fusevma(DRWSELN);    % length to vertical cg from datum


    %     if CONSELN ~= 39

    WGT = MTOWWEI(DRWSELN); % run inertia computation for MTOW

    %     else
    %
    %         IXXINER(DRWSELN) = 1;
    %         IYYINER(DRWSELN) = 1;
    %         IZZINER(DRWSELN) = 1;
    %         IXZINER(DRWSELN) = 1;
    %         WGT = STACAUW(DRWSELN);   % assume the AUW selected in S&C routine
    %
    %     end

    % access the Mitchell inertia computation routine - can conduct either a
    % parametric (statistical) based method or approach it analytically (this is
    % invoked by arbitrarily setting wing weight to zero)

    if IXXINER(DRWSELN) == -1 || IYYINER(DRWSELN) == -1 || IZZINER(DRWSELN) == -1 || IXZINER(DRWSELN) == -1

        %         [IX, IY, IZ, IXZ] = scinert(LB, LW, LF, LT, LE1, LE2, VTALMOA(DRWSELN), ...
        %             HTALMOA(DRWSELN), WB, WW, WF, WTP, WE, WFUEL, WP, ...
        %             LCG, HCG, IXI, IYI, IZI, IXZI, 0, WGT, LFUEL);
        %

        fprintf('\n\tRefined inertia estimate:\n');

        Imat = refined_inertia_calc(PLOTCGS);

    else

        % Coarse approximation (always referred to CoG)

        fprintf('\n\tCoarse approximation (w.r.t. CoG):\n');

        [IX, IY, IZ, IXZ] = scinert(LB, LW, LF, LT, LE1, LE2, VTALMOA(DRWSELN), ...
            HTALMOA(DRWSELN), WB, 0, WF, WTP, WE, WFUEL, WP, ...
            LCG, HCG, IXI, IYI, IZI, IXZI, 0, WGT, LFUEL);

        Imat = [IX,   0, IXZ;...
            0,  IY,   0;...
            IXZ,   0,  IZ];


    end

    % Store single values
    IXXINER(DRWSELN) = Imat(1, 1);
    IYYINER(DRWSELN) = Imat(2, 2);
    IZZINER(DRWSELN) = Imat(3, 3);
    IXZINER(DRWSELN) = Imat(1, 3);
    IYZINER(DRWSELN) = Imat(2, 3);
    IXYINER(DRWSELN) = Imat(1, 2);


else

    fprintf('\tUsing user defined inertias.\n');

    Imat = [IXXINER,       0, IXZINER;...
        0, IYYINER,       0;...
        IXZINER,       0, IZZINER];

end

fprintf('\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n\tIxy = %12.8g\n\tIyz = %12.8g\n\tIxz = %12.8g\n',...
    IXXINER, IYYINER, IZZINER, IXYINER, IYZINER, IXZINER);
fprintf('\tdone.\n');