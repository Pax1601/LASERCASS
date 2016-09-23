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
%--------------------------------------------------------------------------------------------------
% 2008-04-06
% 
% BULKdataCAERO1.m defines parameters necessary for CAERO1 cards. Each
% panel in wing, vertical and horizontal tail corrisponds to each sector
% defined in the "aircraft.xml" file.
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%   CAERO1      ID        DIH       CP        NY        NX       FOIL1     FOIL2   MESHTYPE
%                X         Y         Z         C       SPAN       TAP       SW        TW1       TW2
%               0/1    FRACORD1   FRACORD2    FNX      NAME
%               DIH       NY       SPAN       TPR      FOIL2      SW       TW2     MESHTYPE    
%               0/1    FRACORD1   FRACORD2    FNX      NAME 
% 
% 
% Called by:    exportCAERO1.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataCAERO1(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE,...
                                  X, Y, Z, C, SPAN, TAP, SW, TW1, TW2,...
                                  FRACORD1, FRACORD2, FNX, NAME, FRCS, PSO)

% Re-define master patch
if (nargin >= 23 & FRCS < 1.0)
    TAP1 = TAP;
    TAP = 1 - (1-TAP) *FRCS;
    SPAN = SPAN *FRCS;
end

%--------------------------------------------------------------------------
% Define field 1: CAERO1
%
fprintf(fid, 'CAERO1  ');

%--------------------------------------------------------------------------
% Define field 2: ID
% 
if isempty(ID)  
    fprintf(fid, '        ');    
else    
    [IDstr] = cnvt2_8chs(ID);
    fprintf(fid, '%c', IDstr);    
end

%--------------------------------------------------------------------------
% Define field 3: DIH
% 
if isempty(DIH)    
    fprintf(fid, '        ');    
else    
    [DIHstr] = cnvt2_8chs(DIH);
    fprintf(fid, '%c', DIHstr);    
end

%--------------------------------------------------------------------------
% Define field 4: CP
% 
if isempty(CP)    
    fprintf(fid, '        ');    
else    
    [CPstr] = cnvt2_8chs(CP);
    fprintf(fid, '%c', CPstr);    
end

%--------------------------------------------------------------------------
% Define field 5: NY
% 
[NYstr] = cnvt2_8chs(NY);
fprintf(fid, '%c', NYstr);

%--------------------------------------------------------------------------
% Define field 6: NX
% 
[NXstr] = cnvt2_8chs(NX);
fprintf(fid, '%c', NXstr);

%--------------------------------------------------------------------------
% Define field 7: FOIL1
% 
[FOIL1str] = cnvt2_8chs(FOIL1);
fprintf(fid, '%c', FOIL1str);

%--------------------------------------------------------------------------
% Define field 8: FOIL2
% 
[FOIL2str] = cnvt2_8chs(FOIL2);
fprintf(fid, '%c', FOIL2str);

%--------------------------------------------------------------------------
% Define field 9: MESHTYPE
% 
[MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
fprintf(fid, '%c', MESHTYPEstr);

%--------------------------------------------------------------------------
% Define field 2, new line: X
% 
fprintf(fid, '\n        ');
[Xstr] = cnvt2_8chs(X);
fprintf(fid, '%c', Xstr);

%--------------------------------------------------------------------------
% Define field 3, new line: Y
% 
[Ystr] = cnvt2_8chs(Y);
fprintf(fid, '%c', Ystr);

%--------------------------------------------------------------------------
% Define field 4, new line: Z
% 
[Zstr] = cnvt2_8chs(Z);
fprintf(fid, '%c', Zstr);

%--------------------------------------------------------------------------
% Define field 5, new line: C
% 
[Cstr] = cnvt2_8chs(C);
fprintf(fid, '%c', Cstr);

%--------------------------------------------------------------------------
% Define field 6, new line: SPAN
% 
[SPANstr] = cnvt2_8chs(SPAN);
fprintf(fid, '%c', SPANstr);

%--------------------------------------------------------------------------
% Define field 7, new line: TAP
% 
[TAPstr] = cnvt2_8chs(TAP);
fprintf(fid, '%c', TAPstr);

%--------------------------------------------------------------------------
% Define field 8, new line: SW
% 
[SWstr] = cnvt2_8chs(SW);
fprintf(fid, '%c', SWstr);

%--------------------------------------------------------------------------
% Define field 9, new line: TW1
% 
[TW1str] = cnvt2_8chs(TW1);
fprintf(fid, '%c', TW1str);

%--------------------------------------------------------------------------
% Define field 10, new line: TW2
% 
[TW2str] = cnvt2_8chs(TW2);
fprintf(fid, '%c', TW2str);

%--------------------------------------------------------------------------
% Control surfaces
% 
if (FRACORD1 > 0 && FRACORD2 > 0)
    
    if ((nargin >= 23 && isequal(FRCS, 1.0)) || (nargin < 23))
        
            %----------------------------------------------------------------------
            % Define field 2, new line: 1 if control surface is defined
            fprintf(fid, '\n');
            fprintf(fid, '        1       ');

            %----------------------------------------------------------------------
            % Define field 3, new line: FRACORD1
            [FRACORD1str] = cnvt2_8chs(FRACORD1);
            fprintf(fid, '%c', FRACORD1str);

            %----------------------------------------------------------------------
            % Define field 4, new line: FRACORD2
            [FRACORD2str] = cnvt2_8chs(FRACORD2);
            fprintf(fid, '%c', FRACORD2str);

            %----------------------------------------------------------------------
            % Define field 5, new line: FNX
            [FNXstr] = cnvt2_8chs(FNX);
            fprintf(fid, '%c', FNXstr);

            %----------------------------------------------------------------------
            % Define field 6, new line: NAME
            [NAMEstr] = cnvt2_8chs(NAME);
            fprintf(fid, NAMEstr);
            
    else
        
        switch PSO
            
            case 0
                
                    %----------------------------------------------------------------------
                    % Define field 2, new line: 1 if control surface is defined
                    fprintf(fid, '\n');
                    fprintf(fid, '        1       ');

                    %----------------------------------------------------------------------
                    % Define field 3, new line: FRACORD1
                    [FRACORD1str] = cnvt2_8chs(FRACORD1);
                    fprintf(fid, '%c', FRACORD1str);

                    %----------------------------------------------------------------------
                    % Define field 4, new line: FRACORD2
                    [FRACORD2str] = cnvt2_8chs(FRACORD2);
                    fprintf(fid, '%c', FRACORD2str);

                    %----------------------------------------------------------------------
                    % Define field 5, new line: FNX
                    [FNXstr] = cnvt2_8chs(FNX);
                    fprintf(fid, '%c', FNXstr);

                    %----------------------------------------------------------------------
                    % Define field 6, new line: NAME
                    [NAMEstr] = cnvt2_8chs(NAME);
                    fprintf(fid, NAMEstr);
                    
                    %----------------------------------------------------------------------
                    % Define field 1, 4th line
                    fprintf(fid, '\n');
                    fprintf(fid, '        ');

                    %----------------------------------------------------------------------
                    % Define field 2, 4th line: DIH
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);

                    %--------------------------------------------------------------------------
                    % Define field 3, 4th line: NY
                    [NYstr] = cnvt2_8chs(NY);
                    fprintf(fid, '%c', NYstr);

                    %--------------------------------------------------------------------------
                    % Define field 4, 4th line: SPAN
                    [SPANstr] = cnvt2_8chs(SPAN/FRCS*(1-FRCS));
                    fprintf(fid, '%c', SPANstr);

                    %--------------------------------------------------------------------------
                    % Define field 5, 4th line: TPR
                    TPR = TAP1 / TAP;
                    [TPRstr] = cnvt2_8chs(TPR);
                    fprintf(fid, '%c', TPRstr);

                    %--------------------------------------------------------------------------
                    % Define field 6, 4th line: FOIL2
                    [FOIL2str] = cnvt2_8chs(FOIL2);
                    fprintf(fid, '%c', FOIL2str);    

                    %--------------------------------------------------------------------------
                    % Define field 7, 4th line: SW
                    [SWstr] = cnvt2_8chs(SW);
                    fprintf(fid, '%c', SWstr);

                    %--------------------------------------------------------------------------
                    % Define field 8, 4th line: TW2
                    [TW2str] = cnvt2_8chs(TW2);
                    fprintf(fid, '%c', TW2str);

                    %--------------------------------------------------------------------------
                    % Define field 9, 4th line: MESHTYPE
                    [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                    fprintf(fid, '%c', MESHTYPEstr);
                
            case 2
                
                %----------------------------------------------------------------------
                % Define field 2, 3th line
                fprintf(fid, '\n');
                fprintf(fid, '        0       ');

                %----------------------------------------------------------------------
                % Define field 1, 4th line
                fprintf(fid, '\n');
                fprintf(fid, '        ');

                %----------------------------------------------------------------------
                % Define field 2, 4th line: DIH
                [DIHstr] = cnvt2_8chs(DIH);
                fprintf(fid, '%c', DIHstr);

                %--------------------------------------------------------------------------
                % Define field 3, 4th line: NY
                [NYstr] = cnvt2_8chs(NY);
                fprintf(fid, '%c', NYstr);

                %--------------------------------------------------------------------------
                % Define field 4, 4th line: SPAN
                [SPANstr] = cnvt2_8chs(SPAN/FRCS*(1-FRCS));
                fprintf(fid, '%c', SPANstr);

                %--------------------------------------------------------------------------
                % Define field 5, 4th line: TPR
                TPR = TAP1 / TAP;
                [TPRstr] = cnvt2_8chs(TPR);
                fprintf(fid, '%c', TPRstr);

                %--------------------------------------------------------------------------
                % Define field 6, 4th line: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL2str);    

                %--------------------------------------------------------------------------
                % Define field 7, 4th line: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);

                %--------------------------------------------------------------------------
                % Define field 8, 4th line: TW2
                [TW2str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW2str);

                %--------------------------------------------------------------------------
                % Define field 9, 4th line: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %----------------------------------------------------------------------
                % Define field 2, new line: 1 if control surface is defined
                fprintf(fid, '\n');
                fprintf(fid, '        1       ');

                %----------------------------------------------------------------------
                % Define field 3, new line: FRACORD1
                [FRACORD1str] = cnvt2_8chs(FRACORD1);
                fprintf(fid, '%c', FRACORD1str);

                %----------------------------------------------------------------------
                % Define field 4, new line: FRACORD2
                [FRACORD2str] = cnvt2_8chs(FRACORD2);
                fprintf(fid, '%c', FRACORD2str);

                %----------------------------------------------------------------------
                % Define field 5, new line: FNX
                [FNXstr] = cnvt2_8chs(FNX);
                fprintf(fid, '%c', FNXstr);

                %----------------------------------------------------------------------
                % Define field 6, new line: NAME
                [NAMEstr] = cnvt2_8chs(NAME);
                fprintf(fid, NAMEstr);
                
        end
        
    end
    
% % % % %     if (nargin >= 23 && FRCS < 1.0)
% % % % %         
% % % % %         %----------------------------------------------------------------------
% % % % %         % Define field 2, 3th line
% % % % %         fprintf(fid, '\n');
% % % % %         fprintf(fid, '        0       ');
% % % % % 
% % % % %         %----------------------------------------------------------------------
% % % % %         % Define field 1, 4th line
% % % % %         fprintf(fid, '\n');
% % % % %         fprintf(fid, '        ');
% % % % % 
% % % % %         %----------------------------------------------------------------------
% % % % %         % Define field 2, 4th line: DIH
% % % % %         [DIHstr] = cnvt2_8chs(DIH);
% % % % %         fprintf(fid, '%c', DIHstr);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 3, 4th line: NY
% % % % %         [NYstr] = cnvt2_8chs(NY);
% % % % %         fprintf(fid, '%c', NYstr);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 4, 4th line: SPAN
% % % % %         [SPANstr] = cnvt2_8chs(SPAN/FRCS*(1-FRCS));
% % % % %         fprintf(fid, '%c', SPANstr);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 5, 4th line: TPR
% % % % %         TPR = TAP1 / TAP;
% % % % %         [TPRstr] = cnvt2_8chs(TPR);
% % % % %         fprintf(fid, '%c', TPRstr);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 6, 4th line: FOIL2
% % % % %         [FOIL2str] = cnvt2_8chs(FOIL2);
% % % % %         fprintf(fid, '%c', FOIL2str);    
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 7, 4th line: SW
% % % % %         [SWstr] = cnvt2_8chs(SW);
% % % % %         fprintf(fid, '%c', SWstr);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 8, 4th line: TW2
% % % % %         [TW2str] = cnvt2_8chs(TW2);
% % % % %         fprintf(fid, '%c', TW2str);
% % % % % 
% % % % %         %--------------------------------------------------------------------------
% % % % %         % Define field 9, 4th line: MESHTYPE
% % % % %         [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
% % % % %         fprintf(fid, '%c', MESHTYPEstr);
% % % % %        
% % % % %     end
% % % % %     
% % % % %     %----------------------------------------------------------------------
% % % % %     % Define field 2, new line: 1 if control surface is defined
% % % % %     fprintf(fid, '\n');
% % % % %     fprintf(fid, '        1       ');
% % % % % 
% % % % %     %----------------------------------------------------------------------
% % % % %     % Define field 3, new line: FRACORD1
% % % % %     [FRACORD1str] = cnvt2_8chs(FRACORD1);
% % % % %     fprintf(fid, '%c', FRACORD1str);
% % % % % 
% % % % %     %----------------------------------------------------------------------
% % % % %     % Define field 4, new line: FRACORD2
% % % % %     [FRACORD2str] = cnvt2_8chs(FRACORD2);
% % % % %     fprintf(fid, '%c', FRACORD2str);
% % % % % 
% % % % %     %----------------------------------------------------------------------
% % % % %     % Define field 5, new line: FNX
% % % % %     [FNXstr] = cnvt2_8chs(FNX);
% % % % %     fprintf(fid, '%c', FNXstr);
% % % % % 
% % % % %     %----------------------------------------------------------------------
% % % % %     % Define field 6, new line: NAME
% % % % %     [NAMEstr] = cnvt2_8chs(NAME);
% % % % %     fprintf(fid, NAMEstr);
        
end

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
