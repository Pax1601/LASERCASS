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
%  CAERO1    ID        DIH       CP        NY        NX        FOIL1     FOIL2     MESHTYPE
%            X         Y         Z         C         SPAN      TAP       SW        TW1        TW2
%            0/1       FRACORD11 FRACORD12 FNX       NAME1
%            DIH       NY        SPAN      TPR       FOIL2     SW        TW2       MESHTYPE
%            0/1       FRACORD21 FRACORD22 FNX       NAME2
%
%
% Called by:    exportCAERO1.m
%
% Calls:        cnvt2_8chs.m
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travglini 19/11/2009 to improve the mesh on wing with control
% surface
function [IDM, NXM, NYM, IDS, NXS, NYS] = BULKdataCAERO1_mod(fid, ID, DIH, CP, NY, NX, FOIL1, FOIL2, MESHTYPE,...
    X, Y, Z, C, SPAN, TAP, SW, TW1, TW2,...
    FRACORD1, FRACORD2, FNX, NAME, FRCS, PSO, FRACORD3, FOIL3)
% master CAERO data, ID, nx, ny
IDM = [];
NXM = [];
NYM = [];
% added slave CAERO data, ID, nx, ny
IDS = [];
NXS = [];
NYS = [];
% Set parameters having master and slave patch
if (nargin >= 23 && FRCS < 1.0)
    
    TAP1 = TAP;
    SPAN1 = SPAN;
    TW2tip = TW2;
    
    switch PSO
        
        case 0
            % master patch taper
            TAP = 1 - (1-TAP) *FRCS;
            % master patch span
            SPAN = SPAN1 *FRCS;
            % master patch twist at outer section
            TW2 = TW1 + (TW2tip-TW1)*SPAN/SPAN1;
            
        case 2
            % master patch span
            SPAN = SPAN1 *(1-FRCS) /2;
            % master patch taper
            TAP = 1 - (1-TAP) *SPAN/SPAN1;
            TAP2 = 1- (1-TAP1) *(SPAN1-SPAN)/SPAN1;
            % master patch twist at outer section
            TW2 = TW1 + (TW2tip-TW1)*SPAN/SPAN1;
            TW3 = TW1 + (TW2tip-TW1)*(SPAN1-SPAN)/SPAN1;
            
        case 1
            % master patch taper
            TAP = 1 - (1-TAP) *(1-FRCS);
            % master patch span
            SPAN = SPAN1 *(1-FRCS);
            % master patch twist at outer section
            TW2 = TW1 + (TW2tip-TW1)*SPAN/SPAN1;
        otherwise
           % master patch span
            SPAN = SPAN1 *(1-FRCS) /2;
            % master patch taper
            TAP = 1 - (1-TAP) *SPAN/SPAN1;
            TAP2 = 1- (1-TAP1) *(SPAN1-SPAN)/SPAN1;
            % master patch twist at outer section
            TW2 = TW1 + (TW2tip-TW1)*SPAN/SPAN1;
            TW3 = TW1 + (TW2tip-TW1)*(SPAN1-SPAN)/SPAN1;
    end
    
end
%
if (FRACORD1 > 0 && FRACORD2 > 0)
    
    if ((nargin >= 23 && isequal(FRCS, 1.0)) || (nargin < 23))
        %********%
        % Line 1 %
        %********%
        
        %--------------------------------------------------------------------------
        % Line 1, field 1: CAERO1
        fprintf(fid, 'CAERO1  ');
        
        %--------------------------------------------------------------------------
        % Line 1, field 2: ID
        if isempty(ID)
            fprintf(fid, '        ');
        else
            [IDstr] = cnvt2_8chs(ID);
            fprintf(fid, '%c', IDstr);
        end
        IDM = ID;
        %--------------------------------------------------------------------------
        % Line 1, field 3: DIH
        if isempty(DIH)
            fprintf(fid, '        ');
        else
            [DIHstr] = cnvt2_8chs(DIH);
            fprintf(fid, '%c', DIHstr);
        end
        %--------------------------------------------------------------------------
        % Line 1, field 4: CP
        if isempty(CP)
            fprintf(fid, '        ');
        else
            [CPstr] = cnvt2_8chs(CP);
            fprintf(fid, '%c', CPstr);
        end
        
        %--------------------------------------------------------------------------
        % Line 1, field 5: NY
        [NYstr] = cnvt2_8chs(NY);
        fprintf(fid, '%c', NYstr);
        NYM = NY;        
        
        %--------------------------------------------------------------------------
        % Line 1, field 6: NX
        [NXstr] = cnvt2_8chs(NX);
        fprintf(fid, '%c', NXstr);
        NXM = NX;        
        
        %--------------------------------------------------------------------------
        % Line 1, field 7: FOIL1
        [FOIL1str] = cnvt2_8chs(FOIL1);
        fprintf(fid, '%c', FOIL1str);
        
        %--------------------------------------------------------------------------
        % Line 1, field 8: FOIL2
        [FOIL2str] = cnvt2_8chs(FOIL2);
        fprintf(fid, '%c', FOIL2str);
        
        %--------------------------------------------------------------------------
        % Line 1, field 9: MESHTYPE
        [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
        fprintf(fid, '%c', MESHTYPEstr);
        
        %********%
        % Line 2 %
        %********%
        
        %--------------------------------------------------------------------------
        % Line 2, field 1:
        fprintf(fid, '\n        ');
        
        %--------------------------------------------------------------------------
        % Line 2, field 2: X
        [Xstr] = cnvt2_8chs(X);
        fprintf(fid, '%c', Xstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 3: Y
        [Ystr] = cnvt2_8chs(Y);
        fprintf(fid, '%c', Ystr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 4: Z
        [Zstr] = cnvt2_8chs(Z);
        fprintf(fid, '%c', Zstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 5: C
        [Cstr] = cnvt2_8chs(C);
        fprintf(fid, '%c', Cstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 6: SPAN
        [SPANstr] = cnvt2_8chs(SPAN);
        fprintf(fid, '%c', SPANstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 7: TAP
        [TAPstr] = cnvt2_8chs(TAP);
        fprintf(fid, '%c', TAPstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 8: SW
        [SWstr] = cnvt2_8chs(SW);
        fprintf(fid, '%c', SWstr);
        
        %--------------------------------------------------------------------------
        % Line 2, field 9: TW1
        [TW1str] = cnvt2_8chs(TW1);
        fprintf(fid, '%c', TW1str);
        
        %--------------------------------------------------------------------------
        % Line 2, field 10: TW2
        [TW2str] = cnvt2_8chs(TW2);
        fprintf(fid, '%c', TW2str);
        
        %*****************************%
        % Control surfaces definition %
        %*****************************%
        %********%
        % Line 3 %
        %********%
        
        %----------------------------------------------------------------------
        % Line 3, field 1:
        fprintf(fid, '\n        ');
        
        %----------------------------------------------------------------------
        % Line 3, field 2: 1=control surface is defined
        fprintf(fid, '1       ');
        
        %----------------------------------------------------------------------
        % Line 3, field 3: FRACORD1
        [FRACORD1str] = cnvt2_8chs(FRACORD1);
        fprintf(fid, '%c', FRACORD1str);
        
        %----------------------------------------------------------------------
        % Line 3, field 4: FRACORD2
        [FRACORD2str] = cnvt2_8chs(FRACORD2);
        fprintf(fid, '%c', FRACORD2str);
        
        %----------------------------------------------------------------------
        % Line 3, field 5: FNX
        [FNXstr] = cnvt2_8chs(FNX);
        fprintf(fid, '%c', FNXstr);
        NXM = NXM + FNX;        

        %----------------------------------------------------------------------
        % Line 3, field 6: NAME
        [NAMEstr] = cnvt2_8chs(NAME);
        fprintf(fid, NAMEstr);
    else
        
        switch PSO
            
            case 0 % Aileron attached to the kink2 and expanding from there on the outboard
                %********%
                % Line 1 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(ID)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(ID);
                    fprintf(fid, '%c', IDstr);
                    IDM = ID;
                end
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                [NYstr] = cnvt2_8chs(NY);
                fprintf(fid, '%c', NYstr);
                NYM = NY;        

                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                [NXstr] = cnvt2_8chs(NX);
                fprintf(fid, '%c', NXstr);
                NXM = NX;        

                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z);
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW1);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW2str);
                
                %*****************************%
                % Control surfaces definition %
                %*****************************%
                %********%
                % Line 3 %
                %********%
                
                %----------------------------------------------------------------------
                % Line 3, field 1:
                fprintf(fid, '\n        ');
                
                %----------------------------------------------------------------------
                % Line 3, field 2: 1=control surface is defined
                fprintf(fid, '1       ');
                
                %----------------------------------------------------------------------
                % Line 3, field 3: FRACORD1
                [FRACORD1str] = cnvt2_8chs(FRACORD1);
                fprintf(fid, '%c', FRACORD1str);
                
                %----------------------------------------------------------------------
                % Line 3, field 4: FRACORD2
                [FRACORD2str] = cnvt2_8chs(FRACORD2);
                fprintf(fid, '%c', FRACORD2str);
                
                %----------------------------------------------------------------------
                % Line 3, field 5: FNX
                [FNXstr] = cnvt2_8chs(FNX);
                fprintf(fid, '%c', FNXstr);
                NXM = NXM + FNX;        

                %----------------------------------------------------------------------
                % Line 3, field 6: NAME
                [NAMEstr] = cnvt2_8chs(NAME);
                fprintf(fid, NAMEstr);
                
                
                %%%%%%%%%%%%%%%
                % NEW CAERO
                %%%%%%%%%%%%%%%
                IDS = ID+1;
                %********%
                % Line 1 %
                %********%
                fprintf(fid, '\n');
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS);
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS));
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL3);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+(SPAN)*tan(SW*pi/180)+C*(1-TAP)*0.25 );
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+(SPAN));
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+(SPAN)*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN1-SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP1/TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2tip);
                fprintf(fid, '%c', TW2str);
                
                
            case 2  % Aileron centered on the outboard
                
                % NEW CAERO
                %********%
                % Line 1 %
                %********%
                IDS = ID-1;
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS);
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS)*0.5);
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];
                
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];

                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z);
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW1);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW2str);
                
                
                % CONTROL SURFACE
                fprintf(fid, '\n');
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(ID)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(ID);
                    fprintf(fid, '%c', IDstr);
                    IDM = ID;
                end
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                [NYstr] = cnvt2_8chs(NY);
                fprintf(fid, '%c', NYstr);
                NYM = NY;
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                [NXstr] = cnvt2_8chs(NX);
                fprintf(fid, '%c', NXstr);
                NXM = NX;
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+SPAN*tan(SW*pi/180)+C*(1-TAP)*0.25);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+SPAN);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+SPAN*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN1-2*SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP2/TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW3);
                fprintf(fid, '%c', TW2str);
                %********%
                % Line 5 %
                %********%
                
                %----------------------------------------------------------------------
                % Line 5, field 1:
                fprintf(fid, '\n        ');
                
                %----------------------------------------------------------------------
                % Line 5, field 2: 1=control surface is defined
                fprintf(fid, '1       ');
                
                %----------------------------------------------------------------------
                % Line 5, field 3: FRACORD1
                [FRACORD2str] = cnvt2_8chs(FRACORD2);
                fprintf(fid, '%c', FRACORD2str);
                
                %----------------------------------------------------------------------
                % Line 5, field 4: FRACORD2
                [FRACORD3str] = cnvt2_8chs(FRACORD3);
                fprintf(fid, '%c', FRACORD3str);
                
                %----------------------------------------------------------------------
                % Line 5, field 5: FNX
                [FNXstr] = cnvt2_8chs(FNX);
                fprintf(fid, '%c', FNXstr);
                NXM = NXM + FNX;
                %----------------------------------------------------------------------
                % Line 5, field 6: NAME
                [NAMEstr] = cnvt2_8chs(NAME);
                fprintf(fid, '%c', NAMEstr);
                
                
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % NEW CAERO1
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                IDS = [IDS;ID+1];
                fprintf(fid, '\n');
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS(2))
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS(2));
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS)*0.5);
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];

                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];
                
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL3);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+(SPAN1-SPAN)*tan(SW*pi/180)+C*(1 - TAP2)*0.25);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+(SPAN1-SPAN));
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+(SPAN1-SPAN)*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP2);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP1/TAP2);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW3);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2tip);
                fprintf(fid, '%c', TW2str);
                
                
            case 1 % Aileron attached to the tip and expanding on the outboard panel
                % NEW CAERO
                %********%
                % Line 1 %
                %********%
                IDS = ID+1;
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS);
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS));
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];                
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];
                
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z);
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW1);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW2str);
                %--------------------------------------------------------------------------
                fprintf(fid, '\n');
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(ID)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(ID);
                    fprintf(fid, '%c', IDstr);
                    IDM = ID;
                end
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                [NYstr] = cnvt2_8chs(NY);
                fprintf(fid, '%c', NYstr);
                NYM = NY; 
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                [NXstr] = cnvt2_8chs(NX);
                fprintf(fid, '%c', NXstr);
                NXM = NX;
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL3);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+SPAN*tan(SW*pi/180)+C*(1 - TAP)*0.25);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+SPAN);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+SPAN*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN1-SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP1/TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2tip);
                fprintf(fid, '%c', TW2str);
                
                %                 %----------------------------------------------------------------------
                %                 % Line 4, field 1:
                %                 fprintf(fid, '\n        ');
                %
                %                 %----------------------------------------------------------------------
                %                 % Line 4, field 2: DIH
                %                 [DIHstr] = cnvt2_8chs(DIH);
                %                 fprintf(fid, '%c', DIHstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 3: NY
                %                 [NYstr] = cnvt2_8chs(NY);
                %                 fprintf(fid, '%c', NYstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 4: SPAN
                %                 SPANslave = SPAN1 *FRCS;
                %                 [SPANstr] = cnvt2_8chs(SPANslave);
                %                 fprintf(fid, '%c', SPANstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 5: TPR
                %                 TPRslave = TAP1 / TAP;
                %                 [TPRstr] = cnvt2_8chs(TPRslave);
                %                 fprintf(fid, '%c', TPRstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 6: FOIL2
                %                 [FOIL2str] = cnvt2_8chs(FOIL2);
                %                 fprintf(fid, '%c', FOIL2str);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 7: SW
                %                 [SWstr] = cnvt2_8chs(SW);
                %                 fprintf(fid, '%c', SWstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 8: TW2
                %                 TWslave = TW2tip;
                %                 [TWstr] = cnvt2_8chs(TWslave);
                %                 fprintf(fid, '%c', TWstr);
                %
                %                 %--------------------------------------------------------------------------
                %                 % Line 4, field 9: MESHTYPE
                %                 [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                %                 fprintf(fid, '%c', MESHTYPEstr);
                %
                %********%
                % Line 5 %
                %********%
                
                %----------------------------------------------------------------------
                % Line 5, field 1:
                fprintf(fid, '\n        ');
                
                %----------------------------------------------------------------------
                % Line 5, field 2: 1=control surface is defined
                fprintf(fid, '1       ');
                
                %----------------------------------------------------------------------
                % Line 5, field 3: FRACORD1
                [FRACORD2str] = cnvt2_8chs(FRACORD2);
                fprintf(fid, '%c', FRACORD2str);
                
                %----------------------------------------------------------------------
                % Line 5, field 4: FRACORD2
                [FRACORD3str] = cnvt2_8chs(FRACORD3);
                fprintf(fid, '%c', FRACORD3str);
                
                %----------------------------------------------------------------------
                % Line 5, field 5: FNX
                [FNXstr] = cnvt2_8chs(FNX);
                fprintf(fid, '%c', FNXstr);
                NXM = NXM + FNX;
                %----------------------------------------------------------------------
                % Line 5, field 6: NAME
                [NAMEstr] = cnvt2_8chs(NAME);
                fprintf(fid, '%c', NAMEstr);
            otherwise % Aileron centered on the outboard
                
                % NEW CAERO
                %********%
                % Line 1 %
                %********%
                IDS = ID-1;
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS);
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS)*0.5);
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z);
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW1);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW2str);
                
                
                % CONTROL SURFACE
                fprintf(fid, '\n');
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(ID)
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(ID);
                    fprintf(fid, '%c', IDstr);
                    IDM = ID;
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                [NYstr] = cnvt2_8chs(NY);
                fprintf(fid, '%c', NYstr);
                NYM = NY;
                
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                [NXstr] = cnvt2_8chs(NX);
                fprintf(fid, '%c', NXstr);
                NXM = NX;
                
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL1);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+SPAN*tan(SW*pi/180)+C*(1-TAP)*0.25);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+SPAN);
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+SPAN*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN1-2*SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP2/TAP);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW2);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW3);
                fprintf(fid, '%c', TW2str);
                %********%
                % Line 5 %
                %********%
                
                %----------------------------------------------------------------------
                % Line 5, field 1:
                fprintf(fid, '\n        ');
                
                %----------------------------------------------------------------------
                % Line 5, field 2: 1=control surface is defined
                fprintf(fid, '1       ');
                
                %----------------------------------------------------------------------
                % Line 5, field 3: FRACORD1
                [FRACORD2str] = cnvt2_8chs(FRACORD2);
                fprintf(fid, '%c', FRACORD2str);
                
                %----------------------------------------------------------------------
                % Line 5, field 4: FRACORD2
                [FRACORD3str] = cnvt2_8chs(FRACORD3);
                fprintf(fid, '%c', FRACORD3str);
                
                %----------------------------------------------------------------------
                % Line 5, field 5: FNX
                [FNXstr] = cnvt2_8chs(FNX);
                fprintf(fid, '%c', FNXstr);
                NXM = NXM + FNX;
                %----------------------------------------------------------------------
                % Line 5, field 6: NAME
                [NAMEstr] = cnvt2_8chs(NAME);
                fprintf(fid, '%c', NAMEstr);
                
                
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % NEW CAERO1
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                IDS = [IDS;ID+1];
                fprintf(fid, '\n');
                %--------------------------------------------------------------------------
                % Line 1, field 1: CAERO1
                fprintf(fid, 'CAERO1  ');
                
                %--------------------------------------------------------------------------
                % Line 1, field 2: ID
                if isempty(IDS(2))
                    fprintf(fid, '        ');
                else
                    [IDstr] = cnvt2_8chs(IDS(2));
                    fprintf(fid, '%c', IDstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 3: DIH
                if isempty(DIH)
                    fprintf(fid, '        ');
                else
                    [DIHstr] = cnvt2_8chs(DIH);
                    fprintf(fid, '%c', DIHstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 4: CP
                if isempty(CP)
                    fprintf(fid, '        ');
                else
                    [CPstr] = cnvt2_8chs(CP);
                    fprintf(fid, '%c', CPstr);
                end
                
                %--------------------------------------------------------------------------
                % Line 1, field 5: NY
                nyp = ceil(NY*(1-FRCS)*0.5);
                [NYstr] = cnvt2_8chs(nyp);
                fprintf(fid, '%c', NYstr);
                NYS = [NYS; nyp];
                
                %--------------------------------------------------------------------------
                % Line 1, field 6: NX
                nxp = NX+FNX;
                [NXstr] = cnvt2_8chs(nxp);
                fprintf(fid, '%c', NXstr);
                NXS = [NXS; nxp];
                
                %--------------------------------------------------------------------------
                % Line 1, field 7: FOIL1
                [FOIL1str] = cnvt2_8chs(FOIL2);
                fprintf(fid, '%c', FOIL1str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 8: FOIL2
                [FOIL2str] = cnvt2_8chs(FOIL3);
                fprintf(fid, '%c', FOIL2str);
                
                %--------------------------------------------------------------------------
                % Line 1, field 9: MESHTYPE
                [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
                fprintf(fid, '%c', MESHTYPEstr);
                
                %********%
                % Line 2 %
                %********%
                
                %--------------------------------------------------------------------------
                % Line 2, field 1:
                fprintf(fid, '\n        ');
                
                %--------------------------------------------------------------------------
                % Line 2, field 2: X
                [Xstr] = cnvt2_8chs(X+(SPAN1-SPAN)*tan(SW*pi/180)+C*(1 - TAP2)*0.25);
                fprintf(fid, '%c', Xstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 3: Y
                [Ystr] = cnvt2_8chs(Y+(SPAN1-SPAN));
                fprintf(fid, '%c', Ystr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 4: Z
                [Zstr] = cnvt2_8chs(Z+(SPAN1-SPAN)*tan(DIH*pi/180));
                fprintf(fid, '%c', Zstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 5: C
                [Cstr] = cnvt2_8chs(C*TAP2);
                fprintf(fid, '%c', Cstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 6: SPAN
                [SPANstr] = cnvt2_8chs(SPAN);
                fprintf(fid, '%c', SPANstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 7: TAP
                [TAPstr] = cnvt2_8chs(TAP1/TAP2);
                fprintf(fid, '%c', TAPstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 8: SW
                [SWstr] = cnvt2_8chs(SW);
                fprintf(fid, '%c', SWstr);
                
                %--------------------------------------------------------------------------
                % Line 2, field 9: TW1
                [TW1str] = cnvt2_8chs(TW3);
                fprintf(fid, '%c', TW1str);
                
                %--------------------------------------------------------------------------
                % Line 2, field 10: TW2
                [TW2str] = cnvt2_8chs(TW2tip);
                fprintf(fid, '%c', TW2str);
        end
        
    end
    
else
    %********%
    % Line 1 %
    %********%
    
    %--------------------------------------------------------------------------
    % Line 1, field 1: CAERO1
    fprintf(fid, 'CAERO1  ');
    
    %--------------------------------------------------------------------------
    % Line 1, field 2: ID
    if isempty(ID)
        fprintf(fid, '        ');
    else
        [IDstr] = cnvt2_8chs(ID);
        fprintf(fid, '%c', IDstr);
        IDM = ID;
    end
    %--------------------------------------------------------------------------
    % Line 1, field 3: DIH
    if isempty(DIH)
        fprintf(fid, '        ');
    else
        [DIHstr] = cnvt2_8chs(DIH);
        fprintf(fid, '%c', DIHstr);
    end
    
    %--------------------------------------------------------------------------
    % Line 1, field 4: CP
    if isempty(CP)
        fprintf(fid, '        ');
    else
        [CPstr] = cnvt2_8chs(CP);
        fprintf(fid, '%c', CPstr);
    end
    
    %--------------------------------------------------------------------------
    % Line 1, field 5: NY
    [NYstr] = cnvt2_8chs(NY);
    fprintf(fid, '%c', NYstr);
    NYM = NY;        

    %--------------------------------------------------------------------------
    % Line 1, field 6: NX
    [NXstr] = cnvt2_8chs(NX);
    fprintf(fid, '%c', NXstr);
    NXM = NX;        
    
    %--------------------------------------------------------------------------
    % Line 1, field 7: FOIL1
    [FOIL1str] = cnvt2_8chs(FOIL1);
    fprintf(fid, '%c', FOIL1str);
    
    %--------------------------------------------------------------------------
    % Line 1, field 8: FOIL2
    [FOIL2str] = cnvt2_8chs(FOIL2);
    fprintf(fid, '%c', FOIL2str);
    
    %--------------------------------------------------------------------------
    % Line 1, field 9: MESHTYPE
    [MESHTYPEstr] = cnvt2_8chs(MESHTYPE);
    fprintf(fid, '%c', MESHTYPEstr);
    
    %********%
    % Line 2 %
    %********%
    
    %--------------------------------------------------------------------------
    % Line 2, field 1:
    fprintf(fid, '\n        ');
    
    %--------------------------------------------------------------------------
    % Line 2, field 2: X
    [Xstr] = cnvt2_8chs(X);
    fprintf(fid, '%c', Xstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 3: Y
    [Ystr] = cnvt2_8chs(Y);
    fprintf(fid, '%c', Ystr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 4: Z
    [Zstr] = cnvt2_8chs(Z);
    fprintf(fid, '%c', Zstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 5: C
    [Cstr] = cnvt2_8chs(C);
    fprintf(fid, '%c', Cstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 6: SPAN
    [SPANstr] = cnvt2_8chs(SPAN);
    fprintf(fid, '%c', SPANstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 7: TAP
    [TAPstr] = cnvt2_8chs(TAP);
    fprintf(fid, '%c', TAPstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 8: SW
    [SWstr] = cnvt2_8chs(SW);
    fprintf(fid, '%c', SWstr);
    
    %--------------------------------------------------------------------------
    % Line 2, field 9: TW1
    [TW1str] = cnvt2_8chs(TW1);
    fprintf(fid, '%c', TW1str);
    
    %--------------------------------------------------------------------------
    % Line 2, field 10: TW2
    [TW2str] = cnvt2_8chs(TW2);
    fprintf(fid, '%c', TW2str);
    
    %*****************************%
    % Control surfaces definition %
    %*****************************%
    IDS = [];
end

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
