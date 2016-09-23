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
% PBARSM3 to export cross-sectional details for:
%   Unflanged, integrally stiffened wide column (struct concept = 1)
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%  PBARSM3   PSMID     MID       RGEO      NSM  
%            TS        TW        BS        BW        NFR        
%            C1        C2        D1        D2        E1        E2        F1         F2 
% 
% Called by:    
% 
% Calls:        cnvt2_8chs.m
% 
% CREATED 2008-10-13
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataPBARSM3(fid, PSMID, MID, RGEO, NSM, TS, TW, BS, BW, NFR, C1, C2, D1, D2, E1, E2, F1, F2)

%--------------------------------------------------------------------------
% Line 1, field 1: PBARSM3
fprintf(fid, 'PBARSM3 ');

%--------------------------------------------------------------------------
% Line 1, field 2: PSMID
[PSMIDstr] = cnvt2_8chs(PSMID);
fprintf(fid, '%c', PSMIDstr);

%--------------------------------------------------------------------------
% Line 1, field 3: MID
[MIDstr] = cnvt2_8chs(MID);
fprintf(fid, '%c', MIDstr);

%--------------------------------------------------------------------------
% Line 1, field 4: RGEO
[RGEOstr] = cnvt2_8chs(RGEO);
fprintf(fid, '%c', RGEOstr);

%--------------------------------------------------------------------------
% Line 1, field 5: NSM
[NSMstr] = cnvt2_8chs(NSM);
fprintf(fid, '%c', NSMstr);

%--------------------------------------------------------------------------
% Line 2, field 1:
fprintf(fid, '\n        ');

%--------------------------------------------------------------------------
% Line 2, field 2: TS
[TSstr] = cnvt2_8chs(TS);
fprintf(fid, '%c', TSstr);

%--------------------------------------------------------------------------
% Line 2, field 3: TW
[TWstr] = cnvt2_8chs(TW);
fprintf(fid, '%c', TWstr);

%--------------------------------------------------------------------------
% Line 2, field 4: BS
[BSstr] = cnvt2_8chs(BS);
fprintf(fid, '%c', BSstr);

%--------------------------------------------------------------------------
% Line 2, field 5: BW
[BWstr] = cnvt2_8chs(BW);
fprintf(fid, '%c', BWstr);

%--------------------------------------------------------------------------
% Line 2, field 6: NFR
[NFRstr] = cnvt2_8chs(NFR);
fprintf(fid, '%c', NFRstr);

%--------------------------------------------------------------------------
% Line 3, field 1:
fprintf(fid, '\n        ');

%--------------------------------------------------------------------------
% Line 3, field 2: C1
[C1str] = cnvt2_8chs(C1);
fprintf(fid, '%c', C1str);

%--------------------------------------------------------------------------
% Line 3, field 3: C2
[C2str] = cnvt2_8chs(C2);
fprintf(fid, '%c', C2str);

%--------------------------------------------------------------------------
% Line 3, field 4: D1
[D1str] = cnvt2_8chs(D1);
fprintf(fid, '%c', D1str);

%--------------------------------------------------------------------------
% Line 3, field 5: D2
[D2str] = cnvt2_8chs(D2);
fprintf(fid, '%c', D2str);

%--------------------------------------------------------------------------
% Line 3, field 6: E1
[E1str] = cnvt2_8chs(E1);
fprintf(fid, '%c', E1str);

%--------------------------------------------------------------------------
% Line 3, field 7: E2
[E2str] = cnvt2_8chs(E2);
fprintf(fid, '%c', E2str);

%--------------------------------------------------------------------------
% Line 3, field 8: F1
[F1str] = cnvt2_8chs(F1);
fprintf(fid, '%c', F1str);

%--------------------------------------------------------------------------
% Line 3, field 9: F2
[F2str] = cnvt2_8chs(F2);
fprintf(fid, '%c', F2str);

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
