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

function [] = BULKdataPBARGFF(fid, PSMID, MID, STHICK, AF, FD, RGEO, NSM, ...
                              BM, BEXP, KMG, KP, CF, CKF, FDENS, FE,...
                              C1, C2, D1, D2, E1, E2, F1, F2)

%--------------------------------------------------------------------------
% Define field 1: PBARGFF
fprintf(fid, 'PBARGFF ');

%--------------------------------------------------------------------------
% Define field 2: PSMID
[PSMIDstr] = cnvt2_8chs(PSMID);
fprintf(fid, '%c', PSMIDstr);

%--------------------------------------------------------------------------
% Define field 3: MID
[MIDstr] = cnvt2_8chs(MID);
fprintf(fid, '%c', MIDstr);

%--------------------------------------------------------------------------
% Define field 4: skin THICK
[THICKstr] = cnvt2_8chs(STHICK);
fprintf(fid, '%c', THICKstr);

%--------------------------------------------------------------------------
% Define field 5: area frame
[AFstr] = cnvt2_8chs(AF);
fprintf(fid, '%c', AFstr);

%--------------------------------------------------------------------------
% Define field 6: frame spacing
[FDstr] = cnvt2_8chs(FD);
fprintf(fid, '%c', FDstr);

%--------------------------------------------------------------------------
% Define field 7: RGEO
[RGEOstr] = cnvt2_8chs(RGEO);
fprintf(fid, '%c', RGEOstr);

%--------------------------------------------------------------------------
% Define field 8: NSM
[NSMstr] = cnvt2_8chs(NSM);
fprintf(fid, '%c', NSMstr);

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
%----------------------------------------------------------------------
% Define field 1: empty
fprintf(fid, '        ');
%--------------------------------------------------------------------------
% Define field 1: BM buckling m factor
[GD] = cnvt2_8chs(BM);
fprintf(fid, '%c', GD);
%--------------------------------------------------------------------------
% Define field 2: BEXP buckling exponent factor
[GD] = cnvt2_8chs(BEXP);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 3: KMG section factor
[GD] = cnvt2_8chs(KMG);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 4: KP section factor
[GD] = cnvt2_8chs(KP);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 5: Shanley's factor
[GD] = cnvt2_8chs(CF);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 6: frame factor
[GD] = cnvt2_8chs(CKF);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 7: frame density
[GD] = cnvt2_8chs(FDENS);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% Define field 8: frame Young modulus
[GD] = cnvt2_8chs(FE);
fprintf(fid, '%c', GD);

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
%----------------------------------------------------------------------
% Define field 1: empty
fprintf(fid, '        ');
%--------------------------------------------------------------------------
% Define field 2: C1
[C1str] = cnvt2_8chs(C1);
fprintf(fid, '%c', C1str);

%--------------------------------------------------------------------------
% Define field 3: C2
[C2str] = cnvt2_8chs(C2);
fprintf(fid, '%c', C2str);

%--------------------------------------------------------------------------
% Define field 4: D1
[D1str] = cnvt2_8chs(D1);
fprintf(fid, '%c', D1str);

%--------------------------------------------------------------------------
% Define field 5: D2
[D2str] = cnvt2_8chs(D2);
fprintf(fid, '%c', D2str);

%--------------------------------------------------------------------------
% Define field 6: E1
[E1str] = cnvt2_8chs(E1);
fprintf(fid, '%c', E1str);

%--------------------------------------------------------------------------
% Define field 7: E2
[E2str] = cnvt2_8chs(E2);
fprintf(fid, '%c', E2str);

%--------------------------------------------------------------------------
% Define field 8: F1
[F1str] = cnvt2_8chs(F1);
fprintf(fid, '%c', F1str);

%--------------------------------------------------------------------------
% Define field 9: F2
[F2str] = cnvt2_8chs(F2);
fprintf(fid, '%c', F2str);

%--------------------------------------------------------------------------
% New line
fprintf(fid, '\n');
