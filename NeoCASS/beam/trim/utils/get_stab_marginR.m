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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS      PROGRAMMER      DESCRIPTION
%     121017      2.1.472   L.Cavagna       Creation
%
%*******************************************************************************
%
% function   [RSTAB] = get_stab_marginR(ref_point, CG, CREF, SREF, FUSD, FUSL, RSTABDER, fid)
%
%   DESCRIPTION: Recover longitudinal static stability margin for
%                rigid solver
%
%         INPUT: NAME           TYPE       DESCRIPTION

%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION

%    REFERENCES:
%
%*******************************************************************************
function [RSTAB] = get_stab_marginR(ref_point, CG, CREF, SREF, FUSD, FUSL, RSTABDER, fid)
%
[NEUXR, MNEUXR, SMARGR, SMARGIR] = get_stab_margin_case(ref_point, CG, CREF, SREF, FUSD, FUSL, RSTABDER);
%
fprintf(fid, '\n\nSTABILITY\n');
fprintf(fid,'\n NAME                                     RIGID');
fprintf(fid,'\n Center of gravity            XG/CREF    '); r=sprintf('%-20.5f', CG(1)/CREF); fprintf(fid,'%s',r);                                                                       
%
fprintf(fid,'\n Neutral point at fixed stick XN/CREF    '); r=sprintf('%-20.5f', NEUXR);   fprintf(fid,'%s',r);                                                                       
fprintf(fid,'\n Control point at fixed stick XC/CREF    '); r=sprintf('%-20.5f', MNEUXR);  fprintf(fid,'%s',r);                                                                       
fprintf(fid,'\n Static margin          (XN-XCG)/CREF    '); r=sprintf('%-20.5f', SMARGR);  fprintf(fid,'%s',r);                                                                       
if ischar(SMARGIR)
    fprintf(fid,'\n Static margin index (XN-XCG)/(XC-XN)    Not Available');
else
    fprintf(fid,'\n Static margin index (XN-XCG)/(XC-XN)    '); r=sprintf('%-20.5f', SMARGIR); fprintf(fid,'%s',r);
end
%
RSTAB.XN = NEUXR;
RSTAB.XC = MNEUXR;
RSTAB.Stab_margin = SMARGR;
RSTAB.Stab_margin_index = SMARGIR;
%
end
%
