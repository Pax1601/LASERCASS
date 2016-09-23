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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     092211      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function edge2nas_force(filename, outname)
%
%   DESCRIPTION: Export NASTRAN displacements to Edge FFA format for coupled 
%   aeroelastics (ISOOPT = 202)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename       FILE       Input filename 
%                                          (.ainp .binp Edge files)
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                outname        FILE       Output filename with forces 
%                                          in NASTRAN format
%
%    REFERENCES:
%
%*******************************************************************************
%
function edge2nas_force(filename, outname)
%
[headname, ext] = strtok(filename, '.');
if (isempty(ext))
  error('Filename has no extension.');
end
%
if (ext(2) == 'a')
  [ds, ps]= ffaaload(filename);
else
  [ds, ps]= ffabload(filename);
end
%
[pc,in,ierr] = ffa_find(ds,'name','grid_idents');
%
if (in==0)
  error('Unable to find grid_idents dataset.');
end
%
id = ffa_get(ds, ps{in});
%
[pc,in,ierr] = ffa_find(ds,'name','force');
%
if (in==0)
  error('Unable to find force dataset.');
end
%
F = ffa_get(ds, ps{in});
nstr = length(id);
TOTF = sum(F);
%
fp = fopen(outname, 'w');
%
SET = 1;
REF = 0;
if (size(F,2) == 3)
    TOTF = zeros(3,1);
    for k=1:nstr
    Fmag = norm(F(k,:));
    if (Fmag > eps)
      fprintf(fp, 'FORCE,%d,%d,%d,',SET,  id(k), REF);
      fprintf(fp, '%f,%f,%f,%f\n'  ,Fmag, F(k,1)/Fmag, F(k,2)/Fmag, F(k,3)/Fmag);
      TOTF(1) = TOTF(1) + F(k,1);
      TOTF(2) = TOTF(2) + F(k,2);
      TOTF(3) = TOTF(3) + F(k,3);
    end
  end
else
    TOTF = zeros(2,1);
    for k=1:nstr
    Fmag = norm(F(k,:));
    if (Fmag > eps)
      fprintf(fp, 'FORCE,%d,%d,%d,',SET,  id(k), REF);
      fprintf(fp, '%f,%f,%f,0.0\n'  ,Fmag, F(k,1)/Fmag, F(k,2)/Fmag);
      TOTF(1) = TOTF(1) + Fmag*F(k,1);
      TOTF(2) = TOTF(2) + Fmag*F(k,2);
    end
  end
end
%
fclose(fp);
fprintf(1, 'Total force: ');
disp(TOTF);



