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
%
%***********************************************************************************************************************
%
%  NeoSYM
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna
%***********************************************************************************************************************
%
function MAT = aero_interp_M(Mlist, Klist, AERO, MACH)
%function MAT = aero_interp_M(fid, Mlist, Klist, AERO, MACH)
%
nr = size(AERO,1);
nc = size(AERO,2);
nk = size(AERO,3);
nm = size(AERO,4);
if (length(Klist)~=nk)
  error('MKAERO red. frequency list and matrix dimensions do not agree.');
end
if (length(Mlist)~=nm)
  error('MKAERO Mach list and matrix dimensions do not agree.');
end
%
str = [];
for r=1:nm
  str = [str, ' ', num2str(Mlist(r))];
end
%fprintf(fid, ['\n\t - Mach numbers available for Aero matrix:', str,'.']);
MAT = zeros(nr, nc, nk);
%
if(nm==1)
  MAT = AERO;
%  fprintf(fid, '\n\t - No interpolation required. The available Mach number %g will be used.', Mlist(1));
else
%
  for r=1:nr
    for c=1:nc
      for k=1:nk
        MAT(r,c,k) = interp1(Mlist, squeeze(AERO(r,c,k,:)), MACH, 'spline', 'extrap');
      end
    end
  end       
%  fprintf(fid, '\n\t - Interpolation to Mach %g completed.\n', MACH);
end
%
end % function

