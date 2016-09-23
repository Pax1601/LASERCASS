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
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
% Update aero mesh in the preprocessor to sample KMAX with a fixed
% number of panel (Param.DLM_NP)
% Aspect ratio is given by Param.DLM_AR
% Maximum reduced frequency is given by Param.DLM_KMAX
%
function [geo, interp] = update_k_mesh(fid, Aero, CDLM, KMAX, AR, NP)
%
geo    = Aero.geo;
interp = Aero.Interp;
%
hx = CDLM*2*pi/NP/KMAX;
hy = AR*hx;
%
fprintf('\n\tAerodynamic mesh updating:');
fprintf('\n\t\t- Maximum reduced frequency: %g.', KMAX); 
fprintf('\n\t\t- Panel aspect ratio: %g.', AR); 
fprintf('\n\t\t- Number of panels in the wavelength: %d.', NP); 
%
fprintf('\n\t\t- Panel chord: %g [m].', hx); 
fprintf('\n\t\t- Panel span: %g [m].',  hy); 
%
for i=1:geo.nwing
%
  ntot = 0;
  chord = geo.c(i);
%
  for k=1:geo.nelem(i)
    c1 = chord;
    c2 = geo.T(i,k) * c1;
    if geo.flapped(i,k)
      c = max(c1-geo.fc(i,k,1)*c1,c2-geo.fc(i,k,2)*c2);
      if (geo.nx(i,k))
        geo.nx(i,k) = ceil(c/hx);
      end 
      if (geo.nx(i,k))
        geo.ny(i,k) = ceil(abs(geo.b(i,k))/hy); 
      end
      ntot = ntot + geo.nx(i,k)*geo.ny(i,k);
      c = max(geo.fc(i,k,1)*c1,geo.fc(i,k,2)*c2);
      if (geo.fnx(i,k))
        geo.fnx(i,k) = ceil(c/hx);
        ntot = ntot + geo.fnx(i,k)*geo.ny(i,k);
      end
    else
      c = max(c1,c2);
      if (geo.nx(i,k))
        geo.nx(i,k) = ceil(c/hx);
      end 
      if (geo.nx(i,k))
        geo.ny(i,k) = ceil(abs(geo.b(i,k))/hy); 
      end
      ntot = ntot + geo.nx(i,k)*geo.ny(i,k);
    end
    chord = c2;
%
  end
  fprintf('\n\t\tTotal number of panel for patch %d: %d.', i, ntot);
  index = find(interp.Patch==Aero.ID(i));
  if (~isempty(index))
    interp.Index(index,1) = 1;
    interp.Index(index,2) = ntot;
  else
    fprintf(fid, '\n\t### Warning: patch %d has no associated spline.', Aero.ID(i));
  end
%
end
fprintf(fid,'\n\t\tdone.');
end





