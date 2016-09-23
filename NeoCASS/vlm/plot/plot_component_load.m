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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************

function [Y, FS, CL] = plot_component_load(nfig, CAEROID, AERO, FN, QREF, pattern)
%
[dummy, index] = intersect(AERO.ID, CAEROID);
np = length(index);
DOF = AERO.lattice_vlm.DOF;
%
TOTAREA = 0;
cont = 0;
v1 = zeros(1,3); v2 = v1;
for i=1:np
  ind = index(i);
  nx = AERO.geo.nx(ind) + AERO.geo.fnx(ind);
  ny = AERO.geo.ny(ind);
  ind1 = DOF(ind,1,1); ind2=DOF(ind,1,2);  
  dof = [ind1:ind2];
  for k=1:ny
    cont = cont+1;
    FS(cont) = -sum(FN(dof((k-1)*nx+1:k*nx)));
    Y(cont) = AERO.lattice_vlm.COLLOC(dof(k*nx),2);
    p1 = dof((k-1)*nx+1);
    p2 = dof(k*nx);
%
    v1(1:3) = AERO.lattice_vlm.XYZ(p1,1,:) - AERO.lattice_vlm.XYZ(p2,3,:); 
    v2(1:3) = AERO.lattice_vlm.XYZ(p1,2,:) - AERO.lattice_vlm.XYZ(p2,4,:); 
    AREA = norm(cross(v1, v2))/2;
%
    CL(cont) = FS(cont)/AREA/QREF;
    TOTAREA=TOTAREA+AREA;
  end
end
[Ys, index] = sort(Y,'descend');
FSs = FS(index);
CLs = CL(index);
%
figure(nfig); close; figure(nfig);
plot(Ys,CLs,pattern); title('Strip aerodynamic lift coefficient'); ylabel('CL'); xlabel('y [m]');
figure(nfig+1); close; figure(nfig+1);
plot(Ys,cumsum(FSs),pattern); title('Aerodynamic force sum from tip'); ylabel('Force [N]'); xlabel('y [m]');
  