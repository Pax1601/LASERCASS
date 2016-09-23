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

% this function simply change wake vortex position according to the actual state
% function extracted from Tornado
function [lattice] = defo_wakesetup(lattice, state, ref) 

infdist=6*ref.b_ref;

[a b c] = size(lattice.VORTEX);

if b~=8
  error('Wrong vortex struct dimension in defo_wakesetup. Wake must exist before being moved');
end

V2 = lattice.VORTEX(:,2:7,:);

c=[2 7];

state.alpha = 0;

infx = infdist * cos(state.alpha) * cos(state.betha);
infy = infdist * sin(state.betha);
infz = infdist * sin(state.alpha) * cos(state.betha);

for t = 1:a
	for s = 1:2

		x = infx + lattice.VORTEX(t,c(s),1);
		y = infy + lattice.VORTEX(t,c(s),2);
		z = infz + lattice.VORTEX(t,c(s),3);

		psi = state.P / state.AS*x;
		theta = state.Q / state.AS*x;
		fi = state.R / state.AS*x;

		dx(t,s) = -x * (2-cos(theta) - cos(fi));
		dy(t,s) = sin(psi) * z - sin(fi) * x + (1-cos(psi)) * y;
		dz(t,s) = sin(theta) * x - sin(psi) * y + (1-cos(psi)) * z;

	end
end

for i=1:a
   
   INF1(i,1,1) = lattice.VORTEX(i,c(1),1) + infx + dx(i,1);
   INF1(i,1,2) = lattice.VORTEX(i,c(1),2) + infy + dy(i,1);
   INF1(i,1,3) = lattice.VORTEX(i,c(1),3) + infz + dz(i,1);
   
   INF2(i,1,1) = lattice.VORTEX(i,c(2),1) + infx + dx(i,2);
   INF2(i,1,2) = lattice.VORTEX(i,c(2),2) + infy + dy(i,2);
   INF2(i,1,3) = lattice.VORTEX(i,c(2),3) + infz + dz(i,2);

end

lattice.VORTEX = [INF1(:,1,:) V2(:,:,:) INF2(:,1,:)];

end
