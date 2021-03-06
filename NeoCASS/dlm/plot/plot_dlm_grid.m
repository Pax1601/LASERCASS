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
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function plot_dlm_grid(lattice, dlm, nfig)

if (nargin == 2)
  nfig = 1;
end

cref = dlm.aero.cref;
x = cref * lattice.COLLOC(1:lattice.np,1);
y = cref * lattice.COLLOC(1:lattice.np,2);
z = cref * lattice.COLLOC(1:lattice.np,3);

figure(nfig); close; figure(nfig);
plot3(x,y,z,'xr');
hold on
grid;

for n = 1:lattice.np
	
	xv = cref * lattice.XYZ(n,:,1);
	yv = cref * lattice.XYZ(n,:,2);
	zv = cref * lattice.XYZ(n,:,3);
    plot3(xv,yv,zv,'-');

	d_linex = cref * [lattice.DOUBLET(n,1,1) , lattice.DOUBLET(n,2,1)];
	d_liney = cref * [lattice.DOUBLET(n,1,2) , lattice.DOUBLET(n,2,2)];
	d_linez = cref * [lattice.DOUBLET(n,1,3) , lattice.DOUBLET(n,2,3)];
	plot3(d_linex, d_liney, d_linez,'r--');

end

x = cref * lattice.MID_DPOINT(1:lattice.np,1);
y = cref * lattice.MID_DPOINT(1:lattice.np,2);
z = cref * lattice.MID_DPOINT(1:lattice.np,3);
plot3(x,y,z,'.r');

if (dlm.param.symm)
	
	offset = lattice.np + 1;
	np_tot = lattice.npsymm + lattice.np;
	
	x = cref * lattice.COLLOC(offset:np_tot,1);
	y = cref * lattice.COLLOC(offset:np_tot,2);
	z = cref * lattice.COLLOC(offset:np_tot,3);
	plot3(x,y,z,'xk');

	for n = offset:np_tot

		xv = cref * lattice.XYZ(n,:,1);
		yv = cref * lattice.XYZ(n,:,2);
		zv = cref * lattice.XYZ(n,:,3);
    	plot3(xv,yv,zv,'-k');

		d_linex = cref * [lattice.DOUBLET(n,1,1) , lattice.DOUBLET(n,2,1)];
		d_liney = cref * [lattice.DOUBLET(n,1,2) , lattice.DOUBLET(n,2,2)];
		d_linez = cref * [lattice.DOUBLET(n,1,3) , lattice.DOUBLET(n,2,3)];
		plot3(d_linex, d_liney, d_linez,'k--');

	end

	x = cref * lattice.MID_DPOINT(offset:np_tot,1);
	y = cref * lattice.MID_DPOINT(offset:np_tot,2);
	z = cref * lattice.MID_DPOINT(offset:np_tot,3);

	plot3(x,y,z,'.k');

end

axis equal;
view(2);

title('Doublet Lattice grid');
