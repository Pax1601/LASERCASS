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

function plot_dlm_xy_CP(geo, lattice, dlm, patch_index, stripe_index, mode_index, freq_index, Mach_index, nfig)

if (nargin == 8)
  nfig = 1;
end

figure(nfig); close; figure(nfig);
hold on;

if (stripe_index > geo.ny(patch_index))
  error('Strip index exceed given mesh.');
end
% get offset
offset = sum( (geo.nx(1:(patch_index-1))+geo.fnx(1:(patch_index-1)) ) .* geo.ny(1:(patch_index-1))) + ...
              (stripe_index-1) * (geo.nx(patch_index) + geo.fnx(patch_index)) + 1;

%chord = 1.0 / abs(lattice.XYZ(offset, 1, 1) - lattice.XYZ((offset + geo.nx(patch_index) -1), 4, 1));
chord = dlm.aero.cref;

plot(chord .* (lattice.MID_DPOINT(offset : (offset + geo.nx(patch_index) -1), 1)), ...
     real(-dlm.data.Cp(offset : (offset + geo.nx(patch_index) -1), mode_index, freq_index, Mach_index)), ...
     '-ro', 'MarkerSize', 6, 'MarkerFaceColor','r', 'LineWidth',1);

plot(chord .* (lattice.MID_DPOINT(offset : (offset + geo.nx(patch_index) -1), 1)), ...
     imag(-dlm.data.Cp(offset : (offset + geo.nx(patch_index) -1), mode_index, freq_index, Mach_index)), ...
     '-ko', 'MarkerSize', 6, 'MarkerFaceColor','k', 'LineWidth',1);
	 
label1 = ['Real'];
label2 = ['Imag'];

H = legend(label1, label2, 1);
set(H, 'EdgeColor', [1 1 1])
set(H, 'Color', 'none')

labelt = ['Mach ', num2str(dlm.aero.M(Mach_index)), ', k ', num2str(dlm.aero.k(freq_index)), ', Mode ', num2str(mode_index)];
title(labelt); 
xlabel('X coord');
ylabel('-Cp');
