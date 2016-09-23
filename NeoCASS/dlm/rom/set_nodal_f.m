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
% Exports a rectangular aero matrix ndof x nmodes
% Modal Cp are interpolated along FE mesh as force and moments
function Qnh = set_nodal_f(fid, info, NODE, nmodes, AERO, k_n, Mach_n, CP)
%
area = AERO.lattice.area;
N = AERO.lattice.N;
ndof = max(max(NODE.DOF));
Qnh = zeros(ndof, nmodes, k_n, Mach_n);
%
A = repmat(area, 1, 3);
normal_area = N .* A;
%
for m = 1: Mach_n % Mach loop
	for n = 1 : nmodes % mode loop
		for k = 1: k_n % red. freq. loop
%
          Fxa = CP(:, n, k, m) .* normal_area(:,1);
          Fya = CP(:, n, k, m) .* normal_area(:,2);
          Fza = CP(:, n, k, m) .* normal_area(:,3);
          results.F = [Fxa, Fya, Fza];
          results.Fb = {};
%
          Qnh(:, n, k, m) = gf_transfer_aero_nodal(info, NODE.DOF, NODE, AERO , results);
%
		end
	end
end
%
end
%
