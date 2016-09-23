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
function [Ic, In, Iv, Imv] = interf_vlm_model1(fid, ncaero, NODE, AERO)
% interface collocation points
  fprintf(fid, '\n\t - Assemblying collocation points interpolation matrix...'); 
  Ic = assembly_colloc_interp_mat1(ncaero, NODE, AERO, AERO.lattice.COLLOC);
  fprintf(fid, 'done.'); 
% interface nodes
  fprintf(fid, '\n\t - Assemblying nodes interpolation matrix...'); 
  In = assembly_node_interp_mat1(ncaero, NODE, AERO, AERO.lattice.XYZ);
  fprintf(fid, 'done.'); 
% interface vorticies
  fprintf(fid, '\n\t - Assemblying vorticies interpolation matrix...'); 
  Iv = assembly_vortex_interp_mat1(ncaero, NODE, AERO);
  fprintf(fid, 'done.'); 
% interface mid vorticies for force transfer to structure
  [tot_np, nv, dim] = size(AERO.lattice.VORTEX);
  switch nv
	  case 6
	    col = [3 4];
	  case 8
	    col = [4 5];
	  otherwise
		  error('Wrong size for vortex lattice database. Unable to create interface matrix for vorticies.');
  end
  x1 = zeros(tot_np, 2); x2 = zeros(tot_np, 2); x3 = zeros(tot_np, 2);
  v1 = zeros(tot_np, 1); v2 = zeros(tot_np, 1); v3 = zeros(tot_np, 1);
  x1 = AERO.lattice.VORTEX(:, col,1); x2 = AERO.lattice.VORTEX(:, col,2); x3 = AERO.lattice.VORTEX(:, col,3);
  v1 = mean(x1, 2); v2 = mean(x2, 2); v3 = mean(x3, 2);
  aer_data = [v1, v2, v3];
  fprintf(fid, '\n\t - Assemblying vortices midpoint interpolation matrix...'); 
  Imv = assembly_midv_interp_mat1(ncaero, NODE, AERO, aer_data);
  fprintf(fid, 'done.'); 
end
