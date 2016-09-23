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
% Export a .aigi spatial coupling file to Edge binary format
% Input: bsetf .bset Edge file
%        outf, output filename BIGI extension
function neo2edge_mls_cou(bsetf, outf)
%
[ds, ps, ie] = ffabload(bsetf, 'mute');
[pc, in, ie] = ffa_find(ds, 'name' ,'moving_surfaces');
[dsub, ie] = ffa_getsub(ds,ps{in});
[psb, ie, et] = ffa_list(dsub);
[pb, inb, ie] = ffa_find(dsub, 'name' ,'boundary_name');
%
ds_aigi   = ffa_create('igi_interp_data');
ds_title  = ffa_create('title', 'L', 'intergrid interpolation data');
ds_method = ffa_create('igi_method', 'L', 'MLS');
%
[ds_aigi, ie] = ffa_putsub(ds_aigi, ds_title);
[ds_aigi, ie] = ffa_putsub(ds_aigi, ds_method);
%
for n=1:length(inb)
  bname = ffa_get(dsub,psb{inb(n)});
  fprintf('\nBoundary: %s', bname);
  poly = input('\nPolynomial: ');
  weight = input('\nWeight: ');
  np = input('\nPoints: ');
  cond = input('\nConditioning number: ');
  str_set = input('\nStructural set: ');
  rad_set = input('\nMaximum radius: ');
%
  ds_bc   = ffa_create('boundary');
  ds_name = ffa_create('b_name', 'L', bname);
  ds_poly = ffa_create('poly','I', poly); 
  ds_w    = ffa_create('weight','I',weight); 
  ds_np   = ffa_create('n_points','I', np); 
%
  [ds_bc, ie] = ffa_putsub(ds_bc, ds_name);
  [ds_bc, ie] = ffa_putsub(ds_bc, ds_poly);
  [ds_bc, ie] = ffa_putsub(ds_bc, ds_w);
  [ds_bc, ie] = ffa_putsub(ds_bc, ds_np);
%
  if (~isempty(cond))
      ds_cond = ffa_create('cond','R',cond);
      [ds_bc, ie] = ffa_putsub(ds_bc, ds_cond);
  end
  if (~isempty(str_set))
      ds_set  = ffa_create('grid_idents','I',str_set);
      [ds_bc, ie] = ffa_putsub(ds_bc, ds_set);
  end
  if(~isempty(rad_set))
      ds_rad  = ffa_create('r_max','R',rad_set);
      [ds_bc, ie] = ffa_putsub(ds_bc, ds_rad);
  end
%
  [ds_aigi, ie] = ffa_putsub(ds_aigi, ds_bc);
%
end

ie = ffa_dump(ds_aigi, outf);
