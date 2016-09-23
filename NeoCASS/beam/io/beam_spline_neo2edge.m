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
% Export bdis file to Edge binary format
% Input: fbmsh .bmsh mesh file
%        fbsetf .bset Edge file
%        id_str structural IDS
%        str_data structural coordinates
%        str_displ modal displacements (3D matrix)
%        IDmode mode IDS
%        Freqmode mode frequency Hz
%        output headname for bdis files
%        PLOT 1 or 0 to enable / disable plot
function beam_spline_neo2edge(fbmsh, fbset, id_str, str_data, str_displ, IDmode, Freqmode, output, PLOT)
%
toll = 0;
scale = 100;
close all;
NMODES = size(str_displ,3);
AER_DISPL = [];
AER_ID = [];
[ds, ps, ie] = ffabload(fbset, 'mute');
[pc, in, ie] = ffa_find(ds, 'name' ,'moving_surfaces');
[dsub, ie] = ffa_getsub(ds,ps{in});
[psb, ie, et] = ffa_list(dsub, 'mute');
[pb, inb, ie] = ffa_find(dsub, 'name' ,'boundary_name');
%
[pssl, insl, ie] = ffa_find(ds, 'name' ,'sliding_planes');
[dsubsl, ie] = ffa_getsub(ds,ps{insl});
%
[psfree, infree, ie] = ffa_find(ds, 'name' ,'free_surfaces');
[dsubfree, ie] = ffa_getsub(ds,ps{infree});
%
[psfixed, infixed, ie] = ffa_find(ds, 'name' ,'fixed_surfaces');
[dsubfixed, ie] = ffa_getsub(ds,ps{infixed});
%
[de, pe] = ffabload(fbmsh,'mute');
[pc, in, ie] = ffa_find(de, 'name' ,'boundary');
p = ffa_find(de, 'name', 'coordinates');
coord = ffa_get(de,p{1});
%
nfig = 0;
data = {};
orig = [];
id_flag = [];
for n=1:length(inb)

  bname = ffa_get(dsub,psb{inb(n)});
  fprintf('\nBoundary: %s\n', bname);
  str_set = input('List structural nodes ID: ');
  orig(n,1) = input('Node ID to be used as spline origin: ');
  orig(n,2) = input('Node ID to define spline axis from origin: ');
  data{n} = sort(str_set);
  for k=1:length(in)
    [ds,ie] = ffa_getsub(de,pe{in(k)});
    pc = ffa_list(ds, 'mute');
    [pc, in2, ie] = ffa_find(ds, 'name' ,'boundary_name');
    kname = ffa_get(ds,pc{1});
    if (strcmp(strcat(kname),strcat(bname)))
      [pc, in2, ie] = ffa_find(ds, 'name' ,'bound_elem_nodes');
      id_patch = ffa_get(ds,pc{1});
      id_patch = sort(unique(reshape(id_patch, size(id_patch ,1)*size(id_patch,2),1)));
      id_patch = setdiff(id_patch, id_flag);
      AER_ID = [AER_ID; id_patch];
% aer
      naer_patch = length(id_patch);
      coord_patch = coord(id_patch,:);
      id_flag = [id_flag; id_patch];
% str
      [v, str_index, dummy] = intersect(id_str, data{n});
      str_patch = str_data(str_index,:);
      str_displ_patch = str_displ(str_index,:,:);
      nstr_patch = length(str_index);
% spline
      [v, i1, dummy]  = intersect(id_str, orig(n,1));
      [v, i2, dummy]  = intersect(id_str, orig(n,2));
      END = str_data(i2,:);
      ORIG = str_data(i1,:);
      Y = END - ORIG; Y = Y ./ norm(Y);
      X = [1 0 0];
      Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
      X = (crossm(Y) * Z')'; X = X ./ norm(X);
      Rmat1 = [X; Y; Z];
      X = [0 0 -1]; 
      Z = (crossm(X) * Y')'; Z = Z ./ norm(Z);
      X = (crossm(Y) * Z')'; X = X ./ norm(X);
      Rmat2 = [X; Y; Z];
      H = beam_interface2(str_patch, coord_patch, toll, ORIG, Rmat1);
      Hy = beam_interface2(str_patch, coord_patch, toll, ORIG, Rmat2);
      Z = zeros(nstr_patch,1); RX = zeros(nstr_patch,1); RY = zeros(nstr_patch,1);
      Z2 = zeros(nstr_patch,1); RX2 = zeros(nstr_patch,1); RY2 = zeros(nstr_patch,1);
      aer_displ_patch = zeros(naer_patch,6,NMODES);
      for m=1:NMODES
        for mm=1:nstr_patch
          Z(mm,1) = dot(Rmat1(3,:), str_displ_patch(mm,1:3,m));
          ROT = Rmat1 * str_displ_patch(mm,4:6,m)';
          RX(mm,1) = ROT(1); RY(mm,1) = ROT(2);
%
          Z2(mm,1) = dot(Rmat2(3,:), str_displ_patch(mm,1:3,m));
          ROT2 = Rmat2 * str_displ_patch(mm,4:6,m)';
          RX2(mm,1) = ROT2(1); RY2(mm,1) = ROT2(2);
%
        end
        DISPL  = H * [Z;RX;RY]; 
        DISPL2 = Hy * [Z2;RX2;RY2];
        for mm=1:naer_patch
          aer_displ_patch(mm,1:3,m) = DISPL(mm) .* Rmat1(:,3)' ;
          % correct inplane displacements
          aer_displ_patch(mm,1,m) =   DISPL2(mm) .* Rmat2(3,1); 
          aer_displ_patch(mm,2,m) =   DISPL2(mm) .* Rmat2(3,2); 
%
          aer_displ_patch(mm,4:6,m) = Rmat1' * [DISPL(mm+naer_patch); DISPL(mm+2*naer_patch);0];  
        end
        if (PLOT)
          nfig = m; figure(nfig); hold on; 
          plot3(coord_patch(:,1) + scale.*aer_displ_patch(:,1,m),coord_patch(:,2) + scale.*aer_displ_patch(:,2,m),...
               coord_patch(:,3) + scale.*aer_displ_patch(:,3,m),'k.')
          plot3(str_patch(:,1) + scale.*str_displ_patch(:,1,m),str_patch(:,2) + scale.*str_displ_patch(:,2,m),...
               str_patch(:,3) + scale.*str_displ_patch(:,3,m),'rs')
          axis equal;
        end
      end      
%
      AER_DISPL = [AER_DISPL; aer_displ_patch];
      break;
    end
  end
end
%
ds_bdis   = ffa_create('surface_movement');
ds_brand  = ffa_create('brand', 'L', 'Edge 5.2.0 www.foi.se/edge');
ds_title  = ffa_create('title', 'L', 'BEAM SPLINE surface displacement');
% dsub
for k=1:NMODES
  ds   = ffa_create('surface_movement');
  [ds, ie] = ffa_putsub(ds, ds_brand);
  [ds, ie] = ffa_putsub(ds, ds_title);
  [ds, ie] = ffa_putsub(ds, dsub);
  [ds, ie] = ffa_putsub(ds, dsubsl);
  [ds, ie] = ffa_putsub(ds, dsubfree);
  [ds, ie] = ffa_putsub(ds, dsubfixed);
  ds_mode   = ffa_create('mode');
  ds_id = ffa_create('identifier','I', IDmode(k));
  ds_freq= ffa_create('frequency_hz','R', Freqmode(k));
  [ds_mode, ie] = ffa_putsub(ds_mode, ds_id);
  [ds_mode, ie] = ffa_putsub(ds_mode, ds_freq);
  [ds, ie] = ffa_putsub(ds, ds_mode);
  ds_id = ffa_create('nodes_moving','I', AER_ID);
  ds_displ = ffa_create('displacement','R', AER_DISPL(:,1:3,k));
  [ds, ie] = ffa_putsub(ds, ds_id);
  [ds, ie] = ffa_putsub(ds, ds_displ);
%
  fname = [output,'beamspl_displ_',num2str(k),'.bdis'];
  ie = ffa_dump(ds, fname);
end

