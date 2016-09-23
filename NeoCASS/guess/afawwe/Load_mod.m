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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Main function to compute total tension and compression stress resultants
% along fuselage length, considering a pull-up maneuver at given normal
% load factor, a taxi over a runway bump and hard landing with specified
% sink velocity.
%
% Called by:    AFaWWE_mod.m
% 
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080722      1.0     A. Da Ronch      Creation
%*******************************************************************************
function loads = Load_mod(fid, pdcylin, aircraft, geo, manloads, NZ, VEL, ntrim, nconf)
%
%
  loads.nconf = nconf;
  loads.wing.FS = [];
  loads.wing.M = [];
  loads.wing.Mt = [];
  loads.vtail.FS = [];
  loads.vtail.M = [];
  loads.vtail.Mt = [];
  loads.htail.FS = [];
  loads.htail.M = [];
  loads.htail.Mt = [];
  loads.canard.FS = [];
  loads.canard.M = [];
  loads.canard.Mt = [];
  loads.fus.Nxt = [];
  loads.fus.Nxc = [];
  loads.fus.Ny = [];
  loads.fus.FS = [];
  loads.fus.M = [];
  loads.fus.Mt = [];
  loads.tbooms.Nxc = [];
%
  KS = pdcylin.smartcad.Ks;
%
  loads.nv.N = NZ;
  loads.nv.V = VEL;
%
  if isequal(pdcylin.stick.model.fuse, 1)
%
    IndB = find(geo.fus.r ~= 0); 
%   pressure load
    [NxP, Ny] = Fus_Press(aircraft, geo);
    loads.fus.Ny = KS .* Ny;
    NxP = KS .* NxP;
%   guess loads
    FS = KS .* squeeze(manloads.fus(:,2,:));
    ne = size(FS,1); nc = size(FS,2);
    Mt = KS .* squeeze(manloads.fus(:,4,:));
    M  = KS .* squeeze(manloads.fus(:,6,:));
    NxB = zeros(ne, nc);
    for k=1:length(IndB)
      NxB(IndB(k),:) = M(IndB(k),:) ./ (pi*geo.fus.r(IndB(k))^2);
    end
    loads.fus.man.Nxt = repmat(NxP,1,nc) + NxB;
    loads.fus.man.Nxc = abs(NxB - repmat(NxP,1,nc).*isequal(pdcylin.fact.stab, 1));
%   sizing loads
    loads.fus.Nxt  = max(loads.fus.man.Nxt, [], 2);
    loads.fus.Nxc  = max(loads.fus.man.Nxc, [], 2);
    [loads.fus.FS, loads.fus.man.FS_i] = max(abs(FS),[],2);   
    [loads.fus.Mt, loads.fus.man.Mt_i] = max(abs(Mt),[],2);
    [loads.fus.M, loads.fus.man.M_i] = max(abs(M),[],2);
%   maneuver loads
    loads.fus.man.FS = reshape(FS,ne,ntrim,nconf); 
    loads.fus.man.Mt = reshape(Mt,ne,ntrim,nconf);
    loads.fus.man.M  = reshape(M,ne,ntrim,nconf);
    index = [];
%   store max conf index
    for k=1:nconf
      index = [index, ones(1,ntrim) .* k];
    end
    loads.fus.man.FS_j = index(loads.fus.man.FS_i)'; 
    loads.fus.man.Mt_j = index(loads.fus.man.Mt_i)'; 
    loads.fus.man.M_j  = index(loads.fus.man.M_i)'; 
%   store max maneuver index
    index = repmat([1:ntrim], 1, nconf);
    loads.fus.man.FS_k = index(loads.fus.man.FS_i)';
    loads.fus.man.Mt_k = index(loads.fus.man.Mt_i)';
    loads.fus.man.M_k = index(loads.fus.man.M_i)';
%
    loads.fus.man.FS_i = zeros(ne, nconf);
    loads.fus.man.Mt_i = zeros(ne, nconf);
    loads.fus.man.M_i = zeros(ne, nconf);
%
    for k=1:nconf
      [v, loads.fus.man.FS_i(:,k)] = max(abs(loads.fus.man.FS(:,:,k)),[],2);
      [v, loads.fus.man.Mt_i(:,k)] = max(abs(loads.fus.man.Mt(:,:,k)),[],2);
      [v, loads.fus.man.M_i(:,k)] = max(abs(loads.fus.man.M(:,:,k)),[],2);
    end
%
  end
  if isfield(aircraft,'Tailbooms')
     if aircraft.Tailbooms.present
      M = KS .* squeeze(manloads.tboomsr(:,6,:));
      ne = size(M,1); nc = size(M,2);
      NxB = zeros(ne, nc);
      for k=1:nc
        NxB(:,k) = M(:,k) ./ geo.tbooms.A;
      end
%     loads.tbooms.Nxc = abs(loads.tbooms.NxB) + KS*max(abs(manloads.tboomsr(:,1)),abs(manloads.tboomsl(:,1)))./geo.tbooms.P;
      loads.tbooms.man.Nxc = abs(NxB);
      loads.tbooms.Nxc = max(loads.tbooms.man.Nxc, [], 2);
      [loads.tbooms.M, loads.tbooms.man.M_i] = max(abs(M),[],2);
      loads.tbooms.man.M  = reshape(M,ne,ntrim,nconf);
      index = [];
%     store max conf index
      for k=1:nconf
        index = [index, ones(1,ntrim) .* k];
      end
      loads.tbooms.man.M_j  = index(loads.tbooms.man.M_i)'; 
%     store max maneuver index
      index = repmat([1:ntrim], 1, nconf);
      loads.tbooms.man.M_k = index(loads.tbooms.man.M_i)';
      loads.tbooms.man.M_i = zeros(ne, nconf);
      for k=1:nconf
        [v, loads.tbooms.man.M_i(:,k)] = max(abs(loads.tbooms.man.M(:,:,k)),[],2);
      end
    end
  end
  if isequal(pdcylin.stick.model.winr, 1)
    [loads.wing, loads.wing.man] = lift_comp_load(manloads.winr, pdcylin.stick.nwing_carryth+1, KS, ntrim, nconf);
  end
  %
  if aircraft.Horizontal_tail.present
    offset = 1;
    if (geo.htail.twc >0)
      offset = pdcylin.stick.nhtail_carryth+1;
    end
    [loads.htail, loads.htail.man] = lift_comp_load(manloads.horr, offset, KS, ntrim, nconf);
  end
  %
  if isequal(pdcylin.stick.model.vert, 1)
    [loads.vtail, loads.vtail.man] = lift_comp_load(manloads.vtail, 1, KS, ntrim, nconf);
  end
  %
  if aircraft.Canard.present
    [loads.canard, loads.canard.man] = lift_comp_load(manloads.canr, pdcylin.stick.ncanard_carryth+1, KS, ntrim, nconf);
  end
end
%--------------------------------------------------------------------------------------------------
function [loads, loadsman] = lift_comp_load(manloads, offset, KS, ntrim, nconf)
%
  loads.N = []; 
  loads.FS = []; 
  loads.Mt = []; 
  loads.M = []; 
  loadsman.FS = []; 
  loadsman.Mt = [];
  loadsman.M = [];
  loadsman.N_i = []; 
  loadsman.FS_i = []; 
  loadsman.Mt_i = [];
  loadsman.M_i = [];
  loadsman.FS_j = []; 
  loadsman.Mt_j = [];
  loadsman.M_j = [];
%
  N = KS .* squeeze(manloads(offset:end,1,:));
  FS = KS .* squeeze(manloads(offset:end,2,:));
  Mt = KS .* squeeze(manloads(offset:end,4,:));
  M  = KS .* squeeze(manloads(offset:end,6,:));
  [loads.N, loadsman.N_i] = min(N,[],2); 
  [loads.FS, loadsman.FS_i] = max(abs(FS),[],2); 
  [loads.Mt, loadsman.Mt_i] = max(abs(Mt),[],2); 
  [loads.M, loadsman.M_i] = max(abs(M),[],2);
  ne = size(FS,1); 
  loadsman.N = reshape(N,ne,ntrim,nconf); 
  loadsman.FS = reshape(FS,ne,ntrim,nconf); 
  loadsman.Mt = reshape(Mt,ne,ntrim,nconf);
  loadsman.M  = reshape(M,ne,ntrim,nconf);
  index = [];
  for k=1:nconf
      index = [index, ones(1,ntrim) .* k];
  end
  loadsman.N_j  = index(loadsman.N_i)'; 
  loadsman.FS_j = index(loadsman.FS_i)'; 
  loadsman.Mt_j = index(loadsman.Mt_i)'; 
  loadsman.M_j  = index(loadsman.M_i)'; 
%
  index = repmat([1:ntrim], 1, nconf);
  loadsman.N_k = index(loadsman.N_i)';
  loadsman.FS_k = index(loadsman.FS_i)';
  loadsman.Mt_k = index(loadsman.Mt_i)';
  loadsman.M_k = index(loadsman.M_i)';
%
  loadsman.N_i = zeros(ne, nconf);
  loadsman.FS_i = zeros(ne, nconf);
  loadsman.Mt_i = zeros(ne, nconf);
  loadsman.M_i = zeros(ne, nconf);
%
  for k=1:nconf
    [v, loadsman.N_i(:,k)] = min(loadsman.N(:,:,k),[],2);
    [v, loadsman.FS_i(:,k)] = max(abs(loadsman.FS(:,:,k)),[],2);
    [v, loadsman.Mt_i(:,k)] = max(abs(loadsman.Mt(:,:,k)),[],2);
    [v, loadsman.M_i(:,k)] = max(abs(loadsman.M(:,:,k)),[],2);
  end
%
end