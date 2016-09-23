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

function manloads = gmod_loads(internal, stick, BAR, BEAM, ntrim)
% 
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
% fuselage
manloads.fus = store_loads(length(stick.ID.fuse_thick), internal, BAR, stick.EID.fuse_thick);
% wing 1 right
if stick.model.winr
  manloads.winr = store_loads(length(stick.ID.winr_thick), internal, BAR, stick.EID.winr_thick);
end
% wing 1 left
if stick.model.winr
  manloads.winl = store_loads(length(stick.ID.winl_thick), internal, BAR, stick.EID.winl_thick);
end
% htail  right
if stick.model.horr
  manloads.horr = store_loads(length(stick.ID.horr_thick), internal, BAR, stick.EID.horr_thick);
end
% htail  right
if stick.model.horl
  manloads.horl = store_loads(length(stick.ID.horl_thick), internal, BAR, stick.EID.horl_thick);
end
% canard  right
if stick.model.canr
  manloads.canr = store_loads(length(stick.ID.canr_thick), internal, BAR, stick.EID.canr_thick);
end
% canard  left
if stick.model.canl
  manloads.canl = store_loads(length(stick.ID.canl_thick), internal, BAR, stick.EID.canl_thick);
end
% wing 2 right
if stick.model.win2r
  manloads.win2r = store_loads(length(stick.ID.win2r_thick), internal, BAR, stick.EID.win2r_thick);
end
% wing 2 left
if stick.model.win2r
  manloads.win2l = store_loads(length(stick.ID.win2l_thick), internal, BAR, stick.EID.win2l_thick);
end
% vert
if stick.model.vert
  manloads.vtail = store_loads(length(stick.ID.vert_thick), internal, BAR, stick.EID.vert_thick);
end
% vert2
if stick.model.vert2
  manloads.vtail2 = store_loads(length(stick.ID.vert2_thick), internal, BAR, stick.EID.vert2_thick);
end
% tbooms right
if stick.model.tboomsr
  manloads.tboomsr = store_loads(length(stick.ID.tboomsr_thick), internal, BAR, stick.EID.tboomsr_thick);
end
% tbooms right
if stick.model.tboomsl
  manloads.tboomsl = store_loads(length(stick.ID.tboomsl_thick), internal, BAR, stick.EID.tboomsl_thick);
end
%
end
%-------------------------------------------------------------------------------
function loads = store_loads(ns, internal, BAR, EID)
%  
  ntrim = size(internal,3);
  loads = zeros(ns,6,ntrim);
% 1st beam
  ALL = internal(BAR.ID == EID(1),:,:);
  loads(1,:,:) = ALL(1,1:6,:);
  loads(2,:,:) = ALL(1,7:12,:);
% 2nd beam -> end-1
  for i = 2 : ns-1
    ALL = internal(BAR.ID == EID(i),:,:);
    for k=1:ntrim
      nv = [ALL(1,1:6,k);loads(i,:,k)];
      [v,index] = max(abs(nv),[],1);
      for j=1:length(index)
        loads(i,j,k) = nv(index(j),j);
      end
    end
   loads(i+1,:,:) = ALL(1,7:12,:);
  end
%
end