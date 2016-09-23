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

function [I] = assembly_node_interp_mat1(ncaero, NODE, AERO, XYZ)
I = [];
rh = [];
ch = [];
vh = [];
loc_offset = 0;
coordp=[];
ngrid = size(NODE.Coord, 1); % total nodes
tot_np = size(XYZ, 1);
ndof = max(max(NODE.DOF));
offset = 0;
for n=1:ncaero
	np = 1 + AERO.lattice.DOF(n, AERO.geo.nelem(n), 2) - AERO.lattice.DOF(n, 1, 1); % total stored panels on patch
	SI = AERO.IS(n).data; % interpolation set index
  if (sum(SI)>0)
    SET_AV = unique(SI);  % find interpolation set used
    IDPA=[AERO.lattice.DOF(n, 1, 1):AERO.lattice.DOF(n, 1, 2)];
	  for i = 1:length(SET_AV) % loop on set
      if (SET_AV(i)>0)
        ri = [];
        ci = [];
        vi = [];
	      ninterp = SET_AV(i);
		    nset = AERO.Interp.Set(ninterp);
        PANEL = find(SI == ninterp);
        NP = length(PANEL);
        ID = [];
		    str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords
        nstr = size(str_data,1);
		    aero_data = zeros(NP*4,3);
        loc_offset = 0;
        for nl = 1: NP
          index  = [(IDPA(PANEL(nl))-1)*4+1: IDPA(PANEL(nl))*4];
			    aero_data(loc_offset+1:loc_offset+4, 1) = XYZ(PANEL(nl)+offset, 1:4, 1);
			    aero_data(loc_offset+1:loc_offset+4, 2) = XYZ(PANEL(nl)+offset, 1:4, 2);
			    aero_data(loc_offset+1:loc_offset+4, 3) = XYZ(PANEL(nl)+offset, 1:4, 3);
          loc_offset = loc_offset + 4;
          ID  = [ID , (IDPA(PANEL(nl))-1)*4+1: IDPA(PANEL(nl))*4];
        end
        ID = ID';
        i1 = AERO.Interp.Param(ninterp, 1);
        i2 = AERO.Interp.Param(ninterp, 2);
        toll = AERO.Interp.Param(ninterp, 3);
        [In, Rmat] = beam_interface(str_data, aero_data, i1, i2, toll); % interface points in local frame
%------------------------------------------------------------------------------------------------------------------
%    R picks DOF and rotates them in the local frame
        for k=1:nstr
          index = find(NODE.DOF(AERO.Set.Node(nset).data(k),1:3)); % check translation DOF are not fixed
          if(~isempty(index))
            ri = [ri, repmat(k, 1, length(index))];
            ci = [ci, NODE.DOF(AERO.Set.Node(nset).data(k),index)];
            vi = [vi, Rmat(3,index)];
          end
          index = find(NODE.DOF(AERO.Set.Node(nset).data(k),4:6));
          if(~isempty(index))
            ri = [ri, repmat(nstr + k, 1, length(index)), repmat(2*nstr + k, 1, length(index))];
            ci = [ci, NODE.DOF(AERO.Set.Node(nset).data(k),index+3), NODE.DOF(AERO.Set.Node(nset).data(k),index+3)];
            vi = [vi, Rmat(1,index), Rmat(2,index)];
          end
        end
        ri = double(ri);
        ci = double(ci);
        vi = double(vi);
        nrows = double(3*nstr);
        ncols = double(ndof);
        R = sparse(ri, ci, vi, nrows, ncols);
%
%------------------------------------------------------------------------------------------------------------------
%     Rt rotates aero displacements in the absolute frame
        naer = length(PANEL)*4;
        ri = [];
        ci = [];
        vi = [];
        RmatT = Rmat';
        for k=1:naer
          ri = [ri, ID(k)];
          ci = [ci, k];
          vi = [vi, RmatT(1,3)];
        end
        for k=1:naer
          ri = [ri, ID(k)+tot_np*4];
          ci = [ci, k];
          vi = [vi, RmatT(2,3)];
        end
        for k=1:naer
          ri = [ri, ID(k)+tot_np*8];
          ci = [ci, k];
          vi = [vi, RmatT(3,3)];
        end
        nrows = double(4*3*tot_np);
        ncols = double(3*naer);
        Rt = sparse(ri, ci, vi, nrows, ncols);
%------------------------------------------------------------------------------------------------------------------
%     Assemble interpolation matrix
        I = Rt * In * R; 
        [r, c, v] = find(I);
%     offset rows
        rh =  [rh; r];
        ch =  [ch; c];
        vh =  [vh; v];
      end
    end % set loop
  end 
  offset = offset + np;
end % thread loop
%
rh = double(rh);
ch = double(ch);
vh = double(vh);
nrows = double(12*tot_np);
ncols = double(ndof);
I = sparse(rh, ch, vh, nrows, ncols);
%
end
%
