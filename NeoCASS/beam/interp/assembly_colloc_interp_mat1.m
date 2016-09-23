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

function I = assembly_colloc_interp_mat1(ncaero, NODE, AERO, COLLOC)

I = [];
rh = [];
ch = [];
vh = [];
%
ngrid = size(NODE.Coord, 1); % total nodes
tot_np = size(COLLOC, 1);
ntot = 0;
ndof = max(max(NODE.DOF));
%
for n=1:ncaero
	np = 1 + AERO.lattice.DOF(n, AERO.geo.nelem(n), 2) - AERO.lattice.DOF(n, 1, 1); % total stored panels on patch
  IDPA=[AERO.lattice.DOF(n, 1, 1):AERO.lattice.DOF(n, 1, 2)];
	SI = AERO.IS(n).data; % interpolation set index
  if (sum(SI)>0)
    SET_AV = unique(SI);  % find interpolation set used
	  for i = 1:length(SET_AV) % loop on set
      if (SET_AV(i)>0)
        ri = [];
        ci = [];
        vi = [];
	      ninterp = SET_AV(i);
		    nset = AERO.Interp.Set(ninterp);
        PANEL = find(SI == ninterp);
        naer = length(PANEL);
		    str_data = NODE.Coord(AERO.Set.Node(nset).data,1:3); % get stuctural coords
        nstr = size(str_data,1);
%
        i1 = AERO.Interp.Param(ninterp, 1);
        i2 = AERO.Interp.Param(ninterp, 2);
        toll = AERO.Interp.Param(ninterp, 3);
        [Ic, Rmat] = beam_interface(str_data, COLLOC(PANEL+ntot, 1:3), i1, i2, toll); % interface points in local frame
%------------------------------------------------------------------------------------------------------------------
%     R picks DOF and rotates them in the local frame
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
%   Rt rotates displacements in the absolute frame 
        ri = [];
        ci = [];
        vi = [];
        RmatT = Rmat';
        for k=1:naer
          ri = [ri, IDPA(PANEL(k)), IDPA(PANEL(k))+tot_np, IDPA(PANEL(k))+2*tot_np];
          ci = [ci, repmat(k, 1, 3)];
          vi = [vi, RmatT(:,3)'];
          ri = [ri, repmat([IDPA(PANEL(k))+3*tot_np, IDPA(PANEL(k))+4*tot_np, IDPA(PANEL(k))+5*tot_np],1,2)];
          ci = [ci, repmat(k+naer, 1, 3), repmat(k+2*naer, 1, 3)];
          vi = [vi, RmatT(:,1)', RmatT(:,2)'];
        end
        nrows = double(6*tot_np); ncols = double(3*naer);
        Rt = sparse(ri, ci, vi, nrows, ncols);
%------------------------------------------------------------------------------------------------------------------
%     Assemble interpolation matrix
        I = Rt * Ic * R; 
        [r, c, v] = find(I);
%     offset rows
        rh =  [rh; r];
        ch =  [ch; c];
        vh =  [vh; v];
      end
    end % set loop
  end
  ntot = ntot + np;
end % thread loop
%
rh = double(rh);
ch = double(ch);
vh = double(vh);
nrows = double(6*tot_np);
ncols = double(ndof);
I = sparse(rh, ch, vh, nrows, ncols);
%
end