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

function Ic = interf_body_model1(fid, NODE, AERO)
%
  Ic = [];
  toll = 0;
  nbody = length(AERO.body.ID);
  if (nbody)
    fprintf(fid, '\n\t - Assemblying body collocation points interpolation matrix...');
    ndof = max(max(NODE.DOF));
%   body loop
    for i=1:nbody
%
      nset = AERO.body.SET(i);
      if (nset>0)
        % use body reference frame for spline
        ORIG = AERO.body.geo.ref_point(i,:);
        % rotate frame to have y along body axis
        Rmatz = (expm(crossm([0 0 -pi/2]))*AERO.body.Rmat(:,:,i))'; 
        % rotate frame to have y along body axis
        Rmaty = (expm(crossm([pi/2 0 0]))*expm(crossm([0 0 -pi/2]))*AERO.body.Rmat(:,:,i))'; 
        % aero data
        aero_data = AERO.body.lattice.Elem.Midpoint_loc{i};
        naer = size(aero_data,1);
        % structural data
        str_data = NODE.Coord(AERO.Set.Node(nset).data,:);     
        nstr = size(str_data,1);
        Hz = beam_interface2(str_data, aero_data, toll, ORIG, Rmatz);
        Hy = beam_interface2(str_data, aero_data, toll, ORIG, Rmaty);
        ri = []; ci = []; viz = []; viy = [];
%       R rotates forces in the local frame: pick only z component
        for k=1:naer
          ri = [ri, k.*ones(1, 3)];
          ci = [ci, (k-1)*3+1:k*3];
          viz = [viz, Rmatz(3,:)];
          viy = [viy, Rmaty(3,:)];
        end
        ri = double(ri); ci = double(ci); viz = double(viz); viy = double(viy);
%       add 2naer zeros in rows for bending and torque 
        nrows = double(3*naer); ncols = double(3*naer); 
        Rz = sparse(ri, ci, viz, nrows, ncols);
        Ry = sparse(ri, ci, viy, nrows, ncols);
        % Rt rotates aero forces in the absolute frame and send them to nodal DOF
        ri = []; ci = []; viz = []; viy = [];
        RmatTz = Rmatz';
        RmatTy = Rmaty';
        for k=1:nstr
          index = find(NODE.DOF(AERO.Set.Node(nset).data(k),1:3)); % check translation DOF are not fixed
          if (~isempty(index))
            ri = [ri, NODE.DOF(AERO.Set.Node(nset).data(k),index)];  
            ci = [ci, k.*ones(1,length(index))];
            viz = [viz, RmatTz(index,3)'];
            viy = [viy, RmatTy(index,3)'];
          end
          index = find(NODE.DOF(AERO.Set.Node(nset).data(k),4:6)); % check rotation DOF are not fixed
          if (~isempty(index))
            ri = [ri, repmat(NODE.DOF(AERO.Set.Node(nset).data(k),3+index), 1, 2)];  
            ci = [ci, repmat((nstr+k), 1, length(index)), repmat((2*nstr+k), 1, length(index))];
            viz = [viz, RmatTz(index,1)', RmatTz(index,2)'];
            viy = [viy, RmatTy(index,1)', RmatTy(index,2)'];
          end
        end
        ri = double(ri); ci = double(ci); viz = double(viz); viy = double(viy);
        nrows = double(ndof); ncols = double(3*nstr);
        Rtz = sparse(ri, ci, viz, nrows, ncols);
        Rty = sparse(ri, ci, viy, nrows, ncols);
        %------------------------------------------------------------------------------------------------------------------
        %   Assemble interpolation matrix
        I = Rtz * (Hz') * Rz + Rty * (Hy') * Ry; 
        [r, c, v] = find(I); r = double(r); c = double(c); v = double(v); 
        nrows = double(ndof); ncols = double(3*naer);
        Ic{i} = sparse(r, c, v, nrows, ncols);
      end
    end
    fprintf(fid, 'done.');
  end
