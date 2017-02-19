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
% assembly stiffness matrix for linear calculation
function K = st_lin_pre_matrix(INFO, DOF, NODER, COORD, BAR, BEAM, CELAS, PRESTRESS)

COLLOC = zeros(2,3);
NODE = zeros(3,3);
N = zeros(12,18);
i = [];
j = [];
v = [];	

% assembly BAR contributions
[iel, jel, vel] = set_DT_pre_mat([1:INFO.nbar], BAR, BAR.R, DOF, COORD, COORD, NODER, NODER, PRESTRESS);

% sparse matrix
i = [i; iel]; j = [j; jel]; v = [v; vel];
	
% assembly BEAM contributions
[iel, jel, vel] = set_DT_mat([1:INFO.nbeam], BEAM, BEAM.R, DOF, COORD, COORD, NODER, NODER);
% sparse matrix
i = [i; iel]; j = [j; jel]; v = [v; vel];
if ~isempty(CELAS.ID)
  icel = [];
  jcel = [];
  vcel = []; 
  for k = 1 : length(CELAS.ID) 
    if CELAS.Node(k,1) == 0 % node 2 linked to ground
      ndof = length(CELAS.DOF(k,2).data);
      ic = double(CELAS.DOF(k,2).data');
      jc = double(CELAS.DOF(k,2).data');
      vc = ones(ndof,1)*CELAS.STiff(k);
      icel = [icel;ic];
      jcel = [jcel;jc];
      vcel = [vcel;vc]; 
    elseif CELAS.Node(k,2) == 0 % node 1 linked to ground
      ndof = length(CELAS.DOF(k,1).data);
      ic = double(CELAS.DOF(k,1).data');
      jc = double(CELAS.DOF(k,1).data');
      vc = ones(ndof,1)*CELAS.STiff(k);
      icel = [icel;ic]; 
      jcel = [jcel;jc];
      vcel = [vcel;vc];
    else
      ndof1 = length(CELAS.DOF(k,1).data);
      ndof2 = length(CELAS.DOF(k,2).data);
      % diagonal term
      ic = [double(CELAS.DOF(k,1).data');double(CELAS.DOF(k,2).data')];
      vc = [ones(ndof1,1)*CELAS.STiff(k)*ndof2;ones(ndof2,1)*CELAS.STiff(k)*ndof1];
      icel = [icel;ic]; jcel = [jcel;ic]; vcel = [vcel;vc];
      for nce = 1 : ndof1
        ic = ones(ndof2,1)*double(CELAS.DOF(k,1).data(nce));
        jc = double(CELAS.DOF(k,2).data');
        vc = -ones(ndof2,1)*CELAS.STiff(k);
        icel = [icel;ic];
        jcel = [jcel;jc];
        vcel = [vcel;vc]; 
      end
      for nce = 1 : ndof2
        ic = ones(ndof1,1)*double(CELAS.DOF(k,2).data(nce));
        jc = double(CELAS.DOF(k,1).data');
        vc = -ones(ndof1,1)*CELAS.STiff(k);
        icel = [icel;ic];
        jcel = [jcel;jc];
        vcel = [vcel;vc];
      end
    end 
  end  
  i = [i;icel]; j = [j;jcel]; v = [v;vcel]; 
end

% compress stiffness matrix
K = sparse(i, j, v, INFO.ndof, INFO.ndof);

end
