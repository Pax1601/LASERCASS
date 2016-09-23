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

function [D, Kll, Klv, Kvv, Kvl, rdof, ldof, EPS] = get_suport_shapes(K, NODE, SUPORT, MODE_AMPL)
%
  nnodes = size(SUPORT,1);
  index = [];
  for i=1:nnodes
    m = find(SUPORT(i,1) == [NODE.ID]);
    dof = num2str(SUPORT(i,2));
    for k=1:length(dof)
      index = [index, NODE.DOF(m, str2num(dof(k)))];
    end
  end
  nc = size(K,2);
  col = setdiff([1:nc], index);
  %
  Kll = K(col, col);
  Klv = K(col, index);
  Kvv = K(index, index);
  Kvl = K(index, col);
  %
%   D = -inv(Kll)*Klv;
  D = -Kll\Klv;
  %
  V     = zeros(length(index), 1);
  Vloc  = zeros(length(col),1);
  RMODE = zeros(nc,length(index));
  EPS   = zeros(length(index),1);
  for n=1:length(index)
    V(:) = 0.0;
    V(n) = MODE_AMPL;
    RMODE(col, n) = D*V;
    RMODE(index(n),n) = MODE_AMPL;
    EPS(n) = RMODE(:,n)' * K * RMODE(:,n);
  end
%
  rdof = index;
  ldof = col;
end
