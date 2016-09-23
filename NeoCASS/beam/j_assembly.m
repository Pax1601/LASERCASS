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

function [i, j, v] = j_assembly(nbar, BAR, BARR, NODE0, NODER0, COORD, NODER, DOF, FORCES, BARF, SOL, FLW, SCALE, LOADS, INFO)
				
i = [];
j = [];
v = [];

% assembly rotation contribution
[ir, jr, vr] = set_DR_mat(nbar, BAR, BARR, COORD, NODER, DOF, BARF, SOL);
% assembly arm contribution
[ia, ja, va] = set_DA_mat(nbar, BAR, BARR, DOF, NODER, FORCES, SOL);
%[it, jt, vt] = set_DT_mat_nl([1:nbar], BAR, BARR, DOF, NODE0, COORD, NODER0, NODER, SOL);
% assembly internal forces contribution
[it, jt, vt] = set_DT_mat([1:nbar], BAR, BARR, DOF, NODE0, COORD, NODER0, NODER);
% assembly follower forces contributions
[iflw, jflw, vflw] = set_DFLW_mat(LOADS, INFO, FLW, NODER, DOF, SOL, SCALE);

i = [ir; ia; it; iflw];
j = [jr; ja; jt; jflw];
v = [vr;-va; vt; vflw];

end
