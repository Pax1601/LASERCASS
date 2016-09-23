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
%**************************************************************************
%  FFAST Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     062212      2.0     L.Cavagna     Creation
%
%**************************************************************************
% Extracted from get_bar_force_strain
%
function SIGMA = get_auto_internal_load(nbar, Bar, PBar, Mat, Node, NDispl)
%
SIGMA = [];
Nbar = length(nbar);
NMODES = size(NDispl,3);
%
%
sol = zeros(18,1);
SIGMA = zeros(Nbar*12, NMODES);
cont = 0;
for n = nbar
  cont = cont+1;
  n1 = Bar.Conn(n, 1);
  n2 = Bar.Conn(n, 2);
  n3 = Bar.Conn(n, 3);
  % involved dofs
  dof = [Node.DOF(n1,1:6), Node.DOF(n2,1:6),Node.DOF(n3,1:6)];
  %
  R2 = set_R_mat(Bar.R(:,:,4,n), Bar.R(:,:,5,n));
  D = set_D_mat(Bar.D(:,:,1,n), Bar.D(:,:,2,n));
  f1 = Node.R(:,:, n1) * Bar.Offset(n, 1:3)';
  f2 = Node.R(:,:, n2) * Bar.Offset(n, 4:6)';
  f3 = Node.R(:,:, n3) * Bar.Offset(n, 7:9)';
  % node global coords
  c1 = f1 + Node.Coord(n1,:)';
  c2 = f2 + Node.Coord(n2,:)';
  c3 = f3 + Node.Coord(n3,:)';
  % set N matrix
  N = set_N_mat(c1, c2, c3, c1, c2, c3, f1, f2, f3);
  % get internal forces on recovery points
  SIGMA_bar = zeros(12,NMODES);
  for k=1:NMODES
    sol = [NDispl(n1, 1:6, k), NDispl(n2, 1:6, k), NDispl(n3, 1:6, k)];
    SIGMA_bar(:,k) = D * R2' * N * sol';
  end
  offset = (cont-1)*12 +1;
  SIGMA(offset:cont*12,:) = SIGMA_bar;
end

