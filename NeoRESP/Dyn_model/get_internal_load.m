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
%     300910      1.5     L.Travaglini     Creation
%
%**************************************************************************

function [ MF, dofF] = get_internal_load(nbar, Bar, PBar, Mat, Node, FUSE_DP)

% (ci sono dentro PBar, Mat e FUSE_DP in caso si voglia estendere il tutto
% ai margini di sicurezza (difficile fare matrice per vettore (soluzione) a
% quel punto)

% nbar --> id of Bar selected
% SOL  --> solution vector n_node selected x ntime

% dofF --> dof involved from nbar
% MF -->  Matrix length(nbar)*4*length(ndofF)

Nbar = length(nbar);

% MF = zeros(Nbar*4*6,Nbar*3*6);

row = zeros(1,6*4*18*Nbar);
col = row;
val = row;

dofF = zeros(Nbar*3*6,1);
cont1 = 0;
cont2 = 0;
cont3 = 0;

for n = nbar
    
    
    n1 = Bar.Conn(n, 1);
    n2 = Bar.Conn(n, 2);
    n3 = Bar.Conn(n, 3);
    % involved dofs
    dof = [Node.DOF(n1,1:6), Node.DOF(n2,1:6),Node.DOF(n3,1:6)];
    dofF(cont2+1:cont2+18) = dof';
    
%     R = set_R_matT(Bar.R(:,:,1,n), Bar.R(:,:,2,n), Bar.R(:,:,3,n));
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
    COLLOC = interp_colloc_pos(c1, c2, c3);
    
    NODE = [Node.Coord(n1, :); Node.Coord(n2, :); Node.Coord(n3, :)];
    
    A = set_A_mat(COLLOC, NODE);
    %
    N = set_N_mat(c1, c2, c3, c1, c2, c3, f1, f2, f3);
    % get internal forces on recovery points
    Fr = D * R2' * N;
    % get internal forces on boredr points
    Fb = (A([1:6,13:18],:) * D) *R2' * N;
    %     Fb = -R'*Bar.KIL([1:6,13:18],:,n);
    
    Ft = [Fb(1:6,:);Fr(1:6,:);Fr(7:end,:);Fb(7:end,:)];
    
    for i = 1 : 24
        row(cont3+1:cont3+18) = cont1+i;
        col(cont3+1:cont3+18) = cont2+1:cont2+18;
        val(cont3+1:cont3+18) = Ft(i,:);
        cont3 = cont3+18;
    end
%     MF(cont1+1:cont1+6,cont2+1:cont2+18) = Fb(1:6,:);
%     cont1 = cont1+6;
%     MF(cont1+1:cont1+6,cont2+1:cont2+18) = Fr(1:6,:);
%     cont1 = cont1+6;
%     MF(cont1+1:cont1+6,cont2+1:cont2+18) = Fr(7:end,:);
%     cont1 = cont1+6;
%     MF(cont1+1:cont1+6,cont2+1:cont2+18) = Fb(7:end,:);
    cont1 = cont1+24;
    
    cont2 = cont2+18;
    
end

MF = sparse(row,col,val,Nbar*4*6,Nbar*3*6);


