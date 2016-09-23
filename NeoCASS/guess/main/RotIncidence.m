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
%--------------------------------------------------------------------------------------------------
% 
% CR: root chord
% CT: tip chord
% incR: root incidence
% incT: tip incidence
% QCR: QC point coordinate at root
% QCT: QC point coordinate at tip
% 
%
% Called by:    STICKwing.m
% 
% Calls:        none
% 
% <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [ptos] = RotIncidence(CR, CT, incR, incT, QC)

% Node 1 at LE root
X1 = QC(1,1) - 1/2*CR*cos(incR*pi/180);
Y1 = QC(2,1);
Z1 = QC(3,1) + 1/2*CR*sin(incR*pi/180);    
% % X1 = QC(1,1) - 1/4*CR*cos(incR*pi/180);
% % Y1 = QC(2,1);
% % Z1 = QC(3,1) + 1/4*CR*sin(incR*pi/180);    

% Node 2 at LE tip
X2 = QC(1,2) - 1/2*CT*cos(incT*pi/180);
Y2 = QC(2,2);
Z2 = QC(3,2) + 1/2*CT*sin(incT*pi/180);
% % X2 = QC(1,2) - 1/4*CT*cos(incT*pi/180);
% % Y2 = QC(2,2);
% % Z2 = QC(3,2) + 1/4*CT*sin(incT*pi/180);

% Node 3 at TE tip
X3 = QC(1,2) + 1/2*CT*cos(incT*pi/180);
Y3 = QC(2,2);
Z3 = QC(3,2) - 1/2*CT*sin(incT*pi/180);    
% % X3 = QC(1,2) + 3/4*CT*cos(incT*pi/180);
% % Y3 = QC(2,2);
% % Z3 = QC(3,2) - 3/4*CT*sin(incT*pi/180);    

% Node 4 at TE tip
X4 = QC(1,1) + 1/2*CR*cos(incR*pi/180);
Y4 = QC(2,1);
Z4 = QC(3,1) - 1/2*CR*sin(incR*pi/180);
% % X4 = QC(1,1) + 3/4*CR*cos(incR*pi/180);
% % Y4 = QC(2,1);
% % Z4 = QC(3,1) - 3/4*CR*sin(incR*pi/180);

% Nodes to define inboard panel
ptos = [X1, X2, X3, X4;...
        Y1, Y2, Y3, Y4;...
        Z1, Z2, Z3, Z4];

         
         