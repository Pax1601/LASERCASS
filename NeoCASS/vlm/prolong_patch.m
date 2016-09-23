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
%*******************************************************************************
%  SimSAC Project
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
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     221112      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function [P2, C] = prolong_patch(STARTP, c, b, T, SW, TW, dihed, SPANADD)
%
%   DESCRIPTION: given a patch, prolong its inboard boundary
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                STARTP         real       patch apex             
%                c,b,T,SW,TW,   real       patch chord, span,taper,sweep
%                dihed                     twist at root and dihedral            
%                SPANADD        real       length of patch to add
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                P2             real       apex of the new patch
%                C              real       root chord of the new patch
%    REFERENCES:
%
%*******************************************************************************

function [P2, C] = prolong_patch(STARTP, c, b, T, SW, TW, dihed, SPANADD)
% patch origin
ox = STARTP(1);
oy = STARTP(2);
oz = STARTP(3);
% twist contribution
lem(1)=0.25*c;
lem(2)=0.25*T*c;
lem(3)=-0.75*T*c;
lem(4)=-0.75*c;
DX =[(1-cos(TW))*cos(SW) (1-cos(TW))*cos(SW) (1-cos(TW))*cos(SW) (1-cos(TW))*cos(SW)].*lem;
DY =-[sin(TW)*sin(dihed)*cos(SW) sin(TW)*sin(dihed)*cos(SW) sin(TW)*sin(dihed)*cos(SW) sin(TW)*sin(dihed)*cos(SW)].*lem;
DZ =[sin(TW)*cos(dihed) sin(TW)*cos(dihed) sin(TW)*cos(dihed) sin(TW)*cos(dihed)].*lem;
% patch nodes
wingx =[0 0.25*c+b*tan(SW)-0.25*T*c 0.25*c+b*tan(SW)+0.75*T*c c] + ox + DX;
wingy =[0 b*cos(dihed) b*cos(dihed) 0] + oy + DY;
wingz =[0 b*sin(dihed) b*sin(dihed) 0] + oz + DZ;
NODES= [wingx', wingy', wingz'];
%
CTIP  = norm(NODES(2,:)-NODES(3,:));
CROOT = norm(NODES(1,:)-NODES(4,:));
%
C4R = 0.25*(NODES(4,:) - NODES(1,:)) + NODES(1,:);
C4T = 0.25*(NODES(3,:) - NODES(2,:)) + NODES(2,:);

SPAN = norm(C4T-C4R);
Y = (C4R - C4T)'; Y = Y ./norm(Y);
X = [1 0 0]';
Z = crossm(X)*Y; Z = Z ./norm(Z);
X = crossm(Y)*Z; X = X ./norm(X);
Rmat = [X, Y, Z]; % from local to global

P1 = (C4R - C4T)';
P1R = Rmat'*P1;
P2R = P1R;
P2R(2) = P2R(2) + SPANADD/cos(SW);

SPANTOT = P2R(2);

M = (CROOT-CTIP)/P1R(2);
C = CTIP + M * SPANTOT;
P2 = Rmat*P2R;
P2(1) = P2(1) - C/4;
P2 = P2 + C4T';
end