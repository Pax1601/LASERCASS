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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%  This function defines N point on domain defined by P (3 x n matrix)
%
%
% Called by:    Stick_Points_Fuse.m, Stick_Points_Wing.m,
%               Stick_Points_Vert.m, Stick_Points_Hori.m
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     091026      1.3.9    Travaglini       Creation
%
% Now the function defines exactly N point determining firstly the principal
% direction of element (for example the principal direction of fuselage is x)
%*******************************************************************************
function node = DeltaElem(P,N)

if N+1 <= size(P,2)
    node = P;
else
    
    Diff = max(diff(P,1,2),[],2);
    [~, ind] = max(Diff);
    Node = zeros(3,N+1);
    Node(ind,:) = linspace(P(ind,1),P(ind,end),N+1);
    switch ind
        case 1
            Node(2,:) = interp1(P(1,:),P(2,:),Node(1,:),'linear');
            Node(3,:) = interp1(P(1,:),P(3,:),Node(1,:),'linear');
        case 2
            Node(1,:) = interp1(P(2,:),P(1,:),Node(2,:),'linear');
            Node(3,:) = interp1(P(2,:),P(3,:),Node(2,:),'linear');
        case 3
            Node(1,:) = interp1(P(3,:),P(1,:),Node(3,:),'linear');
            Node(2,:) = interp1(P(3,:),P(2,:),Node(3,:),'linear');
    end
      
    % Adding P points and delete the twin (or those too near)
%     dind = 0.5*(Node(ind,2)-Node(ind,1));
    node = [Node, P];
    
    % Delete coincident nodes
    [~, Y] = sort(node(ind,:), 'ascend');
    node = node(:,Y);
    node = [node(:,diff(node(ind,:))~=0),node(:,end)];
    N2 = size(node,2);
    indP = zeros(size(node,2),1);
    Min = zeros(size(P,2),1);
    IndNP = zeros(size(P,2),1);
    for i = 1 : size(node,2)
        if isempty( find(node(ind,i)==P(ind,:),1))
            indP(i) = 1;
        else
            indP(i) = 0;
        end
    end
    
    if N2 > N+1
        for i = 1 : N2-(N+1)
            for j = 1 : size(P,2)
                [Min(j),IndNP(j)] = min(abs( node(ind,indP==1 )-P(ind,j)));   
            end
            [dummy,indmin] = min(Min);
            A = find(indP==1);
            node(:,A(IndNP(indmin))) = P(:,indmin);
            indP(A(IndNP(indmin))) =0;
        end
    end
    node = [node(:,diff(node(ind,:))~=0),node(:,end)];
    
    %         Diff = diff(node(ind,:));
    %         ind2 = find(abs(Diff)>=dind);
    %         node = node(:,ind2+1);
    % for i = 1 : size(P,2)
    %     pp = meshgrid(P(:,i),1:N+1)';
    %     Diff2 = Node-pp;
    %     [dummy,ind2] = min( ( Diff2(1,:).^2 + Diff2(2,:).^2 + Diff2(3,:).^2) );
    %     node(:,ind2) = P(:,i);
    % end
end
