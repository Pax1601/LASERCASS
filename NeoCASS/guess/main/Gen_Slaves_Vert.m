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
%--------------------------------------------------------------------------------------------------------------------------------
% It generates 4 slave nodes per each master node
% 
% 
% Called by:    writeRB02file.m
% 
% Calls:        
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------------------------------------
function [stick] = Gen_Slaves_Vert(stick, geo)
% Modified by Travaglini 19/11/2009 to adapt it to change about link
% between vtail and htail

% Identification number for slave nodes
IDslaveV = 29999;
% Interpolation domain
% span = [geo.vtail.span_inboard; geo.vtail.span_outboard];
% bS = [geo.vtail.bS_inboard; geo.vtail.bS_outboard];
% [Yvert, indx] = nodeinterp_vert(geo.vtail.zLE, span, bS, stick.nodes.vert);
% strind = indx(1);
%$$$$$$$$$
% stick.vtail.Yvert = Yvert;
% stick.vtail.strind = strind;
%$$$$$$$$$

% if abs(Yvert(end) - geo.vtail.bS) > 1e-3
%     warning('***** Interpolation domain does not coincide with previous values from geometry module *****');
% end
         
% Dimensions of aerodynamic box around nodes
Y  = [0; cumsum(stick.vtail.Lbeam_thick)];
Yc = [0; cumsum(stick.vtail.Lbeam)];
deltaX = interp1(Y, geo.vtail.Zs,  Yc, 'linear', 'extrap');
deltaY = interp1(Y, geo.vtail.tbs,  Yc, 'linear', 'extrap');
%
deltaX = deltaX./2;     % stick model is in the middle
deltaY = deltaY./2;     % half thickness to add to top and bottom slave nodes   

% Define for each node, 4 slaves
for i = 1: length(stick.ID.vert)
    
    % Update index for slave nodes
    t = (i-1)*4 + 1;
   
    
    % 1st node at LE
    n1 = [0; 0; -deltaX(i)];
    if i > 1 && i < length(stick.ID.vert)
        stick.slaves.nodes.vert(:,t) = stick.nodes.vert(:,i) + stick.CBAR.vert.R(:,:,i-1)*n1;
    else
        stick.slaves.nodes.vert(1,t) = stick.nodes.vert(1,i) - deltaX(i);
        stick.slaves.nodes.vert(2,t) = stick.nodes.vert(2,i);
        stick.slaves.nodes.vert(3,t) = stick.nodes.vert(3,i);
    end
    stick.slaves.ID.vert(t,1) = IDslaveV + t;
    % 2nd node on negative Y semi-plane
    n2 = [0; -deltaY(i); 0];
    if i > 1 && i < length(stick.ID.vert)
        stick.slaves.nodes.vert(:,t+1) = stick.nodes.vert(:,i) + stick.CBAR.vert.R(:,:,i-1)*n2;
    else
        stick.slaves.nodes.vert(1,t+1) = stick.nodes.vert(1,i);
        stick.slaves.nodes.vert(2,t+1) = stick.nodes.vert(2,i) - deltaY(i);
        stick.slaves.nodes.vert(3,t+1) = stick.nodes.vert(3,i);
    end
    stick.slaves.ID.vert(t+1,1) = IDslaveV + t + 1;
    % 3th node at TE
    n3 = [0; 0; deltaX(i)];
    if i > 1 && i < (length(stick.ID.vert))
        stick.slaves.nodes.vert(:,t+2) = stick.nodes.vert(:,i) + stick.CBAR.vert.R(:,:,i-1)*n3;
    else
        stick.slaves.nodes.vert(1,t+2) = stick.nodes.vert(1,i) + deltaX(i);
        stick.slaves.nodes.vert(2,t+2) = stick.nodes.vert(2,i);
        stick.slaves.nodes.vert(3,t+2) = stick.nodes.vert(3,i);
    end
    stick.slaves.ID.vert(t+2,1) = IDslaveV + t + 2;
    % 4th node on positive Y semi-plane
    n4 = [0; deltaY(i); 0];
    if i > 1 && i < (length(stick.ID.vert) )
        stick.slaves.nodes.vert(:,t+3) = stick.nodes.vert(:,i) + stick.CBAR.vert.R(:,:,i-1)*n4;
    else
        stick.slaves.nodes.vert(1,t+3) = stick.nodes.vert(1,i);
        stick.slaves.nodes.vert(2,t+3) = stick.nodes.vert(2,i) + deltaY(i);
        stick.slaves.nodes.vert(3,t+3) = stick.nodes.vert(3,i);
    end
    stick.slaves.ID.vert(t+3,1) = IDslaveV + t + 3;
    
end

% % HT is under the VT root aerodynamic chord
% if isempty(stick.ID.horr) ~= 1
% 
%     if geo.htail.zLE < geo.vtail.zLE
%         % find nodes under the aerodynamic VT chord
%         strm = find(stick.nodes.vert(3,:)<geo.vtail.zLE & stick.nodes.vert(3,:)>=stick.ptos.horr(3,1));
% 
%         if isempty(strm)
%             % nothing
%         else
% 
%             IDslaveVN = stick.slaves.ID.vert(end);
% 
%             for i = 1:length(strm)
% 
%                 % Update index for slave nodes
%                 t = (i-1)*4 + 1;
%                 % 1st node at LE
%                 A(1,t) = stick.nodes.vert(1,strm(i)) - deltaX(1);
%                 A(2,t) = stick.nodes.vert(2,strm(i));
%                 A(3,t) = stick.nodes.vert(3,strm(i));
%                 B(t,1) = IDslaveVN + t;
%                 % 2nd node on negative Y semi-plane
%                 A(1,t+1) = stick.nodes.vert(1,strm(i));
%                 A(2,t+1) = stick.nodes.vert(2,strm(i)) - deltaY(1);
%                 A(3,t+1) = stick.nodes.vert(3,strm(i));
%                 B(t+1,1) = IDslaveVN + t + 1;
%                 % 3th node at TE
%                 A(1,t+2) = stick.nodes.vert(1,strm(i)) + deltaX(1);
%                 A(2,t+2) = stick.nodes.vert(2,strm(i));
%                 A(3,t+2) = stick.nodes.vert(3,strm(i));
%                 B(t+2,1) = IDslaveVN + t + 2;
%                 % 4th node on positive Y semi-plane
%                 A(1,t+3) = stick.nodes.vert(1,strm(i));
%                 A(2,t+3) = stick.nodes.vert(2,strm(i)) + deltaY(1);
%                 A(3,t+3) = stick.nodes.vert(3,strm(i));
%                 B(t+3,1) = IDslaveVN + t + 3;
% 
%             end
% 
%             stick.slaves.nodes.vert = [A, stick.slaves.nodes.vert];
%             stick.slaves.ID.vert = [B; stick.slaves.ID.vert];
% 
%         end
% 
%     end
% 
% end
% 
