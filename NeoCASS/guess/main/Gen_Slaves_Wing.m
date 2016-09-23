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
% Called by:    writeRB02file.m
% 
% Calls:        
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------------------------------------
function [stick] = Gen_Slaves_Wing(stick, geo)

% Identification number for slave nodes
IDslaveW = 19999;

dihedral = zeros(geo.wing.leny, 1);
for j = 1:length(geo.wing.CAERO1.n)
    dihedral(geo.wing.index(j):geo.wing.index(j+1),1) = geo.wing.CAERO1.dihedral(j);
end
Y  = [0; cumsum(stick.wing.Lbeam_thick)];
Yc = [0; cumsum(stick.wing.Lbeam)];
deltaX     = interp1(Y, geo.wing.Zs,  Yc, 'linear', 'extrap');
deltaXX    = interp1(Y, geo.wing.rs,  Yc, 'linear', 'extrap');
deltaZ     = interp1(Y, geo.wing.tbs,  Yc, 'linear', 'extrap');
deltaAlpha = interp1(Y, geo.wing.incidence,  Yc, 'linear', 'extrap');
deltaInc   = interp1(Y, dihedral,  Yc, 'linear', 'extrap');
%
deltaX = deltaX./2;         % stick model is in the middle
deltaXX = deltaXX./2;
deltaZ = deltaZ./2;         % half thickness to add to top and bottom slave nodes
deltaInc = deltaInc*pi/180; % convert into radiant

% Define for each node, 4 slaves
for i = 1:length(stick.ID.winr)
    
    %---------------------------------------------------------------
    % Constrain the 4 slaves to be within the plane of symmetry
    %---------------------------------------------------------------
    if isequal( stick.nodes.winrC2(2,i), 0 )
        deltaInc(i) = 0;
    end
    
    % Update index for slave nodes
    t = (i-1)*4 + 1;
    
    % 1st node at LE
    n1 = [0; 0; -deltaX(i)];
    if i > 1 & i < length(stick.ID.winr)
        stick.slaves.nodes.winr(:,t) = stick.nodes.winrC2(:,i) + stick.CBAR.winr.R(:,:,i-1)*n1;
    else
        stick.slaves.nodes.winr(1,t) = stick.nodes.winrC2(1,i) - deltaXX(i).*cos(deltaAlpha(i));
        stick.slaves.nodes.winr(2,t) = stick.nodes.winrC2(2,i) - deltaXX(i).*sin(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.winr(3,t) = stick.nodes.winrC2(3,i) + deltaXX(i).*sin(deltaAlpha(i)).*cos(deltaInc(i));        
    end
    stick.slaves.ID.winr(t,1) = IDslaveW + t;
    %
    % 2nd node at the bottom
    n2 = [0; -deltaZ(i); 0];
    if i > 1 & i < length(stick.ID.winr)
        stick.slaves.nodes.winr(:,t+1) = stick.nodes.winrC2(:,i) + stick.CBAR.winr.R(:,:,i-1)*n2;
    else
        stick.slaves.nodes.winr(1,t+1) = stick.nodes.winrC2(1,i) - deltaZ(i).*sin(deltaAlpha(i));
        stick.slaves.nodes.winr(2,t+1) = stick.nodes.winrC2(2,i) + deltaZ(i).*cos(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.winr(3,t+1) = stick.nodes.winrC2(3,i) - deltaZ(i).*cos(deltaAlpha(i)).*cos(deltaInc(i));
    end
    stick.slaves.ID.winr(t+1,1) = IDslaveW + t + 1;
    %
    % 3th node at TE
    n3 = [0; 0; deltaX(i)];
    if i > 1 & i < length(stick.ID.winr)
        stick.slaves.nodes.winr(:,t+2) = stick.nodes.winrC2(:,i) + stick.CBAR.winr.R(:,:,i-1)*n3;
    else
        stick.slaves.nodes.winr(1,t+2) = stick.nodes.winrC2(1,i) + deltaXX(i).*cos(deltaAlpha(i));
        stick.slaves.nodes.winr(2,t+2) = stick.nodes.winrC2(2,i) + deltaXX(i).*sin(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.winr(3,t+2) = stick.nodes.winrC2(3,i) - deltaXX(i).*sin(deltaAlpha(i)).*cos(deltaInc(i));        
    end
    stick.slaves.ID.winr(t+2,1) = IDslaveW + t + 2;    
    %
    % 4th node at the top
    n4 = [0; deltaZ(i); 0];
    if i > 1 & i < length(stick.ID.winr)
        stick.slaves.nodes.winr(:,t+3) = stick.nodes.winrC2(:,i) + stick.CBAR.winr.R(:,:,i-1)*n4;
    else
        stick.slaves.nodes.winr(1,t+3) = stick.nodes.winrC2(1,i) + deltaZ(i).*sin(deltaAlpha(i));
        stick.slaves.nodes.winr(2,t+3) = stick.nodes.winrC2(2,i) - deltaZ(i).*cos(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.winr(3,t+3) = stick.nodes.winrC2(3,i) + deltaZ(i).*cos(deltaAlpha(i)).*cos(deltaInc(i));
    end
    stick.slaves.ID.winr(t+3,1) = IDslaveW + t + 3;
    
end
%
if isequal(stick.model.winl, 1)
    
    % Slave nodes symmetric w.r.t. XZ plane
    stick.slaves.nodes.winl = stick.slaves.nodes.winr;
    stick.slaves.nodes.winl(2,:) = -stick.slaves.nodes.winl(2,:);
    stick.slaves.ID.winl = [stick.slaves.ID.winr(1:4)', stick.slaves.ID.winr(end)+1:stick.slaves.ID.winr(end)+length(stick.slaves.ID.winr)-4]';

end
