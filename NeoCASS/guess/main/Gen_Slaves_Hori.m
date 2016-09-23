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
% 
% Called by:    writeRB02file.m
% 
% Calls:        
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------------------------------------
% Modified By Travaglini 19/11/2009 (only clean)
function [stick] = Gen_Slaves_Hori(stick, geo)

% Identification number for slave nodes
IDslaveH = 39999;

dihedral = zeros(geo.htail.leny, 1);
for j = 1:length(geo.htail.CAERO1.n)
    dihedral(geo.htail.index(j):geo.htail.index(j+1),1) = geo.htail.CAERO1.dihedral(j);
end

% Dimensions of aerodynamic box around nodes
Y  = [0; cumsum(stick.htail.Lbeam_thick)];
Yc = [0; cumsum(stick.htail.Lbeam)];
deltaX     = interp1(Y, geo.htail.Zs,  Yc, 'linear', 'extrap');
deltaXX    = interp1(Y, geo.htail.rs,  Yc, 'linear', 'extrap');
deltaZ     = interp1(Y, geo.htail.tbs,  Yc, 'linear', 'extrap');
deltaAlpha = interp1(Y, geo.htail.incidence,  Yc, 'linear', 'extrap');
deltaInc   = interp1(Y, dihedral,  Yc, 'linear', 'extrap');
%
deltaX = deltaX./2;         % stick model is in the middle
deltaXX = deltaXX./2;
deltaZ = deltaZ./2;         % half thickness to add to top and bottom slave nodes   
deltaInc = deltaInc*pi/180; % convert into radiant

% Define for each node, 4 slaves
for i = 1:length(stick.ID.horr)
    
    %---------------------------------------------------------------
    % Constrain the 4 slaves at be within the plane of symmetry
    %---------------------------------------------------------------
    if isequal( stick.nodes.horrC2(2,i), 0 )
        deltaInc(i) = 0;
    end
    
    % Update index for slave nodes
    t = (i-1)*4 + 1;
    % 1st node at LE
	n1 = [0; 0; -deltaX(i)];
    if i > 1 && i < length(stick.ID.horr)
        stick.slaves.nodes.horr(:,t) = stick.nodes.horrC2(:,i) + stick.CBAR.horr.R(:,:,i-1)*n1;
    else
        stick.slaves.nodes.horr(1,t) = stick.nodes.horrC2(1,i) - deltaXX(i).*cos(deltaAlpha(i));
        stick.slaves.nodes.horr(2,t) = stick.nodes.horrC2(2,i) - deltaXX(i).*sin(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.horr(3,t) = stick.nodes.horrC2(3,i) + deltaXX(i).*sin(deltaAlpha(i)).*cos(deltaInc(i));
    end
    stick.slaves.ID.horr(t,1) = IDslaveH + t;
    % 2nd node at the bottom
    n2 = [0; -deltaZ(i); 0];
    if i > 1 && i < length(stick.ID.horr)
        stick.slaves.nodes.horr(:,t+1) = stick.nodes.horrC2(:,i) + stick.CBAR.horr.R(:,:,i-1)*n2;
    else
        stick.slaves.nodes.horr(1,t+1) = stick.nodes.horrC2(1,i) - deltaZ(i).*sin(deltaAlpha(i));
        stick.slaves.nodes.horr(2,t+1) = stick.nodes.horrC2(2,i) + deltaZ(i).*cos(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.horr(3,t+1) = stick.nodes.horrC2(3,i) - deltaZ(i).*cos(deltaAlpha(i)).*cos(deltaInc(i));
    end
    stick.slaves.ID.horr(t+1,1) = IDslaveH + t + 1;
    % 3th node at TE
    n3 = [0; 0; deltaX(i)];
    if i > 1 && i < length(stick.ID.horr)
        stick.slaves.nodes.horr(:,t+2) = stick.nodes.horrC2(:,i) + stick.CBAR.horr.R(:,:,i-1)*n3;
    else
        stick.slaves.nodes.horr(1,t+2) = stick.nodes.horrC2(1,i) + deltaXX(i).*cos(deltaAlpha(i));
        stick.slaves.nodes.horr(2,t+2) = stick.nodes.horrC2(2,i) + deltaXX(i).*sin(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.horr(3,t+2) = stick.nodes.horrC2(3,i) - deltaXX(i).*sin(deltaAlpha(i)).*cos(deltaInc(i));        
    end
    stick.slaves.ID.horr(t+2,1) = IDslaveH + t + 2;
    % 4th node at the top
    n4 = [0; deltaZ(i); 0];
    if i > 1 && i < length(stick.ID.horr)
        stick.slaves.nodes.horr(:,t+3) = stick.nodes.horrC2(:,i) + stick.CBAR.horr.R(:,:,i-1)*n4;
    else
        stick.slaves.nodes.horr(1,t+3) = stick.nodes.horrC2(1,i) + deltaZ(i).*sin(deltaAlpha(i));
        stick.slaves.nodes.horr(2,t+3) = stick.nodes.horrC2(2,i) - deltaZ(i).*cos(deltaAlpha(i)).*sin(deltaInc(i));
        stick.slaves.nodes.horr(3,t+3) = stick.nodes.horrC2(3,i) + deltaZ(i).*cos(deltaAlpha(i)).*cos(deltaInc(i));
    end
    stick.slaves.ID.horr(t+3,1) = IDslaveH + t + 3;
     
end

% Update OFFSET for slave nodes
stick.slaves.nodes.horr(1,:) = stick.slaves.nodes.horr(1,:) + stick.OFFSET.hori(1,1);
stick.slaves.nodes.horr(2,:) = stick.slaves.nodes.horr(2,:) + stick.OFFSET.hori(2,1);
stick.slaves.nodes.horr(3,:) = stick.slaves.nodes.horr(3,:) + stick.OFFSET.hori(3,1);

%
if isequal(stick.model.horl, 1)

    % Slave nodes symmetric w.r.t. XZ plane
    stick.slaves.nodes.horl = stick.slaves.nodes.horr;
    stick.slaves.nodes.horl(2,:) = -stick.slaves.nodes.horl(2,:);
    stick.slaves.ID.horl = [stick.slaves.ID.horr(1:4)', stick.slaves.ID.horr(end)+1:stick.slaves.ID.horr(end)+length(stick.slaves.ID.horr)-4]';

end
