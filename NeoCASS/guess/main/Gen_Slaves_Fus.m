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
%
%   Modified 4-11-2009 by A.Scotti
%   Fuselage is split into three parts
%   Slave nodes for Nose and Tail cone are generated from a first spline, 
%   The remaining part of fuselage is generated using a dedicated spline,
%   relying on three different data: fuselage ver and hor thickness
%   evaluated at nose cone end, mid fuselage and tail cone beginning.
%--------------------------------------------------------------------------------------------------------------------------------
% Modified by Travaglini 19/11/2009, The previous change is removed and
% slave nodes are generated with only one linear interpolation
function [stick] = Gen_Slaves_Fus(stick, geo)

% Identification number for slave nodes
IDslaveF = 9999;

% Vertical and horizontal dimension of the section over nodes
deltaZ = interp1(geo.fus.thick_dom, geo.fus.thick_ver, stick.nodes.fuse(1,:),'linear');
deltaY = interp1(geo.fus.thick_dom, geo.fus.thick_hor, stick.nodes.fuse(1,:),'linear');

% deltaZ_fus = interp1([geo.fus.lengthN; geo.fus.lengthN+(geo.fus.lengthMID)/2;geo.fus.bodl-geo.fus.lengthAB],...
%                      [interp1(geo.fus.thick_dom, geo.fus.thick_ver, geo.fus.lengthN,'linear');interp1(geo.fus.thick_dom, geo.fus.thick_ver, geo.fus.lengthN+(geo.fus.lengthMID)/2,'linear');interp1(geo.fus.thick_dom, geo.fus.thick_hor, geo.fus.bodl-geo.fus.lengthAB,'linear')],geo.fus.x_nodes,'linear');
% deltaY_fus = interp1([geo.fus.lengthN; geo.fus.lengthN+(geo.fus.lengthMID)/2;geo.fus.bodl-geo.fus.lengthAB],[interp1(geo.fus.thick_dom, geo.fus.thick_hor, geo.fus.lengthN,'linear');interp1(geo.fus.thick_dom, geo.fus.thick_hor, geo.fus.lengthN+(geo.fus.lengthMID)/2,'linear');interp1(geo.fus.thick_dom, geo.fus.thick_hor, geo.fus.bodl-geo.fus.lengthAB,'linear')],geo.fus.x_nodes,'linear');

% Define for each node, 4 slaves
for i = 1:length(stick.ID.fuse)

    % Update index for slave nodes
    t = (i-1)*4 + 1;

    % 1st node, horizontal section, y-positive

    stick.slaves.nodes.fuse(1,t) = stick.nodes.fuse(1,i);
%     if (stick.slaves.nodes.fuse(1,t)>=geo.fus.lengthN & stick.slaves.nodes.fuse(1,t)<=(geo.fus.bodl-geo.fus.lengthAB))
%         %       stick.slaves.nodes.fuse(2,t) = stick.nodes.fuse(2,i) + geo.fus.R;
%         stick.slaves.nodes.fuse(2,t) = stick.nodes.fuse(2,i) + deltaY_fus(i);
%     else
        stick.slaves.nodes.fuse(2,t) = stick.nodes.fuse(2,i) + deltaY(i);
%     end
    stick.slaves.nodes.fuse(3,t) = stick.nodes.fuse(3,i);
    stick.slaves.ID.fuse(t,1) = IDslaveF + t;

    % 2nd node, vertical section, z-positive

    stick.slaves.nodes.fuse(1,t+1) = stick.nodes.fuse(1,i);
    stick.slaves.nodes.fuse(2,t+1) = stick.nodes.fuse(2,i);
%     if (stick.slaves.nodes.fuse(1,t)>=geo.fus.lengthN & stick.slaves.nodes.fuse(1,t)<=(geo.fus.bodl-geo.fus.lengthAB))
%         %      stick.slaves.nodes.fuse(3,t+1) = stick.nodes.fuse(3,i) + geo.fus.R;
%         stick.slaves.nodes.fuse(3,t+1) = stick.nodes.fuse(3,i) + deltaZ_fus(i);
%     else
        stick.slaves.nodes.fuse(3,t+1) = stick.nodes.fuse(3,i) + deltaZ(i);
%     end
    stick.slaves.ID.fuse(t+1,1) = IDslaveF + t + 1;

    % 3th node, horizontal section, y-negative

    stick.slaves.nodes.fuse(1,t+2) = stick.nodes.fuse(1,i);
%     if (stick.slaves.nodes.fuse(1,t)>=geo.fus.lengthN & stick.slaves.nodes.fuse(1,t)<=(geo.fus.bodl-geo.fus.lengthAB))
%         %stick.slaves.nodes.fuse(2,t+2) = stick.nodes.fuse(2,i) - geo.fus.R;
%         stick.slaves.nodes.fuse(2,t+2) = stick.nodes.fuse(2,i) - deltaY_fus(i);
%     else
        stick.slaves.nodes.fuse(2,t+2) = stick.nodes.fuse(2,i) - deltaY(i);
%     end
    stick.slaves.nodes.fuse(3,t+2) = stick.nodes.fuse(3,i);
    stick.slaves.ID.fuse(t+2,1) = IDslaveF + t + 2;

    % 4th node, vertical section, z-positive

    stick.slaves.nodes.fuse(1,t+3) = stick.nodes.fuse(1,i);
    stick.slaves.nodes.fuse(2,t+3) = stick.nodes.fuse(2,i);
%     if (stick.slaves.nodes.fuse(1,t)>=geo.fus.lengthN & stick.slaves.nodes.fuse(1,t)<=(geo.fus.bodl-geo.fus.lengthAB))
%         %stick.slaves.nodes.fuse(3,t+3) = stick.nodes.fuse(3,i) - geo.fus.R;
%         stick.slaves.nodes.fuse(3,t+3) = stick.nodes.fuse(3,i) - deltaZ_fus(i);
%     else
        stick.slaves.nodes.fuse(3,t+3) = stick.nodes.fuse(3,i) - deltaZ(i);
%     end
    stick.slaves.ID.fuse(t+3,1) = IDslaveF + t + 3;

    % offset to get absolut coordinates for slaves nodes
    for j = t:t+3
        stick.slaves.nodes.fuse(:,j) = stick.slaves.nodes.fuse(:,j) + stick.OFFSET.fuse(:,1);
    end

end
