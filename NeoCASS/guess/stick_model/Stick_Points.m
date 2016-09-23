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
% The global reference system is located on fuselage nose, x-axis is along
% fuselage length, z-axis is going upwards
% 
% 
% Called by:    Stick_Model.m
% 
% Calls:        Stick_Points_Fuse.m, Stick_Points_Wing.m, Stick_Points_Vert.m, Stick_Points_Hori.m
%
%   <andreadr@kth.se>
% Modified by Travaglini 22/october/2009
%--------------------------------------------------------------------------------------------------
function [stick, geo] = Stick_Points( aircraft, stick, geo)

if aircraft.fuselage.present
    [stick] = Stick_Points_Fuse(aircraft, stick, geo);
end

if isfield(aircraft,'Tailbooms')
   if aircraft.Tailbooms.present
       [stick] = Stick_Points_Tbooms( stick, geo); 
   end
end
%
if aircraft.wing1.present
    [stick, geo] = Stick_Points_Wing(aircraft, stick, geo);
end
%
if aircraft.Vertical_tail.present
    [stick, geo] = Stick_Points_Vert(aircraft, stick, geo);
end
%

if aircraft.Horizontal_tail.present
   [stick, geo] = Stick_Points_Hori(aircraft, stick, geo);
end
% if  isequal(stick.model.canr, 0)
% questo di sotto va bene solo per il momento
% if aircraft.Vertical_tail.Twin_tail
%     delta_vtail=abs(stick.ptos.horrC2(:,end)-stick.ptos.vert(:,1));
%     for i=1:length(stick.ptos.vert)
%         stick.ptos.vert(:,i)=stick.ptos.vert(:,i)+delta_vtail;
%     end
%     for i=1:length(stick.ptospanel.vert)
%         stick.ptospanel.vert(:,i)=stick.ptospanel.vert(:,i)+delta_vtail;
%     end
%     
%     %         stick.ptos.vert = [stick.ptos.horrC2(:,end), stick.ptos.vert];
%     
% end

        
        
        
% end
%

if aircraft.Canard.present 
   [stick, geo] = Stick_Points_Canard(stick, geo);
end

if aircraft.wing2.present
   [stick, geo] = Stick_Points_Wing2(aircraft, stick, geo);
end

%--------------------------------------------------------------------------
% Sort by ascending order points added in fuselage and vertical tail
% 
% if isequal(stick.model.fuse, 1)
%     [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
%     % Sort points
%     stick.ptos.fuse(1,:) = Y;
%     stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
%     stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
% end
% %
% if isequal(stick.model.vert, 1)
%     [Y, J] = sort(stick.ptos.vert(3,:), 'ascend');
%     % Sort points
%     stick.ptos.vert(1,:) = stick.ptos.vert(1,J);
%     stick.ptos.vert(2,:) = stick.ptos.vert(2,J);
%     stick.ptos.vert(3,:) = Y;
% end
%
%--------------------------------------------------------------------------


