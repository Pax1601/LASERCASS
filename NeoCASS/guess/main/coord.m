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

function [aero_forces, distances]=coord(forces,stick, aircraft, geo, pdcylin)
% Modified by Travaglini 19/11/2009, find some errors on distribution loads

dist.fuse=[];
dist.wing=[];
dist.vtail=[];
dist.htail=[];
dist.canard=[];


distances.fuse=[];
distances.wing=[];
distances.vtail=[];
distances.htail=[];
distances.canard=[];


% geo.wing.leny =geo.wing.leny-pdcylin.stick.nwing_carryth;

% geo.vtail.leny=geo.vtail.leny;


% FUSELAGE

aero_forces.fuse=forces(1:pdcylin.stick.nfuse+1,:);

% WING1
% left
aero_forces.wing_sx = forces( (pdcylin.stick.nfuse+1) +1 : (pdcylin.stick.nfuse+1)+ size(stick.nodes.winrC2_thick,2),:);
%right
aero_forces.wing_dx =[forces( (pdcylin.stick.nfuse+1) +1,:); forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)+1:...
                              (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2-1,:)];

% VERTICAL TAIL

aero_forces.vtail = forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2:...
    (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2-1 + size(stick.nodes.vert_thick,2) ,:);

if isequal(aircraft.Vertical_tail.Twin_tail, 1)   
    % TWIN TAIL
    aero_forces.vtail2 = forces((pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2):...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2-1 + size(stick.nodes.vert_thick,2)*2,:);
    % HORIZONTAL TAIL
    
    if isequal(stick.model.horr,1)
%         geo.htail.leny = geo.htail.leny-pdcylin.stick.nhtail_carryth;
        aero_forces.htail_sx = forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2:...
              (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2-1 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2),:);
          
        aero_forces.htail_dx = [forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2,:);...
             forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2):...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 2,:)];
    end

    % CANARD
    if isequal(stick.model.canr,1)
%         geo.canard.leny=geo.canard.leny-pdcylin.stick.ncanard_carryth;

        aero_forces.canard_sx = forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 1:...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 2 + size(stick.nodes.canrC2_thick,2),:);

        aero_forces.canard_dx = [forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 1,:);...
             forces((pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 1 + size(stick.nodes.canrC2_thick,2):...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)*2 + size(stick.nodes.horrC2_thick,2)*2 - 3 + size(stick.nodes.canrC2_thick,2)*2,:)];

    end    
    
    
    
    
    
    
    
else
    % HORIZONTAL TAIL
    if isequal(stick.model.horr,1)
%         geo.htail.leny = geo.htail.leny-pdcylin.stick.nhtail_carryth;
        aero_forces.htail_sx = forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2):...
              (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2-1 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2),:);
          
        aero_forces.htail_dx =[forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2),:);...
             forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2):...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2)*2 - 2,:)];
    end

    % CANARD
    if isequal(stick.model.canr,1)
%         geo.canard.leny=geo.canard.leny-pdcylin.stick.ncanard_carryth;

        aero_forces.canard_sx = forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)+ size(stick.nodes.horrC2_thick,2)*2 - 1:...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2)*2 - 2 + size(stick.nodes.canrC2_thick,2),:);

        aero_forces.canard_dx = [forces( (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2)+ size(stick.nodes.horrC2_thick,2)*2 - 1,:);...
             forces((pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2)*2 - 1 + size(stick.nodes.canrC2_thick,2):...
            (pdcylin.stick.nfuse+1) + size(stick.nodes.winrC2_thick,2)*2 + size(stick.nodes.vert_thick,2) + size(stick.nodes.horrC2_thick,2)*2 - 3 + size(stick.nodes.canrC2_thick,2)*2,:)];

    end    
end
            
%% Fuse

for i=1:length(stick.nodes.fuse_thick)-1 
    dist.fuse.x(i)=abs(stick.nodes.fuse_thick(1,i)-stick.nodes.fuse_thick(1,i+1));
    dist.fuse.y(i)=abs(stick.nodes.fuse_thick(2,i)-stick.nodes.fuse_thick(2,i+1));
    dist.fuse.z(i)=abs(stick.nodes.fuse_thick(3,i)-stick.nodes.fuse_thick(3,i+1));
end

for i=1:length(dist.fuse.x)
    distances.fuse.x(i)=sum(dist.fuse.x(end:-1:i));
    distances.fuse.y(i)=sum(dist.fuse.y(end:-1:i));
    distances.fuse.z(i)=sum(dist.fuse.z(end:-1:i));
end
            
               
                    
%% Wing            
            
for i=1:size(stick.nodes.winrC2_thick,2)-1
    
    dist.wing.x(i)=abs(stick.nodes.winrC2_thick(1,i)-stick.nodes.winrC2_thick(1,i+1));
    dist.wing.y(i)=abs(stick.nodes.winrC2_thick(2,i)-stick.nodes.winrC2_thick(2,i+1));
    dist.wing.span(i)=sqrt((stick.nodes.winrC2_thick(1,i)-(stick.nodes.winrC2_thick(1,i+1)))^2+(stick.nodes.winrC2_thick(2,i)...
        -(stick.nodes.winrC2_thick(2,i+1)))^2);
end

for i=1:length(dist.wing.span)
    distances.wing.x(i)=sum(dist.wing.x(end:-1:i));
    distances.wing.y(i)=sum(dist.wing.y(end:-1:i));
    distances.wing.span(i)=sum(dist.wing.span(end:-1:i));
end


%% Vtail
if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    for i=1:size(stick.nodes.vert_thick,2)-1
        dist.vtail.x(i)=abs(stick.nodes.vert_thick(1,i)-stick.nodes.vert_thick(1,i+1));
        dist.vtail.y(i)=abs(stick.nodes.vert_thick(2,i)-stick.nodes.vert_thick(2,i+1));
        dist.vtail.span(i)=sqrt((stick.nodes.vert_thick(1,i)-(stick.nodes.vert_thick(1,i+1)))^2+(stick.nodes.vert_thick(3,i)...
            -(stick.nodes.vert_thick(3,i+1)))^2);
    end
else

    for i=1:size(stick.nodes.vert_thick,2)-1
        dist.vtail.x(i)=abs(stick.nodes.vert_thick(1,i)-stick.nodes.vert_thick(1,i+1));
        dist.vtail.y(i)=abs(stick.nodes.vert_thick(2,i)-stick.nodes.vert_thick(2,i+1));
        dist.vtail.span(i)=sqrt((stick.nodes.vert_thick(1,i)-(stick.nodes.vert_thick(1,i+1)))^2+(stick.nodes.vert_thick(3,i)...
            -(stick.nodes.vert_thick(3,i+1)))^2);
    end
end

for i=1:length(dist.vtail.span)
    distances.vtail.x(i)=sum(dist.vtail.x(end:-1:i));
    distances.vtail.y(i)=sum(dist.vtail.y(end:-1:i));
    distances.vtail.span(i)=sum(dist.vtail.span(end:-1:i));
end





%% Htail
if isequal(stick.model.horr,1)                

    for i = 1: size(stick.nodes.horrC2_thick,2)-1
        dist.htail.x(i)=abs(stick.nodes.horrC2_thick(1,i)-stick.nodes.horrC2_thick(1,i+1));
        dist.htail.y(i)=abs(stick.nodes.horrC2_thick(2,i)-stick.nodes.horrC2_thick(2,i+1));
        dist.htail.span(i)=sqrt((stick.nodes.horrC2_thick(1,i)-(stick.nodes.horrC2_thick(1,i+1)))^2+(stick.nodes.horrC2_thick(2,i)...
            -(stick.nodes.horrC2_thick(2,i+1)))^2);
    end

    for i=1:length(dist.htail.span)
        distances.htail.x(i)=sum(dist.htail.x(end:-1:i));
        distances.htail.y(i)=sum(dist.htail.y(end:-1:i));
        distances.htail.span(i)=sum(dist.htail.span(end:-1:i));
    end

end

%% canard
if isequal(stick.model.canr,1)                

    
    for i= 1: size(stick.nodes.canrC2_thick,2)-1
        dist.canard.x(i)=abs(stick.nodes.canrC2_thick(1,i)-stick.nodes.canrC2_thick(1,i+1));
        dist.canard.y(i)=abs(stick.nodes.canrC2_thick(2,i)-stick.nodes.canrC2_thick(2,i+1));
        dist.canard.span(i)=sqrt((stick.nodes.canrC2_thick(1,i)-(stick.nodes.canrC2_thick(1,i+1)))^2+(stick.nodes.canrC2_thick(2,i)...
            -(stick.nodes.canrC2_thick(2,i+1)))^2);
    end

    for i=1:length(dist.canard.span)
        distances.canard.x(i)=sum(dist.canard.x(end:-1:i));
        distances.canard.y(i)=sum(dist.canard.y(end:-1:i));
        distances.canard.span(i)=sum(dist.canard.span(end:-1:i));
    end


end
