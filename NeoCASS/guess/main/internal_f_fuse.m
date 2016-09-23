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

function [int_forces] =internal_f_fuse(forces, state, stick, distances, forces_wing2_dx, forces_wing2_sx, forces_wing_dx,...
                                            forces_wing_sx, forces_vtail, aircraft, acc)

int_forces=[];
int_forces_fus=[];
int_forces_can=[];
int_forces_wing=[];
int_forces_vtail=[];
int_forces_engines=[];



Xcg = aircraft.weight_balance.COG(27,1,1);




for i=1:length(forces)

%% Wing2   
if isequal(stick.model.canr, 1)
    if  isequal(stick.nodes.fuse_thick(1,i)>=stick.nodes.canrC2_thick(1,1),1)==0
        int_forces_can.Tx(i,:)=0;
        int_forces_can.Ty(i,:)=0;
        int_forces_can.Tz(i,:)=0;
        int_forces_can.Mx(i,:)=0;
        int_forces_can.My(i,:)=0;
        int_forces_can.Mz(i,:)=0;
    else 
        int_forces_can.Tx(i,:)=forces_wing2_dx.Tx(1)+forces_wing2_sx.Tx(1);
        int_forces_can.Ty(i,:)=forces_wing2_dx.Ty(1)+forces_wing2_sx.Ty(1);
        int_forces_can.Tz(i,:)=forces_wing2_dx.Tz(1)+forces_wing2_sx.Tz(1);
        int_forces_can.Mx(i,:)=forces_wing2_dx.Mx(1)+forces_wing2_sx.Mx(1);
        int_forces_can.My(i,:)=forces_wing2_dx.My(1)+forces_wing2_sx.My(1)+(forces_wing2_dx.Tz(1)+forces_wing2_sx.Tz(1))*...
                                                                           (Xcg-stick.nodes.canrC2_thick(1,1));

        int_forces_can.Mz(i,:)=forces_wing2_dx.Mz(1)+forces_wing2_sx.Mz(1);
    
    end
end
        
%% Wing        
    if  isequal(stick.nodes.fuse_thick(1,i)>=stick.nodes.winrC2_thick(1,1),1)==0
        int_forces_wing.Tx(i,:)=0;
        int_forces_wing.Ty(i,:)=0;
        int_forces_wing.Tz(i,:)=0;
        int_forces_wing.Mx(i,:)=0;
        int_forces_wing.My(i,:)=0;
        int_forces_wing.Mz(i,:)=0;
    else 
        int_forces_wing.Tx(i,:)=forces_wing_dx.Tx(1)+forces_wing_sx.Tx(1);
        int_forces_wing.Ty(i,:)=forces_wing_dx.Ty(1)+forces_wing_sx.Ty(1);
        int_forces_wing.Tz(i,:)=forces_wing_dx.Tz(1)+forces_wing_sx.Tz(1);
        int_forces_wing.Mx(i,:)=forces_wing_dx.Mx(1)+forces_wing_sx.Mx(1);
        int_forces_wing.My(i,:)=forces_wing_dx.My(1)+forces_wing_sx.My(1)+(forces_wing_dx.Tz(1)+forces_wing_sx.Tz(1))*...
                                                                           (Xcg-stick.nodes.winrC2_thick(1,1));

        int_forces_wing.Mz(i,:)=forces_wing_dx.Mz(1)+forces_wing_sx.Mz(1);
    
     end    

%% Vtail        
    if  isequal(stick.nodes.fuse_thick(1,i)>=stick.nodes.vert_thick(1,1),1)==0
        int_forces_vtail.Tx(i,:)=0;
        int_forces_vtail.Ty(i,:)=0;
        int_forces_vtail.Tz(i,:)=0;
        int_forces_vtail.Mx(i,:)=0;
        int_forces_vtail.My(i,:)=0;
        int_forces_vtail.Mz(i,:)=0;
    else 
        int_forces_vtail.Tx(i,:)=forces_vtail.Tx(1);
        int_forces_vtail.Ty(i,:)=forces_vtail.Ty(1);
        int_forces_vtail.Tz(i,:)=forces_vtail.Tz(1);
        int_forces_vtail.Mx(i,:)=forces_vtail.Mx(1);
        int_forces_vtail.My(i,:)=forces_vtail.My(1)+(forces_vtail.Tz(1))*(Xcg-stick.nodes.vert_thick(1,1));
        int_forces_vtail.Mz(i,:)=forces_vtail.Mz(1);
    
     end          
        
        
end

%% Fuse
int_forces_fus.Tx(1,:)=0;
int_forces_fus.Ty(1,:)=0;
int_forces_fus.Tz(1,:)=0;
int_forces_fus.Mx(1,:)=0;
int_forces_fus.My(1,:)=0;
int_forces_fus.Mz(1,:)=0;


for i=2:length(forces)
        int_forces_fus.Tx(i,:)=int_forces_fus.Tx(i-1,:)+forces(i,1);
        int_forces_fus.Ty(i,:)=int_forces_fus.Ty(i-1,:)+forces(i,2);
        int_forces_fus.Tz(i,:)=int_forces_fus.Tz(i-1,:)+forces(i,3);
        int_forces_fus.Mx(i,:)=int_forces_fus.Mx(i-1,:);
        int_forces_fus.My(i,:)=int_forces_fus.My(i-1,:)+forces(i,3)*(Xcg-distances.x(i-1));
        int_forces_fus.Mz(i,:)=int_forces_fus.Mz(i-1,:); 
   
end



%% Engines
for i=1:length(forces)
    int_forces_engines.Tx(1,:)=0;
    int_forces_engines.Ty(1,:)=0;
    int_forces_engines.Tz(1,:)=0;
    int_forces_engines.My(1,:)=0;

    
        if  isequal(stick.nodes.fuse_thick(1,i)>=(aircraft.engines1.Location_engines_nacelles_on_X + aircraft.engines1.nacelle_length/2),1)==0
            int_forces_engines.Tx(i,:)=0;
            int_forces_engines.Ty(i,:)=0;
            int_forces_engines.Tz(i,:)=0;
            int_forces_engines.Mx(i,:)=0;
            int_forces_engines.My(i,:)=0;
            int_forces_engines.Mz(i,:)=0;

        else 
            int_forces_engines.Tx(i,:)=-aircraft.weight_balance.COG(7,4,1)*acc(1) ;
            int_forces_engines.Ty(i,:)=-aircraft.weight_balance.COG(7,4,1)*acc(2) ;
            int_forces_engines.Tz(i,:)=-aircraft.weight_balance.COG(7,4,1)*acc(3) ;
            int_forces_engines.My(i,:)=(int_forces_engines.Tz(i,:))*(Xcg-(aircraft.engines1.Location_engines_nacelles_on_X + aircraft.engines1.nacelle_length/2));
    
        end  
    
    
end





%% Total

if isequal(stick.model.canr, 1)
int_forces.Tx=int_forces_fus.Tx+int_forces_can.Tx+int_forces_wing.Tx+int_forces_vtail.Tx+int_forces_engines.Tx;
int_forces.Ty=int_forces_fus.Ty+int_forces_can.Ty+int_forces_wing.Ty+int_forces_vtail.Ty+int_forces_engines.Ty;
int_forces.Tz=int_forces_fus.Tz+int_forces_can.Tz+int_forces_wing.Tz+int_forces_vtail.Tz+int_forces_engines.Tz;
int_forces.Mx=int_forces_fus.Mx+int_forces_can.Mx+int_forces_wing.Mx+int_forces_vtail.Mx;
int_forces.My=int_forces_fus.My+int_forces_can.My+int_forces_wing.My+int_forces_vtail.My+int_forces_engines.My;
int_forces.Mz=int_forces_fus.Mz+int_forces_can.Mz+int_forces_wing.Mz+int_forces_vtail.Mz;
else
int_forces.Tx=int_forces_fus.Tx+int_forces_wing.Tx+int_forces_vtail.Tx+int_forces_engines.Tx;
int_forces.Ty=int_forces_fus.Ty+int_forces_wing.Ty+int_forces_vtail.Ty+int_forces_engines.Ty;
int_forces.Tz=int_forces_fus.Tz+int_forces_wing.Tz+int_forces_vtail.Tz+int_forces_engines.Tz;
int_forces.Mx=int_forces_fus.Mx+int_forces_wing.Mx+int_forces_vtail.Mx;
int_forces.My=int_forces_fus.My+int_forces_wing.My+int_forces_vtail.My+int_forces_engines.My;
int_forces.Mz=int_forces_fus.Mz+int_forces_wing.Mz+int_forces_vtail.Mz;   
end
% disp(int_forces_can.Tx)



