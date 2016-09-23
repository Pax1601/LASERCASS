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

function [int_forces] =internal_f_vtail_twin(forces, state, stick, distances, aircraft)

int_forces=[];




%     for i=1:length(forces)
%         int_forces.Tx(i,:)=sum(forces(end:-1:i,1))+2*forces_htail_dx.Tx(1);
%         int_forces.Ty(i,:)=sum(forces(end:-1:i,2))+2*forces_htail_dx.Ty(1);
%         int_forces.Tz(i,:)=sum(forces(end:-1:i,3))+2*forces_htail_dx.Tz(1);
%         int_forces.Mx(i,:)=sum((forces(end:-1:i,2).*distances.y(end:-1:i)'))+2*forces_htail_dx.Ty(1).*distances.y(end-i+1)'+2*forces_htail_dx.Mx(1);
%         int_forces.My(i,:)=sum((forces(end:-1:i,2).*distances.span(end:-1:i)'))+2*forces_htail_dx.Ty(1).*distances.span(end-i+1)'+2*forces_htail_dx.My(1);
%         int_forces.Mz(i,:)=sum((forces(end:-1:i,3).*distances.y(end:-1:i)'))+2*forces_htail_dx.Tz(1).*distances.y(end-i+1)'+2*forces_htail_dx.Mz(1);
%         
%     end
   
   %Inversione delle forze aerodinamiche
    forces=forces(end:-1:1,:);

    
    % i meno delle forze ereditate dal piano orizzontale sono dovuti alla
    % reazione
    
    if isequal(stick.model.horr, 1)
        int_forces.Tx(1,:)=forces(1,1);
        int_forces.Ty(1,:)=forces(1,2);
        int_forces.Tz(1,:)=forces(1,3);
        int_forces.Mx(1,:)=forces(1,2);
        int_forces.My(1,:)=forces(1,2);
        int_forces.Mz(1,:)=forces(1,3);
    else
        int_forces.Tx(1,:)=forces(1,1);
        int_forces.Ty(1,:)=forces(1,2);
        int_forces.Tz(1,:)=forces(1,3);
        int_forces.Mx(1,:)=forces(1,2);
        int_forces.My(1,:)=forces(1,2);
        int_forces.Mz(1,:)=forces(1,3);
    end
    
    
    
    
    for i=2:length(forces)
        int_forces.Tx(i,:)=int_forces.Tx(i-1,:)+forces(i,1);
        int_forces.Ty(i,:)=int_forces.Ty(i-1,:)+forces(i,2);
        int_forces.Tz(i,:)=int_forces.Tz(i-1,:)+forces(i,3);
        int_forces.Mx(i,:)=int_forces.Mx(i-1,:)+forces(i,2)*distances.y(end-i+2);
        int_forces.My(i,:)=int_forces.My(i-1,:)+forces(i,2)*distances.span(end-i+2);
        int_forces.Mz(i,:)=int_forces.Mz(i-1,:)+forces(i,3)*distances.y(end-i+2);
    end


    int_forces.Tx=int_forces.Tx(end:-1:1);
    int_forces.Ty=int_forces.Ty(end:-1:1);
    int_forces.Tz=int_forces.Tz(end:-1:1);
    int_forces.Mx=int_forces.Mx(end:-1:1);
    int_forces.My=int_forces.My(end:-1:1);
    int_forces.Mz=int_forces.Mz(end:-1:1);