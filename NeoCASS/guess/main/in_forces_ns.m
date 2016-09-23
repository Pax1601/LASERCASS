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

function [inertial_forces_NS]=in_forces_ns(acc, mass, length, aircraft, stick, pdcylin, geo)
% modified by Travaglini 19/11/2009, the inertial loads due to bar mass is
% summed half to one node and half to second node
% Fuselage
l1 = mass.fus.NSM.dstr.*length.fus.Lbeam.*acc(1);
l2 = mass.fus.NSM.dstr.*length.fus.Lbeam.*acc(2);
l3 = mass.fus.NSM.dstr.*length.fus.Lbeam.*acc(3);

inertial_forces_NS.fuse(:,1)= [l1(1)*0.5; (l1(2:end)+l1(1:end-1))*0.5; l1(end)];
inertial_forces_NS.fuse(:,2)= [l2(1)*0.5; (l2(2:end)+l2(1:end-1))*0.5; l2(end)];
inertial_forces_NS.fuse(:,3)= [l3(1)*0.5; (l3(2:end)+l3(1:end-1))*0.5; l3(end)];

% wing 
l1 = mass.wing.NSM.dstr.*length.wing.Lbeam.*acc(1);
l2 = mass.wing.NSM.dstr.*length.wing.Lbeam.*acc(2);
l3 = mass.wing.NSM.dstr.*length.wing.Lbeam.*acc(3);

inertial_forces_NS.wing(:,1) = [l1(1)*0.5; (l1(2:end)+l1(1:end-1))*0.5; l1(end)];
inertial_forces_NS.wing(:,2) = [l2(1)*0.5; (l2(2:end)+l2(1:end-1))*0.5; l2(end)];
inertial_forces_NS.wing(:,3) = [l3(1)*0.5; (l3(2:end)+l3(1:end-1))*0.5; l3(end)];

% Vertical tail
l1 = mass.vtail.NSM.dstr.*length.vtail.Lbeam.*acc(1);
l2 = mass.vtail.NSM.dstr.*length.vtail.Lbeam.*acc(2);
l3 = mass.vtail.NSM.dstr.*length.vtail.Lbeam.*acc(3);

inertial_forces_NS.vtail(:,1) = [l1(1)*0.5; (l1(2:end)+l1(1:end-1))*0.5; l1(end)];
inertial_forces_NS.vtail(:,2) = [l2(1)*0.5; (l2(2:end)+l2(1:end-1))*0.5; l2(end)];
inertial_forces_NS.vtail(:,3) = [l3(1)*0.5; (l3(2:end)+l3(1:end-1))*0.5; l3(end)];


if isequal(stick.model.horr, 1)
    
    l1 = mass.htail.NSM.dstr.*length.htail.Lbeam.*acc(1);
    l2 = mass.htail.NSM.dstr.*length.htail.Lbeam.*acc(2);
    l3 = mass.htail.NSM.dstr.*length.htail.Lbeam.*acc(3);
    
    inertial_forces_NS.htail(:,1) = [l1(1)*0.5; (l1(2:end)+l1(1:end-1))*0.5; l1(end)];
    inertial_forces_NS.htail(:,2) = [l2(1)*0.5; (l2(2:end)+l2(1:end-1))*0.5; l2(end)];
    inertial_forces_NS.htail(:,3) = [l3(1)*0.5; (l3(2:end)+l3(1:end-1))*0.5; l3(end)];
end

if isequal(stick.model.canr, 1)
    l1 = mass.canard.NSM.dstr.*length.canr.Lbeam.*acc(1);
    l2 = mass.canard.NSM.dstr.*length.canr.Lbeam.*acc(2);
    l3 = mass.canard.NSM.dstr.*length.canr.Lbeam.*acc(3);
    
    inertial_forces_NS.canard(:,1) = [l1(1)*0.5; (l1(2:end)+l1(1:end-1))*0.5; l1(end)];
    inertial_forces_NS.canard(:,2) = [l2(1)*0.5; (l2(2:end)+l2(1:end-1))*0.5; l2(end)];
    inertial_forces_NS.canard(:,3) = [l3(1)*0.5; (l3(2:end)+l3(1:end-1))*0.5; l3(end)];
end