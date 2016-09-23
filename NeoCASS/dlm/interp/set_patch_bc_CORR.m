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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function [D_DISPL, dwn, N_DISPL, ROT] = set_patch_bc_CORR(pind, cref, V, midpoint, colloc, aero_node, normal, str_data, mode, k_list, AERO, ...
    SCALE, PLOT_RES, state)
% get total number of active modes
NMODES = size(mode, 3);
if (SCALE)
    colloc    = colloc .* cref;
    midpoint  = midpoint .* cref;
    aero_node = aero_node .* cref;
end
%
[Id, Ic, In] = set_patch_interf_matrix(pind, midpoint, colloc, aero_node, str_data, AERO);
%
% get midpoint displacements
np = size(colloc, 1);
nk = length(k_list);
dwn = zeros(np, NMODES, nk);
V_dir = repmat([1,0,0], np, 1);
DIST = colloc - midpoint;
DIST = dot(DIST, V_dir, 2);
if abs(colloc(1,2)-aero_node(1,2,2))<=1e-9
    DIST2 = -colloc(:,3) + squeeze(aero_node(:,2,3));
else
    DIST2 = -colloc(:,2) + squeeze(aero_node(:,2,2));
end
if (PLOT_RES)
    offset = 100;
    for m = offset:(offset + NMODES)
        figure(m); close;
    end
end
if norm(state(3:end))
    ARM = (colloc - repmat(AERO.geo.CG, np, 1));
    OMEGA_P = cross(ARM, repmat(state(3:end), np, 1), 2);
    % add rigid body contributions to collocation point velocity
    DeltaAlpha = -OMEGA_P(:,3)/V;
    DeltaBetha = -OMEGA_P(:,2)/V;
end
ROT = zeros(np,3,NMODES);
for m = 1: NMODES
    xdef  = Id * mode(:, 1, m);
    ydef  = Id * mode(:, 2, m);
    zdef  = Id * mode(:, 3, m);
    D_DISPL(:,:,m) = [xdef, ydef, zdef];
    xdef  = Ic * mode(:, 1, m);
    ydef  = Ic * mode(:, 2, m);
    zdef  = Ic * mode(:, 3, m);
    C_DISPL(:,:,m) = [xdef, ydef, zdef];
    xdef  = In * mode(:, 1, m);
    ydef  = In * mode(:, 2, m);
    zdef  = In * mode(:, 3, m);
    N_DISPL(:,:,m) = [xdef, ydef, zdef];
    if (PLOT_RES)
        offset = 99;
        figure(m + offset); hold on; grid;
        % structural displacements
        plot3(str_data(:,1) + mode(:,1,m), str_data(:,2) + mode(:,2,m), str_data(:,3) + mode(:,3,m),'.k');
        % midpoint displacements
        plot3(midpoint(:,1) + D_DISPL(:,1,m),midpoint(:,2) + D_DISPL(:,2,m),midpoint(:,3) + D_DISPL(:,3,m),'r.');
        % collocation point displacements
        plot3(colloc(:,1) + C_DISPL(:,1,m),colloc(:,2) + C_DISPL(:,2,m),colloc(:,3) + C_DISPL(:,3,m),'yo');
    end
    %   get COLLOCATION POINT normal displacements
    CDISP_corr = C_DISPL(:,:,m);
    if ~isempty(find(imag(C_DISPL)~=0))
        stop = 1;
    end
    if norm(state(3:end))
        for i = 1 : np
            Rot_Vel =   [cos(state(1)+DeltaAlpha(i))*cos(state(2)+DeltaBetha(i)), -cos(state(1)+DeltaAlpha(i))*sin(state(2)+DeltaBetha(i)), -sin(state(1)+DeltaAlpha(i));...
                sin(state(2)+DeltaBetha(i)),  cos(state(2)+DeltaBetha(i)), 0;...
                sin(state(1)+DeltaAlpha(i))*cos(state(2)+DeltaBetha(i)), -sin(state(1)+DeltaAlpha(i))*sin(state(2)+DeltaBetha(i)), cos(state(1)+DeltaAlpha(i))];
            CDISP_corr(i,:) = (Rot_Vel*CDISP_corr(i,:)')';
        end
    else
        Rot_Vel =   [cos(state(1))*cos(state(2)), -cos(state(1))*sin(state(2)), -sin(state(1));...
            sin(state(2)),  cos(state(2)), 0;...
            sin(state(1))*cos(state(2)), -sin(state(1))*sin(state(2)), cos(state(1))];
        CDISP_corr = (Rot_Vel*CDISP_corr')';
    end
    CNDISPL = dot(CDISP_corr, normal, 2) ;
    %   get DOUBLET POINT normal displacements
    DNDISPL = dot(D_DISPL(:,:,m), normal, 2) ;
    DN = (dot(C_DISPL(:,:,m), normal, 2) - DNDISPL) ./ DIST;
    % get new normal vector
    aero_node1 = reshape(aero_node(:,1,:),np,3) + N_DISPL(1:4:end,:,m);
    aero_node2 = reshape(aero_node(:,2,:),np,3) + N_DISPL(2:4:end,:,m);
    aero_node3 = reshape(aero_node(:,3,:),np,3) + N_DISPL(3:4:end,:,m);
    aero_node4 = reshape(aero_node(:,4,:),np,3) + N_DISPL(4:4:end,:,m);
    tangent = reshape(  ((aero_node(:,3,:)-aero_node(:,2,:))*0.25 + aero_node(:,2,:)) -   ((aero_node(:,4,:)-aero_node(:,1,:))*0.25 + aero_node(:,1,:)),np,3);
    tangent_norm = sqrt(tangent(:,1).^2 + tangent(:,2).^2 +tangent(:,3).^2);
    tangent = tangent./ repmat(tangent_norm,1,3);
    normal_new = -cross(aero_node3-aero_node1,aero_node4-aero_node2,2);
    normal_new_norm = sqrt(normal_new(:,1).^2 + normal_new(:,2).^2 +normal_new(:,3).^2);
    normal_new = normal_new./repmat(normal_new_norm,1,3);
    tangent_new = (aero_node2+ 0.25*(aero_node3-aero_node2)) - (aero_node1+ 0.25*(aero_node4-aero_node1));
    tangent_norm = sqrt(tangent_new(:,1).^2 + tangent_new(:,2).^2 +tangent_new(:,3).^2);
    tangent_new = tangent_new./ repmat(tangent_norm,1,3);
    ROT(:,:,m) = cross(normal,normal_new) + repmat(dot(cross(tangent_new,tangent),normal,2),1,3).*normal;
    for k = 1:nk 
        dwn(:, m, k) = 1i .*( CNDISPL ).* (k_list(k) / cref) + DN ;
    end
end
end
