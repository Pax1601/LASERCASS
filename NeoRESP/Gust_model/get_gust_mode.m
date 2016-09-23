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
%  FFAST Project
%
%  NeoSYM
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Lorenzo Travaglini
%***********************************************************************************************************************
function MODES = get_gust_mode(Node,ndof,fun,ind)


if ind == 3 % vertical gust
    
    b = max(Node.Coord(:,2));
    
    V = zeros(ndof,1);
    ngrid = size(Node.Coord,1);
    for i = 1: ngrid
        y = Node.Coord(i,2)/b;
        dof = Node.DOF(i, 1:6);
        index = find(dof);
        if ~isempty(find(index==3,1))
            V(dof(3)) = eval(fun);
        end
        
    end
    
    MODES = get_mode_shapes(ngrid, Node.DOF, 1, V);
    mscale = max(max(abs(MODES)));
    MODES = MODES ./ mscale;
    crossR = zeros(3, 3, ngrid);
    for i = 1: ngrid
        
%         NRd(:,:,i) = Rmat(MODES(i, 4:6));
        crossR(:,:,i) = crossm(MODES(i, 4:6));
        
    end
    
    %     if (beam_model.Info.nrbe0 > 0)
    AERO_POS = update_aerobeam_node(ngrid, Node, MODES(:,1:3), crossR(:,:,:));
    % update coord database with slave nodes position
    for n=1:ngrid
        if ~isempty(Node.Aero)
            ne = length(Node.Aero.Index(n).data);
            if ne
                MODES(Node.Aero.Index(n).data, 1:3) = AERO_POS(n).data';
            end
        end
    end
    clear AERO_POS;
    %         fprintf(fid, 'done.');
    %     end
    
elseif ind == 2 %lateral gust
    
    b = max(Node.Coord(:,1));
    
    V = zeros(ndof,1);
    ngrid = size(Node.Coord,1);
    for i = 1: ngrid
        y = Node.Coord(i,1)/b;
        dof = Node.DOF(i, 1:6);
        index = find(dof);
        if ~isempty(find(index==2,1))
            V(dof(2)) = eval(fun);
        end
        
    end
    
    MODES = get_mode_shapes(ngrid, Node.DOF, 1, V);
    mscale = max(max(abs(MODES)));
    MODES = MODES ./ mscale;
    crossR = zeros(3, 3, ngrid);
    for i = 1: ngrid
        
%         NRd(:,:,i) = Rmat(MODES(i, 4:6));
        crossR(:,:,i) = crossm(MODES(i, 4:6));
        
    end
    
    %     if (beam_model.Info.nrbe0 > 0)
    AERO_POS = update_aerobeam_node(ngrid, Node, MODES(:,1:3), crossR(:,:,:));
    % update coord database with slave nodes position
    for n=1:ngrid
        if ~isempty(Node.Aero)
            ne = length(Node.Aero.Index(n).data);
            if ne
                MODES(Node.Aero.Index(n).data, 1:3) = AERO_POS(n).data';
            end
        end
    end
    clear AERO_POS;
    
else
    error('gust must be in z or y direction');
end
