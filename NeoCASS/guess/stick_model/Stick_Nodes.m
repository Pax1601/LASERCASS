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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Generate stick model nodes.
%
%   Author: <andreadr@kth.se>
%
% Called by:    Stick_Model.m
%
% Calls:        DeltaElem.m
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080724      1.0      A. Da Ronch      Creation
%     091119      1.3.9    L. Travaglini    Modification
%     120424      2.1.237  L. Riccobene     Modification
%
%*******************************************************************************
function [stick, geo] = Stick_Nodes(stick, geo, varargin)

%--------------------------------------------------------------------------------------------------
% Fuselage
if isequal(stick.model.fuse, 1)
    
    stick.nodes.fuse_thick = DeltaElem(stick.ptos.fuse, geo.fus.CAERO1.n);
    stick.nodes.fuse       = DeltaElem(stick.ptos.fuse, geo.fus.CAERO1.n_coarse);
    
    % Calculate position of nodes and mid-points along the stick beam line
    geo.fus.x_nodes_thick     = nodeinterp(stick.nodes.fuse_thick);
    geo.fus.x_nodes_1_2_thick = meancouple(geo.fus.x_nodes_thick);
    
    % Calculate position of nodes and mid-points along the stick beam line
    geo.fus.x_nodes     = nodeinterp(stick.nodes.fuse);
    geo.fus.x_nodes_1_2 = meancouple(geo.fus.x_nodes);
    
    % Beam length for structural elements
    for i = 1:length(geo.fus.x_nodes) - 1
        stick.fus.Lbeam(i,1) = norm( stick.nodes.fuse(:,i+1) - stick.nodes.fuse(:,i) );
    end
    
    % Beam length for structural elements
    for i = 1:length(geo.fus.x_nodes_thick) - 1
        stick.fus.Lbeam_thick(i,1) = norm( stick.nodes.fuse_thick(:,i+1) - stick.nodes.fuse_thick(:,i) );
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.fuse.str_rec_coef.Coord = zeros(3, 2*length(stick.PID.fuse));
    for i = 1:length(geo.fus.x_nodes) - 1
        t = i*2-1:i*2;
        % Absolute coordinates in physical position
        N1 = stick.nodes.fuse(:,i)+stick.OFFSET.fuse; N1 = N1';
        N2 = ( (stick.nodes.fuse(:,i)+stick.nodes.fuse(:,i+1))./2 )+stick.OFFSET.fuse; N2 = N2';
        N3 = stick.nodes.fuse(:,i+1)+stick.OFFSET.fuse; N3 = N3';
        d = interp_colloc_pos(N1, N2, N3);
        stick.PBAR.fuse.str_rec_coef.Coord(:,t) = d';
    end
    for i = 1:length(geo.fus.x_nodes_thick) - 1
        t = i*2-1:i*2;
        % Absolute coordinates in physical position
        N1 = stick.nodes.fuse_thick(:,i)+stick.OFFSET.fuse; N1 = N1';
        N2 = ( (stick.nodes.fuse_thick(:,i)+stick.nodes.fuse_thick(:,i+1))./2 )+stick.OFFSET.fuse; N2 = N2';
        N3 = stick.nodes.fuse_thick(:,i+1)+stick.OFFSET.fuse; N3 = N3';
        d = interp_colloc_pos(N1, N2, N3);
        stick.PBAR.fuse.str_rec_coef.Coord_thick(:,t) = d';
    end
    
    % Volume and CG calculation
    SA = pi.*geo.fus.r.^2;  % section area
    geo.fus.V  = meancouple(SA).*diff(geo.fus.x_nodes_thick);
    geo.fus.Vt = geo.fus.V; 
    geo.fus.cg = geo.fus.x_nodes_1_2_thick;
    
end
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Wing
if isequal(stick.model.winr, 1)
    
    % Cycle on the number of sectors defined inside wing
    for i = 1:length(geo.wing.CAERO1.n)
        
        nodesW_thick = DeltaElem(stick.ptos.winrC2(:,i:i+1), geo.wing.CAERO1.n(i));
        % add the deformation
    
%         if nargin > 2
%         pdcylin = varargin{1};
%         % deform the geometry
%         % check if the pdcylin holds the deformation informations
%         % bending 
%         if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
%             %aircraft.deformation.bending.type = ''
%             %aircraft.deformation.bending.coefficients = []
%             %aircraft.deformation.bending.max_tip_value = ''  
%             for j = 1 : length(pdcylin.deformation.bending.coefficients)
%                 nodesW_thick(3,:) = nodesW_thick(3,:)+ (pdcylin.deformation.bending.coefficients(j)* nodesW_thick(2,:).^j)./pdcylin.deformation.bending.max_tip_value;
%                
%             end
%         end
%         end
        %         nodesQ_thick = DeltaElem(stick.ptos.winr(:,i:i+1), geo.wing.CAERO1.n(i));
        
        if i == 1
            stick.nodes.winrC2_thick = [stick.nodes.winrC2_thick, nodesW_thick];
            %             stick.nodes.winr_thick   = [stick.nodes.winr_thick, nodesQ_thick];
            stick.nodes.winr_thick   = stick.nodes.winrC2_thick;
            
        else
            stick.nodes.winrC2_thick = [stick.nodes.winrC2_thick, nodesW_thick(:,2:end)];
            %             stick.nodes.winr_thick   = [stick.nodes.winr_thick, nodesQ_thick(:,2:end)];
            stick.nodes.winr_thick   = stick.nodes.winrC2_thick;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line
        geo.wing.y_nodes_thick     = nodeinterp(stick.nodes.winr_thick);
        geo.wing.y_nodes_1_2_thick = meancouple(geo.wing.y_nodes_thick);
        
        % Beam length for structural elements
        for j = 1:length(geo.wing.y_nodes_thick) - 1
            stick.wing.Lbeam_thick(j,1) = norm( stick.nodes.winrC2_thick(:,j+1) - stick.nodes.winrC2_thick(:,j) );
        end
        
    end
    
    for i = 1:length(geo.wing.CAERO1.n_coarse)
        
        nodesW = DeltaElem(stick.ptos.winrC2(:,i:i+1), geo.wing.CAERO1.n_coarse(i));
        %         nodesQ = DeltaElem(stick.ptos.winr(:,i:i+1), geo.wing.CAERO1.n_coarse(i));
        % add the deformation
    
%         if nargin > 2
%         pdcylin = varargin{1};
%         % deform the geometry
%         % check if the pdcylin holds the deformation informations
%         % bending 
%         if  isfield(pdcylin,'deformation') && isfield(pdcylin.deformation,'bending')
%             %aircraft.deformation.bending.type = ''
%             %aircraft.deformation.bending.coefficients = []
%             %aircraft.deformation.bending.max_tip_value = ''  
%             for j = 1 : length(pdcylin.deformation.bending.coefficients)
%                 nodesW(3,:) = nodesW(3,:)+ (pdcylin.deformation.bending.coefficients(j)* nodesW(2,:).^j)./pdcylin.deformation.bending.max_tip_value;
% 
%             end
%         end
%         end
        if i == 1
            stick.nodes.winrC2 = [stick.nodes.winrC2, nodesW];
            %             stick.nodes.winr   = [stick.nodes.winr, nodesQ];
            stick.nodes.winr   = stick.nodes.winrC2;
        else
            stick.nodes.winrC2 = [stick.nodes.winrC2, nodesW(:,2:end)];
            %             stick.nodes.winr   = [stick.nodes.winr, nodesQ(:,2:end)];
            stick.nodes.winr   = stick.nodes.winrC2;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line
        geo.wing.y_nodes     = nodeinterp(stick.nodes.winr);
        geo.wing.y_nodes_1_2 = meancouple(geo.wing.y_nodes);
        
        % Beam length for structural elements
        for j = 1:length(geo.wing.y_nodes) - 1
            stick.wing.Lbeam(j,1) = norm( stick.nodes.winrC2(:,j+1) - stick.nodes.winrC2(:,j) );
        end
        
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.wing.str_rec_coef.Coord = zeros(3,2*length(stick.PID.wing));
    
    % Volume computation (structural box volume and total volume)
    % Zs = str. box chord, r = aerodynamic chord
    geo.wing.V  = meancouple(geo.wing.Zs).*meancouple(geo.wing.tbs).*diff(geo.wing.y_nodes_thick);
    geo.wing.Vt = meancouple(geo.wing.r).*meancouple(geo.wing.tbs).*diff(geo.wing.y_nodes_thick);
    geo.wing.cg = geo.wing.y_nodes_1_2_thick;
    
    
  
    
    
end
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Vertical tail
if isequal(stick.model.vert, 1)
    
    for k = 1:length(geo.vtail.CAERO1.n)
        
        nodesV_thick = DeltaElem(stick.ptos.vert(:,k:k+1), geo.vtail.CAERO1.n(k));
        
        if k == 1
            stick.nodes.vert_thick = [stick.nodes.vert_thick, nodesV_thick];
        else
            stick.nodes.vert_thick = [stick.nodes.vert_thick, nodesV_thick(:,2:end)];
        end
        % Calculate position of nodes and mid-points along the stick beam line
        geo.vtail.y_nodes_thick     = nodeinterp(stick.nodes.vert_thick);
        geo.vtail.y_nodes_1_2_thick = meancouple(geo.vtail.y_nodes_thick);
        %
        % Beam length for structural elements
        for i = 1:length(geo.vtail.y_nodes_thick) - 1
            stick.vtail.Lbeam_thick(i,1) = norm( stick.nodes.vert_thick(:,i+1) - stick.nodes.vert_thick(:,i) );
        end
        
    end
    
    for k = 1:length(geo.vtail.CAERO1.n_coarse)
        
        nodesV = DeltaElem(stick.ptos.vert(:,k:k+1), geo.vtail.CAERO1.n_coarse(k));
        
        if k == 1
            stick.nodes.vert = [stick.nodes.vert, nodesV];
        else
            stick.nodes.vert = [stick.nodes.vert, nodesV(:,2:end)];
        end
        % Calculate position of nodes and mid-points along the stick beam line
        geo.vtail.y_nodes     = nodeinterp(stick.nodes.vert);
        geo.vtail.y_nodes_1_2 = meancouple(geo.vtail.y_nodes);
        
        % Beam length for structural elements
        for i = 1:length(geo.vtail.y_nodes) - 1
            stick.vtail.Lbeam(i,1) = norm( stick.nodes.vert(:,i+1) - stick.nodes.vert(:,i) );
        end
        
    end
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.vert.str_rec_coef.Coord = zeros(3,2*length(stick.PID.vert));
    
    % Volume computation (structural box volume and total volume)
    % Zs = str. box chord, r = aerodynamic chord
    geo.vtail.V  = meancouple(geo.vtail.Zs).*meancouple(geo.vtail.tbs).*diff(geo.vtail.y_nodes_thick);
    geo.vtail.Vt = meancouple(geo.vtail.r).*meancouple(geo.vtail.tbs).*diff(geo.vtail.y_nodes_thick);
    geo.vtail.cg = geo.vtail.y_nodes_1_2_thick;
    
end
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Horizontal tail
if isequal(stick.model.horr, 1)
    
    for j = 1:length(geo.htail.CAERO1.n)
        
        nodesH_thick = DeltaElem(stick.ptos.horrC2(:,j:j+1), geo.htail.CAERO1.n(j));
        %         nodesh_thick = DeltaElem(stick.ptos.horr(:,j:j+1), geo.htail.CAERO1.n(j));
        
        if j == 1
            stick.nodes.horrC2_thick = [stick.nodes.horrC2_thick, nodesH_thick];
            %             stick.nodes.horr_thick   = [stick.nodes.horr_thick, nodesh_thick];
            stick.nodes.horr_thick   = stick.nodes.horrC2_thick;
        else
            stick.nodes.horrC2_thick = [stick.nodes.horrC2_thick, nodesH_thick(:,2:end)];
            %             stick.nodes.horr_thick   = [stick.nodes.horr_thick, nodesh_thick(:,2:end)];
            stick.nodes.horr_thick   = stick.nodes.horrC2_thick;
        end
        % Calculate position of nodes and mid-points along the stick beam line
        geo.htail.y_nodes_thick = nodeinterp(stick.nodes.horrC2_thick);
        %         if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        %             geo.htail.y_nodes_thick(2:end) = geo.htail.y_nodes_thick(2:end)-geo.htail.y_nodes_thick(2);
        %         end
        geo.htail.y_nodes_1_2_thick = meancouple(geo.htail.y_nodes_thick);
        
        % Beam length for structural elements
        for i = 1:length(geo.htail.y_nodes_thick) - 1
            stick.htail.Lbeam_thick(i,1) = norm( stick.nodes.horrC2_thick(:,i+1) - stick.nodes.horrC2_thick(:,i) );
        end
        
    end
    
    for j = 1:length(geo.htail.CAERO1.n_coarse)
        
        nodesH = DeltaElem(stick.ptos.horrC2(:,j:j+1), geo.htail.CAERO1.n_coarse(j));
        %         nodesh = DeltaElem(stick.ptos.horr(:,j:j+1), geo.htail.CAERO1.n_coarse(j));
        
        if j == 1
            stick.nodes.horrC2 = [stick.nodes.horrC2, nodesH];
            %             stick.nodes.horr   = [stick.nodes.horr, nodesh];
            stick.nodes.horr = stick.nodes.horrC2;
        else
            stick.nodes.horrC2 = [stick.nodes.horrC2, nodesH(:,2:end)];
            %             stick.nodes.horr   = [stick.nodes.horr, nodesh(:,2:end)];
            stick.nodes.horr = stick.nodes.horrC2;
        end
        % Calculate position of nodes and mid-points along the stick beam line
        geo.htail.y_nodes = nodeinterp(stick.nodes.horrC2);
        %         if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        %             geo.htail.y_nodes(2:end) = geo.htail.y_nodes(2:end)-geo.htail.y_nodes(2);
        %         end
        geo.htail.y_nodes_1_2 = meancouple(geo.htail.y_nodes);
        
        % Beam lenght for structural elements
        for i = 1:length(geo.htail.y_nodes) - 1
            stick.htail.Lbeam(i,1) = norm( stick.nodes.horrC2(:,i+1) - stick.nodes.horrC2(:,i) );
        end
        
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.hori.str_rec_coef.Coord = zeros(3,2*length(stick.PID.hori));
    
    % Volume computation (structural box volume and total volume)
    % Zs = str. box chord, r = aerodynamic chord
    geo.htail.V  = meancouple(geo.htail.Zs).*meancouple(geo.htail.tbs).*diff(geo.htail.y_nodes_thick);
    geo.htail.Vt = meancouple(geo.htail.r).*meancouple(geo.htail.tbs).*diff(geo.htail.y_nodes_thick);
    geo.htail.cg = geo.htail.y_nodes_1_2_thick;
    
end
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Canard
if isequal(stick.model.canr, 1)
    
    for j = 1:length(geo.canard.CAERO1.n)
        
        nodesH_thick = DeltaElem(stick.ptos.canrC2(:,j:j+1), geo.canard.CAERO1.n(j));
%         nodesh_thick = DeltaElem(stick.ptos.canr(:,j:j+1), geo.canard.CAERO1.n(j));
        
        if j == 1
            stick.nodes.canrC2_thick = [stick.nodes.canrC2_thick, nodesH_thick];
%             stick.nodes.canr_thick   = [stick.nodes.canr_thick, nodesh_thick];
            stick.nodes.canr_thick   = stick.nodes.canrC2_thick;
        else
            stick.nodes.canrC2_thick = [stick.nodes.canrC2_thick, nodesH_thick(:,2:end)];
%             stick.nodes.canr_thick   = [stick.nodes.canr_thick, nodesh_thick(:,2:end)];
            stick.nodes.canr_thick   = stick.nodes.canrC2_thick;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line        
        geo.canard.y_nodes_thick     = nodeinterp(stick.nodes.canr_thick);
        geo.canard.y_nodes_1_2_thick = meancouple(geo.canard.y_nodes_thick);
        
        % Beam length for structural elements
        for i = 1:length(geo.canard.y_nodes_thick) - 1
            stick.canr.Lbeam_thick(i,1) = norm( stick.nodes.canrC2_thick(:,i+1) - stick.nodes.canrC2_thick(:,i) );
        end
        
    end
    
    for j = 1:length(geo.canard.CAERO1.n_coarse)
        
        nodesH = DeltaElem(stick.ptos.canrC2(:,j:j+1), geo.canard.CAERO1.n_coarse(j));
%         nodesh = DeltaElem(stick.ptos.canr(:,j:j+1), geo.canard.CAERO1.n_coarse(j));
        
        if j == 1
            stick.nodes.canrC2 = [stick.nodes.canrC2, nodesH];
%             stick.nodes.canr   = [stick.nodes.canr, nodesh];
            stick.nodes.canr   = stick.nodes.canrC2;
        else
            stick.nodes.canrC2 = [stick.nodes.canrC2, nodesH(:,2:end)];
%             stick.nodes.canr = [stick.nodes.canr, nodesh(:,2:end)];
            stick.nodes.canr   = stick.nodes.canrC2;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line
        geo.canard.y_nodes     = nodeinterp(stick.nodes.canr);
        geo.canard.y_nodes_1_2 = meancouple(geo.canard.y_nodes);
%         geo.canard.y_nodes(2:end)=geo.canard.y_nodes(2:end)-geo.canard.y_nodes(2);
%         geo.canard.y_nodes_1_2 = (geo.canard.y_nodes(2:end)-geo.canard.y_nodes(1:end-1))./2 + geo.canard.y_nodes(1:end-1);
        
        % Beam length for structural elements
        for i = 1:length(geo.canard.y_nodes) - 1
            stick.canr.Lbeam(i,1) = norm( stick.nodes.canrC2(:,i+1) - stick.nodes.canrC2(:,i) );
        end
        
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.canr.str_rec_coef.Coord = zeros(3,2*length(stick.PID.canr));
    
    % Volume computation (structural box volume and total volume)
    % Zs = str. box chord, r = aerodynamic chord
    geo.canard.V  = meancouple(geo.canard.Zs).*meancouple(geo.canard.tbs).*diff(geo.canard.y_nodes_thick);
    geo.canard.Vt = meancouple(geo.canard.r).*meancouple(geo.canard.tbs).*diff(geo.canard.y_nodes_thick);
    geo.canard.cg = geo.canard.y_nodes_1_2_thick;
    
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Wing2
if isequal(stick.model.win2r, 1)
  
    % Cycle on the number of sectors defined inside wing2
    for i = 1:length(geo.wing2.CAERO1.n)
        
        nodesW_thick = DeltaElem(stick.ptos.win2rC2(:,i:i+1), geo.wing2.CAERO1.n(i));
%         nodesQ_thick = DeltaElem(stick.ptos.win2r(:,i:i+1), geo.wing2.CAERO1.n(i));
        
        if i == 1
            stick.nodes.win2rC2_thick = [stick.nodes.win2rC2_thick, nodesW_thick];
%             stick.nodes.win2r_thick = [stick.nodes.win2r_thick, nodesQ_thick];
            stick.nodes.win2r_thick   = stick.nodes.win2rC2_thick;
            
        else
            stick.nodes.win2rC2_thick = [stick.nodes.win2rC2_thick, nodesW_thick(:,2:end)];
%             stick.nodes.win2r_thick = [stick.nodes.win2r_thick, nodesQ_thick(:,2:end)];        
            stick.nodes.win2r_thick   = stick.nodes.win2rC2_thick;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line
        geo.wing2.y_nodes_thick     = nodeinterp(stick.nodes.win2r_thick);
        geo.wing2.y_nodes_1_2_thick = meancouple(geo.wing2.y_nodes_thick); 
        
        % Beam length for structural elements
        for j = 1:length(geo.wing2.y_nodes_thick) - 1
            stick.wing2.Lbeam_thick(j,1) = norm( stick.nodes.win2rC2_thick(:,j+1) - stick.nodes.win2rC2_thick(:,j) );
        end
        
    end
    
    for i = 1:length(geo.wing2.CAERO1.n_coarse)
        
        nodesW = DeltaElem(stick.ptos.win2rC2(:,i:i+1), geo.wing2.CAERO1.n_coarse(i));
%         nodesQ = DeltaElem(stick.ptos.win2r(:,i:i+1), geo.wing2.CAERO1.n_coarse(i));
        
        if i == 1          
            stick.nodes.win2rC2 = [stick.nodes.win2rC2, nodesW];
%             stick.nodes.win2r   = [stick.nodes.win2r, nodesQ];
            stick.nodes.win2r   = stick.nodes.win2rC2;
        else  
            stick.nodes.win2rC2 = [stick.nodes.win2rC2, nodesW(:,2:end)];
%             stick.nodes.win2r   = [stick.nodes.win2r, nodesQ(:,2:end)];
            stick.nodes.win2r   = stick.nodes.win2rC2;
        end
        
        % Calculate position of nodes and mid-points along the stick beam line
        geo.wing2.y_nodes     = nodeinterp(stick.nodes.win2r);
        geo.wing2.y_nodes_1_2 = meancouple(geo.wing2.y_nodes);
        
        % Beam length for structural elements
        for j = 1:length(geo.wing2.y_nodes) - 1
            stick.wing2.Lbeam(j,1) = norm( stick.nodes.win2rC2(:,j+1) - stick.nodes.win2rC2(:,j) );
        end
        
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.wing2.str_rec_coef.Coord = zeros(3,2*length(stick.PID.wing2));
    
    % Volume computation (structural box volume and total volume)
    % Zs = str. box chord, r = aerodynamic chord
    geo.wing2.V  = meancouple(geo.wing2.Zs).*meancouple(geo.wing2.tbs).*diff(geo.wing2.y_nodes_thick);
    geo.wing2.Vt = meancouple(geo.wing2.r).*meancouple(geo.wing2.tbs).*diff(geo.wing2.y_nodes_thick);
    geo.wing2.cg = geo.wing2.y_nodes_1_2_thick;
    
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Tail booms
if isequal(stick.model.tboomsr, 1)
    
    stick.nodes.tboomsr_thick = DeltaElem(stick.ptos.tbooms, geo.tbooms.CAERO1.n);
    stick.nodes.tboomsr       = DeltaElem(stick.ptos.tbooms, geo.tbooms.CAERO1.n_coarse);
    
    % Calculate position of nodes and mid-points along the stick beam line
    geo.tbooms.x_nodes_thick     = nodeinterp(stick.nodes.tboomsr_thick);
    geo.tbooms.x_nodes_1_2_thick = meancouple(geo.tbooms.x_nodes_thick);
    
    % Calculate position of nodes and mid-points along the stick beam line
    geo.tbooms.x_nodes     = nodeinterp(stick.nodes.tboomsr);
    geo.tbooms.x_nodes_1_2 = meancouple(geo.tbooms.x_nodes);
    
    % Beam length for structural elements
    for i = 1:length(geo.tbooms.x_nodes) - 1
        stick.tbooms.Lbeam(i,1) = norm( stick.nodes.tboomsr(:,i+1) - stick.nodes.tboomsr(:,i) );
    end
    
    % Beam length for structural elements
    for i = 1:length(geo.tbooms.x_nodes_thick) - 1
        stick.tbooms.Lbeam_thick(i,1) = norm( stick.nodes.tboomsr_thick(:,i+1) - stick.nodes.tboomsr_thick(:,i) );
    end
    
    % Introduce coordinates for stress recovery coefficients
    stick.PBAR.tbooms.str_rec_coef.Coord = zeros(3, 2*length(stick.PID.tbooms));
    for i = 1:length(geo.tbooms.x_nodes) - 1
        t = i*2-1:i*2;
        % Absolute coordinates in physical position
        N1 = stick.nodes.tboomsr(:,i); N1 = N1';
        N2 = ( (stick.nodes.tboomsr(:,i))./2 ); N2 = N2';
        N3 = stick.nodes.tboomsr(:,i+1); N3 = N3';
        d = interp_colloc_pos(N1, N2, N3);
        stick.PBAR.tbooms.str_rec_coef.Coord(:,t) = d';
    end
    for i = 1:length(geo.tbooms.x_nodes_thick) - 1
        t = i*2-1:i*2;
        % Absolute coordinates in physical position
        N1 = stick.nodes.tboomsr_thick(:,i); N1 = N1';
        N2 = ( (stick.nodes.tboomsr_thick(:,i)+stick.nodes.tboomsr_thick(:,i+1))./2 ); N2 = N2';
        N3 = stick.nodes.tboomsr_thick(:,i+1); N3 = N3';
        d = interp_colloc_pos(N1, N2, N3);
        stick.PBAR.tbooms.str_rec_coef.Coord_thick(:,t) = d';
    end
    
    % Volume and CG calculation (single tailboom)
    geo.tbooms.V  = meancouple(geo.tbooms.As).*diff(geo.tbooms.x_nodes_thick);
    geo.tbooms.Vt = geo.tbooms.V;
    geo.tbooms.cg = geo.tbooms.x_nodes_1_2_thick;
    
end

end % end of Stick_Nodes.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------------------------------------