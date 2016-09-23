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
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
% 28/oct/2009     1.0     Travaglini       Creation
%
%**************************************************************************
%
% function guess
%
%   DESCRIPTION: This function defines the element RBE2 that linked the
%   different structures, starting from distribution of node of them
%
%**************************************************************************

function [stick] = Link_StructsT(stick,geo,aircraft)
cont = 1;

if stick.model.winr
    switch stick.link.wing
        case{'fuse'}
            % stick.nodes.fuse, stick.nodes.winrC2_thick
            % stick.ID.winr
            
            % Find the master node
            Ma(cont) = (stick.ID.fuse(abs(stick.nodes.fuse(1,:) - stick.nodes.winrC2(1,1))<eps));
            Ma_thick(cont) = (stick.ID.fuse_thick(abs(stick.nodes.fuse_thick(1,:) - stick.nodes.winrC2_thick(1,1))<eps));
            % Find slaves nodes
            if aircraft.wing1.configuration ~=2
                SL1 = (stick.ID.winr(abs(stick.nodes.winrC2(2,:) - stick.ptos.winrC2(2,2))<eps));
                SL1_thick = (stick.ID.winr_thick(stick.nodes.winrC2_thick(2,:) == stick.ptos.winrC2(2,2)));
                
                SL2 = (stick.ID.winl(stick.nodes.winlC2(2,:) == -stick.ptos.winrC2(2,2)));
                SL2_thick = (stick.ID.winl_thick(stick.nodes.winlC2_thick(2,:) == -stick.ptos.winrC2(2,2)));
            else
                stick.model.winl = 0;
                SL1 = (stick.ID.winr(stick.nodes.winrC2(2,:) == stick.ptos.winrC2(2,2)));
                SL1_thick =(stick.ID.winr_thick(stick.nodes.winrC2_thick(2,:) == stick.ptos.winrC2(2,2)));
                % check the possibility of coincident points (fuselage &
                % wing1)
                if abs(stick.nodes.winrC2(3,1)- stick.nodes.fuse(3,stick.nodes.fuse(1,:) == stick.nodes.winrC2(1,1)))<=geo.wing.CSR*geo.wing.Rt_root*0.01
                    SL2 = [];
                    SL2_thick = [];
                    stick.ID.winr(1) = Ma(cont);
                    stick.ID.winr_thick(1) = Ma_thick(cont);
                else
                    SL2 = stick.ID.winr(1);
                    SL2_thick = stick.ID.winr_thick(1);
                end
            end
            stick.link.RBE2(cont).slave = [SL1,SL2];
            stick.link_thick.RBE2(cont).slave = [SL1_thick,SL2_thick];
            %             stick.link.RBE2(cont).master = Ma(cont);
            %             stick.link_thick.RBE2(cont).master = Ma_thick(cont);
            stick.link.RBE2(cont).DOF = '1235';
            stick.link_thick.RBE2(cont).DOF = '1235';
            if isfield(aircraft,'Tailbooms')
                if aircraft.Tailbooms.present
                    cont = cont+1;
                    [DY, IND] = min(abs(stick.nodes.winrC2(2,:) -stick.nodes.tboomsr(2,1) ));
                    [dummy,IND_thick] = min(abs(stick.nodes.winrC2_thick(2,:) -stick.nodes.tboomsr(2,1) ));
                    
                    Ma(cont) = (stick.ID.winr(IND));
                    Ma_thick(cont) = (stick.ID.winr_thick(IND_thick) );
                    
                    SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.wing.X ));
                    SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.wing.X ));
                    % check the possibility of coincident nodes
                    if DY==0 && ~isempty(find(stick.nodes.tboomsr(3,:)) == stick.nodes.winrC2(3,IND ))
                        stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.wing.X ) = Ma(cont);
                        stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.wing.X ) = Ma_thick(cont);
                        Ma = Ma(1:cont-1);
                        Ma_thick = Ma_thick(1:cont-1);
                        cont = cont-1;
                    else
                        stick.link.RBE2(cont).slave = SL1;
                        stick.link_thick.RBE2(cont).slave = SL1_thick;
                        stick.link.RBE2(cont).DOF = '123456';
                        stick.link_thick.RBE2(cont).DOF = '123456';
                    end
                    
                    % left wing
                    cont = cont+1;
                    %                     Ma(cont) = (stick.ID.winl(stick.nodes.winlC2(2,:) == -(stick.nodes.tboomsr(2,1)+stick.wing.DY)) );
                    %                     Ma_thick(cont) = (stick.ID.winl_thick(stick.nodes.winlC2_thick(2,:) == -(stick.nodes.tboomsr_thick(2,1)+stick.wing.DY)) );
                    %                     SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.winlC2(1,stick.ID.winl==Ma(cont))));
                    %                     SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.winlC2_thick(1,stick.ID.winl_thick==Ma_thick(cont))));
                    %                     % check the possibility of coincident nodes
                    %                     if stick.wing.DY==0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.winlC2(3,stick.ID.winl==Ma(cont)) )
                    %                         stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.winlC2(1,stick.ID.winl==Ma(cont))) = Ma(cont);
                    %                         stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.winlC2_thick(1,stick.ID.winl_thick==Ma_thick(cont))) = Ma_thick(cont);
                    %                         Ma = Ma(1:cont-1);
                    %                         Ma_thick = Ma_thick(1:cont-1);
                    %                         cont = cont-1;
                    %                     else
                    %                         stick.link.RBE2(cont).slave = SL1;
                    %                         stick.link_thick.RBE2(cont).slave = SL1_thick;
                    %                         stick.link.RBE2(cont).DOF = '123456';
                    %                         stick.link_thick.RBE2(cont).DOF = '123456';
                    %                     end
                    [DY, IND] = min(abs(stick.nodes.winlC2(2,:) -stick.nodes.tboomsl(2,1) ));
                    [dummy,IND_thick] = min(abs(stick.nodes.winlC2_thick(2,:) -stick.nodes.tboomsl(2,1) ));
                    
                    Ma(cont) = (stick.ID.winl(IND));
                    Ma_thick(cont) = (stick.ID.winl_thick(IND_thick) );
                    
                    SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.wing.X ));
                    SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.wing.X ));
                    % check the possibility of coincident nodes
                    if DY==0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.winlC2(3,IND ))
                        stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.wing.X ) = Ma(cont);
                        stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.wing.X ) = Ma_thick(cont);
                        Ma = Ma(1:cont-1);
                        Ma_thick = Ma_thick(1:cont-1);
                        cont = cont-1;
                    else
                        stick.link.RBE2(cont).slave = SL1;
                        stick.link_thick.RBE2(cont).slave = SL1_thick;
                        stick.link.RBE2(cont).DOF = '123456';
                        stick.link_thick.RBE2(cont).DOF = '123456';
                    end
                    
                end
            end
            
            
        case{'tbooms'} % tailboom without fuselage
            [DY, IND] = min(abs(stick.nodes.winrC2(2,:) -stick.nodes.tboomsr(2,1) ));
            [dummy,IND_thick] = min(abs(stick.nodes.winrC2_thick(2,:) -stick.nodes.tboomsr(2,1) ));
            
            Ma(cont) = (stick.ID.winr(IND));
            Ma_thick(cont) = (stick.ID.winr_thick(IND_thick) );
            
            SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.wing.X ));
            SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.wing.X ));
            % check the possibility of coincident nodes
            if DY==0 && ~isempty(find(stick.nodes.tboomsr(3,:)) == stick.nodes.winrC2(3,IND ))
                stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.wing.X ) = Ma(cont);
                stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.wing.X ) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            
            % left wing
            cont = cont+1;
           
            [DY, IND] = min(abs(stick.nodes.winlC2(2,:) -stick.nodes.tboomsl(2,1) ));
            [dummy,IND_thick] = min(abs(stick.nodes.winlC2_thick(2,:) -stick.nodes.tboomsl(2,1) ));
            
            Ma(cont) = (stick.ID.winl(IND));
            Ma_thick(cont) = (stick.ID.winl_thick(IND_thick) );
            
            SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.wing.X ));
            SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.wing.X ));
            % check the possibility of coincident nodes
            if DY==0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.winlC2(3,IND ))
                stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.wing.X ) = Ma(cont);
                stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.wing.X ) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            
    end
end

if stick.model.vert
    switch stick.link.vert

        case{'wing'}
          if aircraft.Vertical_tail.Twin_tail
            cont = cont+1;
            np = size(stick.nodes.vert,2);
            max_dist = realmax;
            wnode_index = 0; tnode_index = 0;
            for i=1:np
              [ind, dist] = find_nearest_master(stick.nodes.vert(:,i), stick.nodes.winrC2);
              if (dist<max_dist)
                wnode_index = ind;
                tnode_index = i;
                max_dist = dist;
              end
            end
            Ma(cont) = stick.ID.winr(wnode_index);
            stick.link.RBE2(cont).slave = stick.ID.vert(tnode_index);
            stick.link.RBE2(cont).DOF = '123456';
            if  ~isempty(stick.ID.winl)
              Ma(cont+1) = stick.ID.winl(wnode_index);
              stick.link.RBE2(cont+1).slave = stick.ID.vert2(tnode_index);
              stick.link.RBE2(cont+1).DOF = '123456';
            end
            max_dist = realmax;
            wnode_index = 0; tnode_index = 0;
            np = size(stick.nodes.vert_thick,2);
            for i=1:np
              [ind, dist] = find_nearest_master(stick.nodes.vert_thick(:,i), stick.nodes.winrC2_thick);
              if (dist<max_dist)
                wnode_index = ind;
                tnode_index = i;
                max_dist = dist;
              end
            end
            Ma_thick(cont) = stick.ID.winr_thick(wnode_index);
            stick.link_thick.RBE2(cont).slave = stick.ID.vert_thick(tnode_index);
            stick.link_thick.RBE2(cont).DOF = '123456';
            if  ~isempty(stick.ID.winl)
              cont = cont+1;
              Ma_thick(cont) = stick.ID.winl_thick(wnode_index);
              stick.link_thick.RBE2(cont).slave = stick.ID.vert2_thick(tnode_index);
              stick.link_thick.RBE2(cont).DOF = '123456';
            end
          end

        case{'fuse'}

          if aircraft.Vertical_tail.Twin_tail

            cont = cont+1;
            np = size(stick.nodes.vert,2);
            max_dist = realmax;
            fnode_index = 0; tnode_index = 0;
            for i=1:np
              [ind, dist] = find_nearest_master(stick.nodes.vert(:,i), stick.nodes.fuse);
              if (dist<max_dist)
                fnode_index = ind;
                tnode_index = i;
                max_dist = dist;
              end
            end
            Ma(cont) = stick.ID.fuse(fnode_index);
            stick.link.RBE2(cont).slave = [stick.ID.vert(tnode_index), stick.ID.vert2(tnode_index)];
            stick.link.RBE2(cont).DOF = '123456';
            max_dist = realmax;
            fnode_index = 0; tnode_index = 0;
            np = size(stick.nodes.vert_thick,2);
            for i=1:np
              [ind, dist] = find_nearest_master(stick.nodes.vert_thick(:,i), stick.nodes.fuse_thick);
              if (dist<max_dist)
                fnode_index = ind;
                tnode_index = i;
                max_dist = dist;
              end
            end
            Ma_thick(cont) = stick.ID.fuse_thick(fnode_index);
            stick.link_thick.RBE2(cont).slave = [stick.ID.vert_thick(tnode_index), stick.ID.vert2_thick(tnode_index)];
            stick.link_thick.RBE2(cont).DOF = '123456';

          else
            cont = cont+1;
            % Find the master node
            Ma(cont) = (stick.ID.fuse(stick.nodes.fuse(1,:) == geo.vtail.link(1)));
            Ma_thick(cont) = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == geo.vtail.link(1)));
            % Find the slave
            SL1 = (stick.ID.vert(stick.nodes.vert(1,:) == geo.vtail.link(1) ));
            if length(SL1)>1
                MastZ = stick.nodes.fuse(3,stick.nodes.fuse(1,:) == geo.vtail.link(1));
                SLz = (stick.nodes.vert(3,stick.nodes.vert(1,:) == geo.vtail.link(1) ));
                [dummy,SL1bis] = min(abs(SLz-MastZ));
                SL1 = (SL1(SL1bis));
            end
            SL1_thick = (stick.ID.vert_thick(stick.nodes.vert_thick(1,:) == geo.vtail.link(1) ));
            if length(SL1_thick)>1
                MastZ = stick.nodes.fuse_thick(3, stick.nodes.fuse_thick(1,:) == geo.vtail.link(1));
                SLz = (stick.nodes.vert_thick(3, stick.nodes.vert_thick(1,:) == geo.vtail.link(1) ));
                [dummy,SL1bis_thick] = min(abs(SLz-MastZ));
                SL1_thick = (SL1_thick(SL1bis_thick));
            end
            
            % check the possibility of coincident point
            if abs(stick.nodes.fuse(3,stick.nodes.fuse(1,:) == geo.vtail.link(1)) - stick.nodes.vert(3,stick.nodes.vert(1,:) == geo.vtail.link(1)))<=geo.vtail.b*0.001
                stick.ID.vert(stick.nodes.vert(1,:) == geo.vtail.link(1)) = Ma(cont);
                stick.ID.vert_thick(stick.nodes.vert_thick(1,:) == geo.vtail.link(1) )= Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                %             stick.link.RBE2(cont).master = Ma(cont);
                %             stick.link_thick.RBE2(cont).master = Ma_thick(cont);
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
          end


        case{'hori'}
            cont = cont+1;
            Ma(cont) = (stick.ID.horr(stick.nodes.horrC2(2,:) == stick.nodes.vert(2,1)));
            MaN = (stick.nodes.horrC2(2,:) == stick.nodes.vert(2,1));
            Ma_thick(cont) = (stick.ID.horr_thick(stick.nodes.horrC2_thick(2,:) == stick.nodes.vert(2,1)));
            Ma_thickN = (stick.nodes.horrC2_thick(2,:) == stick.nodes.vert(2,1));
            
            SL1 = (stick.ID.vert(abs(stick.nodes.vert(3,:) - stick.nodes.horrC2(3,MaN)) < eps));
            SL1_thick = (stick.ID.vert_thick(abs(stick.nodes.vert_thick(3,:) - stick.nodes.horrC2_thick(3,Ma_thickN)) <eps ));
            
            if abs( stick.nodes.horrC2(1,MaN) - stick.nodes.vert(1,stick.nodes.vert(3,:) == stick.nodes.horrC2(3,MaN)) ) <= geo.htail.CRp*0.001
                stick.ID.vert(stick.nodes.vert(3,:) == stick.nodes.horrC2(3,MaN)) = Ma(cont);
                stick.ID.vert_thick(stick.nodes.vert_thick(3,:) == stick.nodes.horrC2_thick(3,Ma_thickN)) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            %  left vtail
            
            cont = cont+1;
            Ma(cont) = (stick.ID.horl(stick.nodes.horlC2(2,:) == stick.nodes.vert2(2,1)));
            MaN = (stick.nodes.horlC2(2,:) == stick.nodes.vert2(2,1));
            Ma_thick(cont) = (stick.ID.horl_thick(stick.nodes.horlC2_thick(2,:) == stick.nodes.vert2(2,1)));
            Ma_thickN = (stick.nodes.horlC2_thick(2,:) == stick.nodes.vert2(2,1));
            
            SL1 = (stick.ID.vert2(abs(stick.nodes.vert2(3,:) - stick.nodes.horlC2(3,MaN))<eps ));
            SL1_thick = (stick.ID.vert2_thick(abs(stick.nodes.vert2_thick(3,:) == stick.nodes.horlC2_thick(3,Ma_thickN))<eps ));
            
            if abs( stick.nodes.horlC2(1,MaN) - stick.nodes.vert2(1,stick.nodes.vert2(3,:) == stick.nodes.horlC2(3,MaN)) ) <= geo.htail.CRp*0.001
                stick.ID.vert2(stick.nodes.vert2(3,:) == stick.nodes.horlC2(3,MaN)) = Ma(cont);
                stick.ID.vert2_thick(stick.nodes.vert2_thick(3,:) == stick.nodes.horlC2_thick(3,Ma_thickN)) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
        case{'tbooms'} 
            cont = cont+1; 
            Ma(cont) = (stick.ID.vert(stick.nodes.vert(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ))); 
            MaN = (stick.nodes.vert(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ));
            Ma_thick(cont) = (stick.ID.vert_thick(stick.nodes.vert_thick(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ)));
            Ma_thickN = (stick.nodes.vert_thick(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ));
            
            SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.vert(1,MaN) ));
            SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.vert_thick(1,Ma_thickN) ));
            
            if abs( stick.nodes.vert(3,MaN) - stick.nodes.tboomsr(3,stick.nodes.tboomsr(1,:) == stick.nodes.vert(1,MaN)) ) <= geo.htail.CRp*0.001
                stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.vert(1,MaN)) = Ma(cont);
                stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.vert_thick(1,Ma_thickN)) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            %  left vtail
            cont = cont+1;
            Ma(cont) = (stick.ID.vert2(stick.nodes.vert(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ)));
            MaN = (stick.nodes.vert2(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ));
            Ma_thick(cont) = (stick.ID.vert2_thick(stick.nodes.vert_thick(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ)));
            Ma_thickN = (stick.nodes.vert2_thick(3,:) == (stick.nodes.tboomsr(3,1)+stick.vtail.DZ));
            
            SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.vert2(1,MaN) ));
            SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.vert2_thick(1,Ma_thickN) ));
            
            if abs( stick.nodes.vert2(3,MaN) - stick.nodes.tboomsl(3,stick.nodes.tboomsl(1,:) == stick.nodes.vert2(1,MaN)) ) <= geo.htail.CRp*0.001
                stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.vert2(1,MaN)) = Ma(cont);
                stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.vert2_thick(1,Ma_thickN)) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
    end
end

if stick.model.horr
    switch stick.link.hori
        case {'vert'}
            if ~aircraft.Vertical_tail.Twin_tail
                cont = cont+1;
                Ma(cont) = (stick.ID.vert(stick.nodes.vert(3,:) == stick.nodes.horrC2(3,1)));
                Ma_thick(cont) = (stick.ID.vert_thick(stick.nodes.vert_thick(3,:) == stick.nodes.horrC2(3,1)));
                
                SL1 = stick.ID.horr(1);
                SL1_thick = stick.ID.horr_thick(1);
                
                if abs( stick.nodes.horrC2(1,1) - stick.nodes.vert(1,stick.nodes.vert(3,:) == stick.nodes.horrC2(3,1)) ) <= geo.vtail.CR*0.001
                    stick.ID.horr(1) = Ma(cont);
                    stick.ID.horr_thick(1) = Ma_thick(cont);
                    if stick.model.horl
                        stick.ID.horl(1) = Ma(cont);
                        stick.ID.horl_thick(1) = Ma_thick(cont);
                    end
                    Ma = Ma(1:cont-1);
                    Ma_thick = Ma_thick(1:cont-1);
                    cont = cont-1;
                else
                    % check if Master node on vertical tail is a slave node
                    % in link between fuselage and vtail (linked before)
                    if Ma(cont) == stick.link.RBE2(cont-1).slave;
                        Ma = Ma(1:cont-1);
                        stick.link.RBE2(cont-1).slave = [stick.link.RBE2(cont-1).slave,SL1];
                        Ma_thick = Ma_thick(1:cont-1);
                        stick.link_thick.RBE2(cont-1).slave = [stick.link_thick.RBE2(cont-1).slave,SL1_thick];
                        cont = cont-1;
                    else
                        stick.link.RBE2(cont).slave = SL1;
                        stick.link_thick.RBE2(cont).slave = SL1_thick;
                        stick.link.RBE2(cont).DOF = '123456';
                        stick.link_thick.RBE2(cont).DOF = '123456';
                    end
                end
            else
                cont = cont+1;
                Ma(cont) = (stick.ID.horr(stick.nodes.horrC2(2,:) == stick.nodes.vert(2,1)));
                MaN = (stick.nodes.horrC2(2,:) == stick.nodes.vert(2,1));
                Ma_thick(cont) = (stick.ID.horr_thick(stick.nodes.horrC2_thick(2,:) == stick.nodes.vert(2,1)));
                Ma_thickN = (stick.nodes.horrC2_thick(2,:) == stick.nodes.vert(2,1));
                
                SL1 = (stick.ID.vert(stick.nodes.vert(3,:) == stick.nodes.horrC2(3,MaN) ));
                SL1_thick = (stick.ID.vert_thick(stick.nodes.vert_thick(3,:) == stick.nodes.horrC2_thick(3,Ma_thickN) ));
                
                if abs( stick.nodes.horrC2(1,MaN) - stick.nodes.vert(1,stick.nodes.vert(3,:) == stick.nodes.horrC2(3,MaN)) ) <= geo.htail.CRp*0.001
                    stick.ID.vert(stick.nodes.vert(3,:) == stick.nodes.horrC2(3,MaN)) = Ma(cont);
                    stick.ID.vert_thick(stick.nodes.vert_thick(3,:) == stick.nodes.horrC2_thick(3,Ma_thickN)) = Ma_thick(cont);
                    Ma = Ma(1:cont-1);
                    Ma_thick = Ma_thick(1:cont-1);
                    cont = cont-1;
                else
                    stick.link.RBE2(cont).slave = SL1;
                    stick.link_thick.RBE2(cont).slave = SL1_thick;
                    stick.link.RBE2(cont).DOF = '123456';
                    stick.link_thick.RBE2(cont).DOF = '123456';
                end
                %  left vtail
                
                cont = cont+1;
                Ma(cont) = (stick.ID.horl(stick.nodes.horlC2(2,:) == stick.nodes.vert2(2,1)));
                MaN = (stick.nodes.horlC2(2,:) == stick.nodes.vert2(2,1));
                Ma_thick(cont) = (stick.ID.horl_thick(stick.nodes.horlC2_thick(2,:) == stick.nodes.vert2(2,1)));
                Ma_thickN = (stick.nodes.horlC2_thick(2,:) == stick.nodes.vert2(2,1));
                
                SL1 = (stick.ID.vert2(stick.nodes.vert2(3,:) == stick.nodes.horlC2(3,MaN) ));
                SL1_thick = (stick.ID.vert2_thick(stick.nodes.vert2_thick(3,:) == stick.nodes.horlC2_thick(3,Ma_thickN) ));
                
                if abs( stick.nodes.horlC2(1,MaN) - stick.nodes.vert2(1,stick.nodes.vert2(3,:) == stick.nodes.horlC2(3,MaN)) ) <= geo.htail.CRp*0.001
                    stick.ID.vert2(stick.nodes.vert2(3,:) == stick.nodes.horlC2(3,MaN)) = Ma(cont);
                    stick.ID.vert2_thick(stick.nodes.vert2_thick(3,:) == stick.nodes.horlC2_thick(3,Ma_thickN)) = Ma_thick(cont);
                    Ma = Ma(1:cont-1);
                    Ma_thick = Ma_thick(1:cont-1);
                    cont = cont-1;
                else
                    stick.link.RBE2(cont).slave = SL1;
                    stick.link_thick.RBE2(cont).slave = SL1_thick;
                    stick.link.RBE2(cont).DOF = '123456';
                    stick.link_thick.RBE2(cont).DOF = '123456';
                end
            end
            
        case {'fuse'}
            
            if isfield(stick.htail,'DX')
                
                Mah = (stick.ID.fuse(stick.nodes.fuse(1,:) == stick.nodes.horrC2(1,1) + stick.htail.DX));
                Mah_thick = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == stick.nodes.horrC2(1,1)+ stick.htail.DX));
                SL1 = stick.ID.horr(1);
                SL1_thick = stick.ID.horr_thick(1);
                % check if this master node is the same that link vtail
                if isempty(find(Mah == Ma,1))
                    cont = cont+1;
                    Ma(cont) = Mah;
                    Ma_thick(cont) = Mah_thick;
                    stick.link.RBE2(cont).slave = SL1;
                    stick.link_thick.RBE2(cont).slave = SL1_thick;
                    stick.link.RBE2(cont).DOF = '123456';
                    stick.link_thick.RBE2(cont).DOF = '123456';
                else
                    IND = find(Mah == Ma);
                    stick.link.RBE2(IND).slave =[stick.link.RBE2(IND).slave, SL1];
                    IND = find(Mah_thick == Ma_thick);
                    stick.link_thick.RBE2(IND).slave =[stick.link_thick.RBE2(IND).slave, SL1_thick];
                end
            else
                Mah = (stick.ID.fuse(stick.nodes.fuse(1,:) == stick.nodes.horrC2(1,1)));
                Mah_thick = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == stick.nodes.horrC2(1,1)));
                SL1 = stick.ID.horr(1);
                SL1_thick = stick.ID.horr_thick(1);
                % check if this master node is the same that link vtail
                if isempty(find(Mah == Ma,1))
                    cont = cont+1;
                    Ma(cont) = Mah;
                    Ma_thick(cont) = Mah_thick;
                    if abs(stick.nodes.fuse(3,stick.nodes.fuse(1,:) == stick.nodes.horrC2(1,1) )-stick.nodes.horrC2(3,1))<=geo.htail.CRp*geo.htail.Rt_root*0.01
                        stick.ID.horr(1) = Mah;
                        stick.ID.horr_thick(1)= Mah_thick;
                        if stick.model.horl
                            stick.ID.horl(1) = Ma(cont);
                            stick.ID.horl_thick(1) = Ma_thick(cont);
                        end
                        Ma = Ma(1:cont-1);
                        Ma_thick = Ma_thick(1:cont-1);
                        cont = cont-1;
                    else
                        stick.link.RBE2(cont).slave = SL1;
                        stick.link_thick.RBE2(cont).slave = SL1_thick;
                        stick.link.RBE2(cont).DOF = '123456';
                        stick.link_thick.RBE2(cont).DOF = '123456';
                    end
                else
                    IND = find(Mah == Ma);
                    if abs(stick.nodes.fuse(3,stick.nodes.fuse(1,:) == stick.nodes.horrC2(1,1) )-stick.nodes.horrC2(3,1))<=geo.htail.CRp*geo.htail.Rt_root*0.01
                        stick.ID.horr(1) = Mah;
                        stick.ID.horr_thick(1)= Mah_thick;
                        if stick.model.horl
                            stick.ID.horl(1) = Ma(cont);
                            stick.ID.horl_thick(1) = Ma_thick(cont);
                        end
                    else
                        stick.link.RBE2(IND).slave =[stick.link.RBE2(IND).slave, SL1];
                        IND = find(Mah_thick == Ma_thick);
                        stick.link_thick.RBE2(IND).slave =[stick.link_thick.RBE2(IND).slave, SL1_thick];
                    end
                end
            end
        case{'tbooms'}
            cont = cont+1;
            [DY, IND] = min(abs(stick.nodes.horrC2(2,:) -stick.nodes.tboomsr(2,1) ));
            [dummy,IND_thick] = min(abs(stick.nodes.horrC2_thick(2,:) -stick.nodes.tboomsr(2,1) ));
            
            Ma(cont) = (stick.ID.horr(IND));
            Ma_thick(cont) = (stick.ID.horr_thick(IND_thick) );
            
            SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.htail.X ));
            SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.htail.X ));
            
            if DY == 0 && ~isempty(find(stick.nodes.tboomsr(3,:)) == stick.nodes.horrC2(3,IND ))
                stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.htail.X ) = Ma(cont);
                stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.htail.X ) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            %  left htail
            %             cont = cont+1;
            %             Ma(cont) = (stick.ID.horl(stick.nodes.horlC2(2,:) == -(stick.nodes.tboomsr(2,1)+stick.htail.DY)));
            %             MaN = (stick.nodes.horlC2(2,:) == -(stick.nodes.tboomsr(2,1)+stick.htail.DY));
            %             Ma_thick(cont) = (stick.ID.horl_thick(stick.nodes.horlC2_thick(2,:) == -(stick.nodes.tboomsr(2,1)+stick.htail.DY)));
            %             Ma_thickN = (stick.nodes.horlC2_thick(2,:) == -(stick.nodes.tboomsr(2,1)+stick.htail.DY));
            %
            %             SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.horlC2(1,MaN) ));
            %             SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.horlC2_thick(1,Ma_thickN) ));
            %
            %             if abs( stick.nodes.horlC2(1,MaN) - stick.nodes.tboomsl(1,stick.nodes.tboomsl(1,:) == stick.nodes.horlC2(1,MaN)) ) <= geo.htail.CRp*0.001
            %                 stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.horlC2(1,MaN)) = Ma(cont);
            %                 stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.horlC2_thick(1,Ma_thickN)) = Ma_thick(cont);
            %                 Ma = Ma(1:cont-1);
            %                 Ma_thick = Ma_thick(1:cont-1);
            %                 cont = cont-1;
            %             else
            %                 stick.link.RBE2(cont).slave = SL1;
            %                 stick.link_thick.RBE2(cont).slave = SL1_thick;
            %                 stick.link.RBE2(cont).DOF = '123456';
            %                 stick.link_thick.RBE2(cont).DOF = '123456';
            %             end
            cont = cont+1;
            [DY, IND] = min(abs(stick.nodes.horlC2(2,:) -stick.nodes.tboomsl(2,1) ));
            [dummy,IND_thick] = min(abs(stick.nodes.horlC2_thick(2,:) -stick.nodes.tboomsl(2,1) ));
            
            Ma(cont) = (stick.ID.horl(IND));
            Ma_thick(cont) = (stick.ID.horl_thick(IND_thick) );
            
            SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.htail.X ));
            SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.htail.X ));
            
            if DY == 0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.horlC2(3,IND ))
                stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.htail.X ) = Ma(cont);
                stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.htail.X ) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
    end
end

if stick.model.canr
    cont = cont+1;
    Ma(cont) = (stick.ID.fuse(stick.nodes.fuse(1,:) == stick.nodes.canrC2(1,1)));
    Ma_thick(cont) = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == stick.nodes.canrC2_thick(1,1)));
    
    SL1 = (stick.ID.canr(stick.nodes.canrC2(2,:) == stick.ptos.canrC2(2,2)));
    SL1_thick = (stick.ID.canr_thick(stick.nodes.canrC2_thick(2,:) == stick.ptos.canrC2(2,2)));
    
    SL2 = (stick.ID.canl(stick.nodes.canlC2(2,:) == -stick.ptos.canrC2(2,2)));
    SL2_thick = (stick.ID.canl_thick(stick.nodes.canlC2_thick(2,:) == -stick.ptos.canrC2(2,2)));
    
    stick.link.RBE2(cont).slave = [SL1,SL2];
    stick.link_thick.RBE2(cont).slave = [SL1_thick,SL2_thick];
    stick.link.RBE2(cont).DOF = '1235';
    stick.link_thick.RBE2(cont).DOF = '1235';
end

if stick.model.win2r
    switch stick.link.wing2
        case{'fuse'}
            % Find the master node
            if isfield(stick.wing2,'DX')
                Mah = (stick.ID.fuse(stick.nodes.fuse(1,:) == stick.nodes.win2rC2(1,1) + stick.wing2.DX));
                Mah_thick = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == stick.nodes.win2rC2_thick(1,1)+ stick.wing2.DX));
                % Find slaves nodes
                if aircraft.wing1.configuration ~=2
                    SL1 =(stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.ptos.win2rC2(2,2)));
                    SL1_thick = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.ptos.win2rC2(2,2)));
                    
                    SL2 = (stick.ID.win2l(stick.nodes.win2lC2(2,:) == -stick.ptos.win2rC2(2,2)));
                    SL2_thick = (stick.ID.win2l_thick(stick.nodes.win2lC2_thick(2,:) == -stick.ptos.win2rC2(2,2)));
                else
                    stick.model.win2l = 0;
                    SL1 = (stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.ptos.win2rC2(2,2)));
                    SL1_thick = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.ptos.win2rC2(2,2)));
                    SL2 = stick.ID.win2r(1);
                    SL2_thick = stick.ID.win2r_thick(1);
                    
                end
                % check if this master node is the same of wing1
                if isempty(find(Mah == Ma,1))
                    cont = cont+1;
                    Ma(cont) = Mah;
                    Ma_thick(cont) = Mah_thick;
                    stick.link.RBE2(cont).slave = [SL1,SL2];
                    stick.link_thick.RBE2(cont).slave = [SL1_thick,SL2_thick];
                    stick.link.RBE2(cont).DOF = '1235';
                    stick.link_thick.RBE2(cont).DOF = '1235';
                else
                    IND = find(Mah == Ma);
                    stick.link.RBE2(IND).slave =[stick.link.RBE2(IND).slave,SL1,SL2];
                    IND = find(Mah_thick == Ma_thick);
                    stick.link_thick.RBE2(IND).slave =[stick.link_thick.RBE2(IND).slave, SL1_thick,SL2_thick];
                end
            else
                Mah = (stick.ID.fuse(stick.nodes.fuse(1,:) == stick.nodes.win2rC2(1,1)));
                Mah_thick = (stick.ID.fuse_thick(stick.nodes.fuse_thick(1,:) == stick.nodes.win2rC2_thick(1,1)));
                % Find slaves nodes
                if aircraft.wing1.configuration ~=2
                    SL1 =(stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.ptos.win2rC2(2,2)));
                    SL1_thick = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.ptos.win2rC2(2,2)));
                    
                    SL2 = (stick.ID.win2l(stick.nodes.win2lC2(2,:) == -stick.ptos.win2rC2(2,2)));
                    SL2_thick = (stick.ID.win2l_thick(stick.nodes.win2lC2_thick(2,:) == -stick.ptos.win2rC2(2,2)));
                else
                    stick.model.win2l = 0;
                    SL1 = (stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.ptos.win2rC2(2,2)));
                    SL1_thick = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.ptos.win2rC2(2,2)));
                    if abs(stick.nodes.win2rC2(3,1)- stick.nodes.fuse(3,stick.nodes.fuse(1,:) == stick.nodes.win2rC2(1,1)))<=geo.wing2.CSR*geo.wing2.Rt_root*0.01
                        SL2 = [];
                        SL2_thick = [];
                        stick.ID.win2r(1) = Mah;
                        stick.ID.win2r_thick(1) = Mah_thick;
                    else
                        SL2 = stick.ID.win2r(1);
                        SL2_thick = stick.ID.win2r_thick(1);
                    end
                end
                % check if this master node is the same of wing1
                if isempty(find(Mah == Ma,1))
                    cont = cont+1;
                    Ma(cont) = Mah;
                    Ma_thick(cont) = Mah_thick;
                    stick.link.RBE2(cont).slave = [SL1,SL2];
                    stick.link_thick.RBE2(cont).slave = [SL1_thick,SL2_thick];
                    stick.link.RBE2(cont).DOF = '1235';
                    stick.link_thick.RBE2(cont).DOF = '1235';
                else
                    IND = find(Mah == Ma);
                    stick.link.RBE2(IND).slave =[stick.link.RBE2(IND).slave,SL1,SL2];
                    IND = find(Mah_thick == Ma_thick);
                    stick.link_thick.RBE2(IND).slave =[stick.link_thick.RBE2(IND).slave, SL1_thick,SL2_thick];
                end
            end
            if isfield(aircraft,'Tailbooms')
                if aircraft.Tailbooms.present
                    cont = cont+1;
                    Ma(cont) = (stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.nodes.tboomsr(2,1))+stick.wing2.DY );
                    Ma_thick(cont) = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.nodes.tboomsr_thick(2,1))+stick.wing2.DY );
                    SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.win2rC2(1,stick.ID.win2r==Ma(cont))));
                    SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.win2rC2_thick(1,stick.ID.win2r_thick==Ma_thick(cont))));
                    % check the possibility of coincident nodes
                    if stick.wing2.DY==0 && ~isempty(find(stick.nodes.tboomsr(3,:)) == stick.nodes.win2rC2(3,stick.ID.win2r==Ma(cont)) )
                        stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.win2rC2(1,stick.ID.win2r==Ma(cont))) = Ma(cont);
                        stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.win2rC2_thick(1,stick.ID.win2r_thick==Ma_thick(cont))) = Ma_thick(cont);
                        Ma = Ma(1:cont-1);
                        Ma_thick = Ma_thick(1:cont-1);
                        cont = cont-1;
                    else
                        stick.link.RBE2(cont).slave = SL1;
                        stick.link_thick.RBE2(cont).slave = SL1_thick;
                        stick.link.RBE2(cont).DOF = '123456';
                        stick.link_thick.RBE2(cont).DOF = '123456';
                    end
                    
                     % left wing
                     cont = cont+1;
                     Ma(cont) = (stick.ID.win2l(stick.nodes.win2lC2(2,:) == -(stick.nodes.tboomsr(2,1))+stick.wing2.DY) );
                     Ma_thick(cont) = (stick.ID.win2l_thick(stick.nodes.win2lC2_thick(2,:) == -(stick.nodes.tboomsr_thick(2,1))+stick.wing2.DY) );
                     SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.win2lC2(1,stick.ID.win2l==Ma(cont))));
                     SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.win2lC2_thick(1,stick.ID.win2l_thick==Ma_thick(cont))));
                     % check the possibility of coincident nodes
                     if stick.wing2.DY==0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.win2lC2(3,stick.ID.win2l==Ma(cont)) )
                         stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.win2lC2(1,stick.ID.win2l==Ma(cont))) = Ma(cont);
                         stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.win2lC2_thick(1,stick.ID.win2l_thick==Ma_thick(cont))) = Ma_thick(cont);
                         Ma = Ma(1:cont-1);
                         Ma_thick = Ma_thick(1:cont-1);
                     else
                         stick.link.RBE2(cont).slave = SL1;
                         stick.link_thick.RBE2(cont).slave = SL1_thick;
                         stick.link.RBE2(cont).DOF = '123456';
                         stick.link_thick.RBE2(cont).DOF = '123456';
                     end
                    
                end
            end
            
         case{'tbooms'} % tailboom without fuselage
            Ma(cont) = (stick.ID.win2r(stick.nodes.win2rC2(2,:) == stick.nodes.tboomsr(2,1))+stick.wing2.DY );
            Ma_thick(cont) = (stick.ID.win2r_thick(stick.nodes.win2rC2_thick(2,:) == stick.nodes.tboomsr_thick(2,1))+stick.wing2.DY );
            SL1 = (stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.win2rC2(1,stick.ID.win2r==Ma(cont))));
            SL1_thick = (stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.win2rC2_thick(1,stick.ID.win2r_thick==Ma_thick(cont))));
            % check the possibility of coincident nodes
            if stick.wing2.DY==0 && ~isempty(find(stick.nodes.tboomsr(3,:)) == stick.nodes.win2rC2(3,stick.ID.win2r==Ma(cont)) )
                stick.ID.tboomsr(stick.nodes.tboomsr(1,:) == stick.nodes.win2rC2(1,stick.ID.win2r==Ma(cont))) = Ma(cont);
                stick.ID.tboomsr_thick(stick.nodes.tboomsr_thick(1,:) == stick.nodes.win2rC2_thick(1,stick.ID.win2r_thick==Ma_thick(cont))) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
                cont = cont-1;
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            
            % left wing
            cont = cont+1;
            Ma(cont) = (stick.ID.win2l(stick.nodes.win2lC2(2,:) == -(stick.nodes.tboomsr(2,1))+stick.wing2.DY) );
            Ma_thick(cont) = (stick.ID.win2l_thick(stick.nodes.win2lC2_thick(2,:) == -(stick.nodes.tboomsr_thick(2,1))+stick.wing2.DY) );
            SL1 = (stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.win2lC2(1,stick.ID.win2l==Ma(cont))));
            SL1_thick = (stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.win2lC2_thick(1,stick.ID.win2l_thick==Ma_thick(cont))));
            % check the possibility of coincident nodes
            if stick.wing2.DY==0 && ~isempty(find(stick.nodes.tboomsl(3,:)) == stick.nodes.win2lC2(3,stick.ID.win2l==Ma(cont)) )
                stick.ID.tboomsl(stick.nodes.tboomsl(1,:) == stick.nodes.win2lC2(1,stick.ID.win2l==Ma(cont))) = Ma(cont);
                stick.ID.tboomsl_thick(stick.nodes.tboomsl_thick(1,:) == stick.nodes.win2lC2_thick(1,stick.ID.win2l_thick==Ma_thick(cont))) = Ma_thick(cont);
                Ma = Ma(1:cont-1);
                Ma_thick = Ma_thick(1:cont-1);
            else
                stick.link.RBE2(cont).slave = SL1;
                stick.link_thick.RBE2(cont).slave = SL1_thick;
                stick.link.RBE2(cont).DOF = '123456';
                stick.link_thick.RBE2(cont).DOF = '123456';
            end
            
    end
end
% extra links
for (i=1:length(stick.link.extra))
%
  switch(stick.link.extra{i})
    case ('jwing')
      if (aircraft.Horizontal_tail.present && aircraft.wing1.present)
        if (~isempty(stick.ID.winr) && ~isempty(stick.ID.horr))
          cont = cont +1;
          stick.link.RBE2(cont).slave = stick.ID.horr(end);
          stick.link_thick.RBE2(cont).slave = stick.ID.horr_thick(end);
          stick.link.RBE2(cont).DOF = '123456';
          stick.link_thick.RBE2(cont).DOF = '123456';
%         find nearest point
          [ind, ~] = find_nearest_master(stick.nodes.horrC2(:,end), stick.nodes.winrC2);
          Ma(cont) = stick.ID.winr(ind);
%         find nearest point on thick mesh
          [ind, ~] = find_nearest_master(stick.nodes.horrC2_thick(:,end), stick.nodes.winrC2_thick);
          Ma_thick(cont) = stick.ID.winr_thick(ind);
%         force connection to wing tip
%         Ma(cont) = (stick.ID.winr(end)); force connection to wing tip
%         Ma_thick(cont) = (stick.ID.winr_thick(end));
        end
        if (~isempty(stick.ID.winl) && ~isempty(stick.ID.horl))
          cont = cont +1;
          stick.link.RBE2(cont).slave = stick.ID.horl(end);
          stick.link_thick.RBE2(cont).slave = stick.ID.horl_thick(end);
          stick.link.RBE2(cont).DOF = '123456';
          stick.link_thick.RBE2(cont).DOF = '123456';
%         find nearest point
          [ind,~] = find_nearest_master(stick.nodes.horlC2(:,end), stick.nodes.winlC2);
          Ma(cont) = stick.ID.winl(ind);
%         find nearest point on thick mesh
          [ind, ~] = find_nearest_master(stick.nodes.horlC2_thick(:,end), stick.nodes.winlC2_thick);
          Ma_thick(cont) = stick.ID.winl_thick(ind);
%         Ma(cont) = (stick.ID.winl(end));
%         Ma_thick(cont) = (stick.ID.winl_thick(end));
        end
    end
% Stut braced wing
    case ('swing')
      if (aircraft.Canard.present && aircraft.wing1.present)
        if (~isempty(stick.ID.winr) && ~isempty(stick.ID.canr))
          cont = cont +1;
          stick.link.RBE2(cont).slave = stick.ID.canr(end);
          stick.link_thick.RBE2(cont).slave = stick.ID.canr_thick(end);
          stick.link.RBE2(cont).DOF = '123';
          stick.link_thick.RBE2(cont).DOF = '123';
%         find nearest point
          [ind, ~] = find_nearest_master(stick.nodes.canrC2(:,end), stick.nodes.winrC2);
          Ma(cont) = stick.ID.winr(ind);
%         find nearest point on thick mesh
          [ind, ~] = find_nearest_master(stick.nodes.canrC2_thick(:,end), stick.nodes.winrC2_thick);
          Ma_thick(cont) = stick.ID.winr_thick(ind);
%         force connection to wing tip
%         Ma(cont) = (stick.ID.winr(end)); force connection to wing tip
%         Ma_thick(cont) = (stick.ID.winr_thick(end));
        end
        if (~isempty(stick.ID.winl) && ~isempty(stick.ID.canl))
          cont = cont +1;
          stick.link.RBE2(cont).slave = stick.ID.canl(end);
          stick.link_thick.RBE2(cont).slave = stick.ID.canl_thick(end);
          stick.link.RBE2(cont).DOF = '123';
          stick.link_thick.RBE2(cont).DOF = '123';
%         find nearest point
          [ind,~] = find_nearest_master(stick.nodes.canlC2(:,end), stick.nodes.winlC2);
          Ma(cont) = stick.ID.winl(ind);
%         find nearest point on thick mesh
          [ind, ~] = find_nearest_master(stick.nodes.canlC2_thick(:,end), stick.nodes.winlC2_thick);
          Ma_thick(cont) = stick.ID.winl_thick(ind);
%         Ma(cont) = (stick.ID.winl(end));
%         Ma_thick(cont) = (stick.ID.winl_thick(end));
        end
      end
  end
%
end
%
stick.link.Ma = Ma;
stick.link_thick.Ma_thick = Ma_thick;
end
%-------------------------------------------------------------------------------
