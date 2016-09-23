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
% 2008-04-06
%
% slaves.m generates 4 slave nodes per each master node
%
%
% Called by:
%
% Calls:        Gen_Slaves_Wing.m, Gen_Slaves_Vert.m, Gen_Slaves_Hori.m, BULKdataGRID.m, BULKdataRB0.m
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [stick] = writeRBE02file(outf, fid, stick, geo, aircraft)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Aeronode definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

fprintf(outf, '\n\tExporting aeronode coordinates...');


%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% Fuselage
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.fuse, 1)
    %
    % Generate slave nodes
    [stick] = Gen_Slaves_Fus(stick, geo);
    % For each master node, write GRID card 4 times
    for i = 1:length(stick.ID.fuse)
        t = (i-1)*4 + 1;
        BULKdataGRID(fid,stick.slaves.ID.fuse(t),  0,stick.slaves.nodes.fuse(1,t),  stick.slaves.nodes.fuse(2,t),  stick.slaves.nodes.fuse(3,t)  ,0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.fuse(t+1),0,stick.slaves.nodes.fuse(1,t+1),stick.slaves.nodes.fuse(2,t+1),stick.slaves.nodes.fuse(3,t+1),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.fuse(t+2),0,stick.slaves.nodes.fuse(1,t+2),stick.slaves.nodes.fuse(2,t+2),stick.slaves.nodes.fuse(3,t+2),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.fuse(t+3),0,stick.slaves.nodes.fuse(1,t+3),stick.slaves.nodes.fuse(2,t+3),stick.slaves.nodes.fuse(3,t+3),0,0,0);
    end
    %
    % For each node master, write RB0 card
    for i = 1:length(stick.ID.fuse)
        t = (i-1)*4 + 1;
        % ID number for RBE0 card has been chosen equal to ID for master node
        BULKdataRB0(fid,stick.ID.fuse(i),stick.ID.fuse(i),stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3));
%         if isempty(find(stick.ID.fuse(i) == stick.link.Ma,1))
%             BULKdataRB0(fid,stick.ID.fuse(i),stick.ID.fuse(i),stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3));
%         else
%             BULKdataRB0T(fid,stick.ID.fuse(i),stick.ID.fuse(i),[stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.fuse(i)).slave]);
%         end
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.winr, 1)
    
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    % Generate slave nodes
    [stick] = Gen_Slaves_Wing(stick, geo);
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
     
    % For each master node, write GRID card 4 times
    for i = 1:length(stick.ID.winr)
        t = (i-1)*4 + 1;
        BULKdataGRID(fid,stick.slaves.ID.winr(t)  ,0,stick.slaves.nodes.winr(1,t)  ,stick.slaves.nodes.winr(2,t)  ,stick.slaves.nodes.winr(3,t)  ,0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.winr(t+1),0,stick.slaves.nodes.winr(1,t+1),stick.slaves.nodes.winr(2,t+1),stick.slaves.nodes.winr(3,t+1),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.winr(t+2),0,stick.slaves.nodes.winr(1,t+2),stick.slaves.nodes.winr(2,t+2),stick.slaves.nodes.winr(3,t+2),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.winr(t+3),0,stick.slaves.nodes.winr(1,t+3),stick.slaves.nodes.winr(2,t+3),stick.slaves.nodes.winr(3,t+3),0,0,0);
    end
    %
    % For each node master, write RB0 card
    for i = 1:length(stick.ID.winr)
        t = (i-1)*4 + 1;
        % ID number for RBE0 card has been chosen equal to ID for master
        % node
        BULKdataRB0(fid,stick.ID.winr(i),stick.ID.winr(i),stick.slaves.ID.winr(t),stick.slaves.ID.winr(t+1),stick.slaves.ID.winr(t+2),stick.slaves.ID.winr(t+3));
    end
    %
    if isequal(stick.model.winl, 1)
        %
        % For each master node, write GRID card 4 times
        for i = 2:length(stick.ID.winr)     % exclude 1st node because it is already done
            t = (i-1)*4 + 1;
            BULKdataGRID(fid,stick.slaves.ID.winl(t)  ,0,stick.slaves.nodes.winl(1,t)  ,stick.slaves.nodes.winl(2,t)  ,stick.slaves.nodes.winl(3,t)  ,0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.winl(t+1),0,stick.slaves.nodes.winl(1,t+1),stick.slaves.nodes.winl(2,t+1),stick.slaves.nodes.winl(3,t+1),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.winl(t+2),0,stick.slaves.nodes.winl(1,t+2),stick.slaves.nodes.winl(2,t+2),stick.slaves.nodes.winl(3,t+2),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.winl(t+3),0,stick.slaves.nodes.winl(1,t+3),stick.slaves.nodes.winl(2,t+3),stick.slaves.nodes.winl(3,t+3),0,0,0);
        end
        %
        % For each node master, write RB0 card
        for i = 2:length(stick.ID.winr)     % exclude 1st node because it is already done
            t = (i-1)*4 + 1;
            % ID number for RBE0 card has been chosen equal to ID for master node
            BULKdataRB0(fid,stick.ID.winl(i),stick.ID.winl(i),stick.slaves.ID.winl(t),stick.slaves.ID.winl(t+1),stick.slaves.ID.winl(t+2),stick.slaves.ID.winl(t+3));
        end
        
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------


%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% Vertical tail
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.vert, 1)
    
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    % Generate slave nodes
    [stick] = Gen_Slaves_Vert(stick, geo);
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        stick.slaves.nodes.vert2=stick.slaves.nodes.vert;
        stick.slaves.nodes.vert2(2,:)=-stick.slaves.nodes.vert(2,:);
        stick.slaves.ID.vert2=stick.slaves.ID.vert+5000;
    end
    
    % For each master node, write GRID card 4 times
    for i = 1:length(stick.slaves.ID.vert)/4
        t = (i-1)*4 + 1;
        BULKdataGRID(fid,stick.slaves.ID.vert(t),  0,stick.slaves.nodes.vert(1,t),  stick.slaves.nodes.vert(2,t),  stick.slaves.nodes.vert(3,t)  ,0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.vert(t+1),0,stick.slaves.nodes.vert(1,t+1),stick.slaves.nodes.vert(2,t+1),stick.slaves.nodes.vert(3,t+1),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.vert(t+2),0,stick.slaves.nodes.vert(1,t+2),stick.slaves.nodes.vert(2,t+2),stick.slaves.nodes.vert(3,t+2),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.vert(t+3),0,stick.slaves.nodes.vert(1,t+3),stick.slaves.nodes.vert(2,t+3),stick.slaves.nodes.vert(3,t+3),0,0,0);
        if isequal(aircraft.Vertical_tail.Twin_tail, 1)
            BULKdataGRID(fid,stick.slaves.ID.vert2(t),  0,stick.slaves.nodes.vert2(1,t),  stick.slaves.nodes.vert2(2,t),  stick.slaves.nodes.vert2(3,t)  ,0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.vert2(t+1),0,stick.slaves.nodes.vert2(1,t+1),stick.slaves.nodes.vert2(2,t+1),stick.slaves.nodes.vert2(3,t+1),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.vert2(t+2),0,stick.slaves.nodes.vert2(1,t+2),stick.slaves.nodes.vert2(2,t+2),stick.slaves.nodes.vert2(3,t+2),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.vert2(t+3),0,stick.slaves.nodes.vert2(1,t+3),stick.slaves.nodes.vert2(2,t+3),stick.slaves.nodes.vert2(3,t+3),0,0,0);
        end
        
        
    end
    % For each node master, write RB0 card
    for i = 1:length(stick.slaves.ID.vert)/4
        t = (i-1)*4 + 1;
%         strind = find(stick.slaves.nodes.vert(3,1) == stick.nodes.vert(3,:));
        % ID number for RBE0 card has been chosen equal to ID for master node
        BULKdataRB0(fid,stick.ID.vert(i),stick.ID.vert(i),stick.slaves.ID.vert(t),stick.slaves.ID.vert(t+1),stick.slaves.ID.vert(t+2),stick.slaves.ID.vert(t+3));
%         if isempty(find(stick.ID.vert(i) == stick.link.Ma,1))
%             BULKdataRB0(fid,stick.ID.vert(i),stick.ID.vert(i),stick.slaves.ID.vert(t),stick.slaves.ID.vert(t+1),stick.slaves.ID.vert(t+2),stick.slaves.ID.vert(t+3));
%         else
%             BULKdataRB0T(fid,stick.ID.vert(i),stick.ID.vert(i),[stick.slaves.ID.vert(t),stick.slaves.ID.vert(t+1),stick.slaves.ID.vert(t+2),stick.slaves.ID.vert(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.vert(i)).slave]);
%         end
%         BULKdataRB0(fid,stick.ID.vert(i),stick.ID.vert(i),stick.slaves.ID.vert(t),stick.slaves.ID.vert(t+1),stick.slaves.ID.vert(t+2),stick.slaves.ID.vert(t+3));
        if isequal(aircraft.Vertical_tail.Twin_tail, 1)
%             strind2 = find(stick.slaves.nodes.vert2(3,1) == stick.nodes.vert2(3,:));
            % ID number for RBE0 card has been chosen equal to ID for master node
            BULKdataRB0(fid,stick.ID.vert2(i),stick.ID.vert2(i),stick.slaves.ID.vert2(t),stick.slaves.ID.vert2(t+1),stick.slaves.ID.vert2(t+2),stick.slaves.ID.vert2(t+3));
%             if isempty(find(stick.ID.vert2(i) == stick.link.Ma,1))
%                 BULKdataRB0(fid,stick.ID.vert2(i),stick.ID.vert2(i),stick.slaves.ID.vert2(t),stick.slaves.ID.vert2(t+1),stick.slaves.ID.vert2(t+2),stick.slaves.ID.vert2(t+3));
%             else
%                 BULKdataRB0T(fid,stick.ID.vert2(i),stick.ID.vert2(i),[stick.slaves.ID.vert2(t),stick.slaves.ID.vert2(t+1),stick.slaves.ID.vert2(t+2),stick.slaves.ID.vert2(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.vert2(i)).slave]);
%             end
        end
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------



%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.horr, 1)
    
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    % Generate slave nodes
    [stick] = Gen_Slaves_Hori(stick, geo);
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    
    % For each master node, write GRID card 4 times
    for i = 1:length(stick.ID.horr)
        t = (i-1)*4 + 1;
        BULKdataGRID(fid,stick.slaves.ID.horr(t)  ,0,stick.slaves.nodes.horr(1,t)  ,stick.slaves.nodes.horr(2,t)  ,stick.slaves.nodes.horr(3,t)  ,0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.horr(t+1),0,stick.slaves.nodes.horr(1,t+1),stick.slaves.nodes.horr(2,t+1),stick.slaves.nodes.horr(3,t+1),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.horr(t+2),0,stick.slaves.nodes.horr(1,t+2),stick.slaves.nodes.horr(2,t+2),stick.slaves.nodes.horr(3,t+2),0,0,0);
        BULKdataGRID(fid,stick.slaves.ID.horr(t+3),0,stick.slaves.nodes.horr(1,t+3),stick.slaves.nodes.horr(2,t+3),stick.slaves.nodes.horr(3,t+3),0,0,0);
    end
    % For each node master, write RB0 card
    for i = 1:length(stick.ID.horr)
        t = (i-1)*4 + 1;
        % ID number for RBE0 card has been chosen equal to ID for master node
        BULKdataRB0(fid,stick.ID.horr(i),stick.ID.horr(i),stick.slaves.ID.horr(t),stick.slaves.ID.horr(t+1),stick.slaves.ID.horr(t+2),stick.slaves.ID.horr(t+3));
%         if isempty(find(stick.ID.horr(i) == stick.link.Ma,1))
%             BULKdataRB0(fid,stick.ID.horr(i),stick.ID.horr(i),stick.slaves.ID.horr(t),stick.slaves.ID.horr(t+1),stick.slaves.ID.horr(t+2),stick.slaves.ID.horr(t+3));
%         else
%             BULKdataRB0T(fid,stick.ID.horr(i),stick.ID.horr(i),[stick.slaves.ID.horr(t),stick.slaves.ID.horr(t+1),stick.slaves.ID.horr(t+2),stick.slaves.ID.horr(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.horr(i)).slave]);
%         end
    end
    %
    if isequal(stick.model.horl, 1)
        %
        % For each master node, write GRID card 4 times
        for i = 2:length(stick.ID.horr)     % exclude 1st node because it is already done
            t = (i-1)*4 + 1;
            BULKdataGRID(fid,stick.slaves.ID.horl(t)  ,0,stick.slaves.nodes.horl(1,t)  ,stick.slaves.nodes.horl(2,t)  ,stick.slaves.nodes.horl(3,t)  ,0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.horl(t+1),0,stick.slaves.nodes.horl(1,t+1),stick.slaves.nodes.horl(2,t+1),stick.slaves.nodes.horl(3,t+1),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.horl(t+2),0,stick.slaves.nodes.horl(1,t+2),stick.slaves.nodes.horl(2,t+2),stick.slaves.nodes.horl(3,t+2),0,0,0);
            BULKdataGRID(fid,stick.slaves.ID.horl(t+3),0,stick.slaves.nodes.horl(1,t+3),stick.slaves.nodes.horl(2,t+3),stick.slaves.nodes.horl(3,t+3),0,0,0);
        end
        % For each node master, write RB0 card
        for i = 2:length(stick.ID.horr)     % exclude 1st node because it is already done
            t = (i-1)*4 + 1;
            % ID number for RBE0 card has been chosen equal to ID for master node
            BULKdataRB0(fid,stick.ID.horl(i),stick.ID.horl(i),stick.slaves.ID.horl(t),stick.slaves.ID.horl(t+1),stick.slaves.ID.horl(t+2),stick.slaves.ID.horl(t+3));
%             if isempty(find(stick.ID.horl(i) == stick.link.Ma,1))
%                 BULKdataRB0(fid,stick.ID.horl(i),stick.ID.horl(i),stick.slaves.ID.horl(t),stick.slaves.ID.horl(t+1),stick.slaves.ID.horl(t+2),stick.slaves.ID.horl(t+3));
%             else
%                 BULKdataRB0T(fid,stick.ID.horl(i),stick.ID.horl(i),[stick.slaves.ID.horl(t),stick.slaves.ID.horl(t+1),stick.slaves.ID.horl(t+2),stick.slaves.ID.horl(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.horl(i)).slave]);
%             end
        end
        
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------



%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.canr, 1)
    if (aircraft.Strut_wing.present~=1)
      %$$$$$$$$$$$$$$$$$$$$$$$$$$$$
      % Generate slave nodes
      [stick] = Gen_Slaves_Canard(stick, geo);
      %$$$$$$$$$$$$$$$$$$$$$$$$$$$$

      % For each master node, write GRID card 4 times
      for i = 1:length(stick.ID.canr)
          t = (i-1)*4 + 1;
          BULKdataGRID(fid,stick.slaves.ID.canr(t)  ,0,stick.slaves.nodes.canr(1,t)  ,stick.slaves.nodes.canr(2,t)  ,stick.slaves.nodes.canr(3,t)  ,0,0,0);
          BULKdataGRID(fid,stick.slaves.ID.canr(t+1),0,stick.slaves.nodes.canr(1,t+1),stick.slaves.nodes.canr(2,t+1),stick.slaves.nodes.canr(3,t+1),0,0,0);
          BULKdataGRID(fid,stick.slaves.ID.canr(t+2),0,stick.slaves.nodes.canr(1,t+2),stick.slaves.nodes.canr(2,t+2),stick.slaves.nodes.canr(3,t+2),0,0,0);
          BULKdataGRID(fid,stick.slaves.ID.canr(t+3),0,stick.slaves.nodes.canr(1,t+3),stick.slaves.nodes.canr(2,t+3),stick.slaves.nodes.canr(3,t+3),0,0,0);
      end
      %
      % For each node master, write RB0 card
      for i = 1:length(stick.ID.canr)
          t = (i-1)*4 + 1;
          % ID number for RBE0 card has been chosen equal to ID for master node
          BULKdataRB0(fid,stick.ID.canr(i),stick.ID.canr(i),stick.slaves.ID.canr(t),stick.slaves.ID.canr(t+1),stick.slaves.ID.canr(t+2),stick.slaves.ID.canr(t+3));
      end
      %
      if isequal(stick.model.canl, 1)
          %
          % For each master node, write GRID card 4 times
          for i = 2:length(stick.ID.canr)     % exclude 1st node because it is already done
              t = (i-1)*4 + 1;
              BULKdataGRID(fid,stick.slaves.ID.canl(t)  ,0,stick.slaves.nodes.canl(1,t)  ,stick.slaves.nodes.canl(2,t)  ,stick.slaves.nodes.canl(3,t)  ,0,0,0);
              BULKdataGRID(fid,stick.slaves.ID.canl(t+1),0,stick.slaves.nodes.canl(1,t+1),stick.slaves.nodes.canl(2,t+1),stick.slaves.nodes.canl(3,t+1),0,0,0);
              BULKdataGRID(fid,stick.slaves.ID.canl(t+2),0,stick.slaves.nodes.canl(1,t+2),stick.slaves.nodes.canl(2,t+2),stick.slaves.nodes.canl(3,t+2),0,0,0);
              BULKdataGRID(fid,stick.slaves.ID.canl(t+3),0,stick.slaves.nodes.canl(1,t+3),stick.slaves.nodes.canl(2,t+3),stick.slaves.nodes.canl(3,t+3),0,0,0);
          end
          %
          % For each node master, write RB0 card
          for i = 2:length(stick.ID.canr)     % exclude 1st node because it is already done
              t = (i-1)*4 + 1;
              % ID number for RBE0 card has been chosen equal to ID for master node
              BULKdataRB0(fid,stick.ID.canl(i),stick.ID.canl(i),stick.slaves.ID.canl(t),stick.slaves.ID.canl(t+1),stick.slaves.ID.canl(t+2),stick.slaves.ID.canl(t+3));
          end

      end
  end    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------------------------------------------
% tbooms
%--------------------------------------------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.tboomsr, 1)
    %
    % Generate slave nodes
    IDslavetb = 59999;
    % For each master node, write GRID card 4 times
    for i = 1:length(stick.ID.tboomsr)
        t = (i-1)*4 + 1;
        BULKdataGRID(fid,IDslavetb + t   ,0, stick.nodes.tboomsr(1,i),  stick.nodes.tboomsr(2,i),  stick.nodes.tboomsr(3,i)+geo.tbooms.R  ,0,0,0);
        BULKdataGRID(fid,IDslavetb + t+1 ,0, stick.nodes.tboomsr(1,i),  stick.nodes.tboomsr(2,i)-geo.tbooms.R,  stick.nodes.tboomsr(3,i) ,0,0,0);
        BULKdataGRID(fid,IDslavetb + t+2 ,0, stick.nodes.tboomsr(1,i),  stick.nodes.tboomsr(2,i),  stick.nodes.tboomsr(3,i)-geo.tbooms.R,0,0,0);
        BULKdataGRID(fid,IDslavetb + t+3 ,0, stick.nodes.tboomsr(1,i),  stick.nodes.tboomsr(2,i)+geo.tbooms.R,  stick.nodes.tboomsr(3,i),0,0,0);
    end
    %
    % For each node master, write RB0 card
    for i = 1:length(stick.ID.tboomsr)
        t = (i-1)*4 + 1;
        % ID number for RBE0 card has been chosen equal to ID for master node
        BULKdataRB0(fid,stick.ID.tboomsr(i),stick.ID.tboomsr(i),IDslavetb+t ,IDslavetb+t+1 ,IDslavetb+t+2 , IDslavetb+t+3);
%         if isempty(find(stick.ID.fuse(i) == stick.link.Ma,1))
%             BULKdataRB0(fid,stick.ID.fuse(i),stick.ID.fuse(i),stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3));
%         else
%             BULKdataRB0T(fid,stick.ID.fuse(i),stick.ID.fuse(i),[stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.fuse(i)).slave]);
%         end
    end
    
    if isequal(stick.model.tboomsl, 1)
        IDslavetb = IDslavetb+t+4;
        for i = 1:length(stick.ID.tboomsl)
            t = (i-1)*4 + 1;
            BULKdataGRID(fid,IDslavetb + t   ,0, stick.nodes.tboomsl(1,i),  stick.nodes.tboomsl(2,i),  stick.nodes.tboomsl(3,i)+geo.tbooms.R  ,0,0,0);
            BULKdataGRID(fid,IDslavetb + t+1 ,0, stick.nodes.tboomsl(1,i),  stick.nodes.tboomsl(2,i)-geo.tbooms.R,  stick.nodes.tboomsl(3,i) ,0,0,0);
            BULKdataGRID(fid,IDslavetb + t+2 ,0, stick.nodes.tboomsl(1,i),  stick.nodes.tboomsl(2,i),  stick.nodes.tboomsl(3,i)-geo.tbooms.R,0,0,0);
            BULKdataGRID(fid,IDslavetb + t+3 ,0, stick.nodes.tboomsl(1,i),  stick.nodes.tboomsl(2,i)+geo.tbooms.R,  stick.nodes.tboomsl(3,i),0,0,0);
        end
        %
        % For each node master, write RB0 card
        for i = 1:length(stick.ID.tboomsr)
            t = (i-1)*4 + 1;
            % ID number for RBE0 card has been chosen equal to ID for master node
            BULKdataRB0(fid,stick.ID.tboomsl(i),stick.ID.tboomsl(i),IDslavetb+t ,IDslavetb+t+1 ,IDslavetb+t+2 , IDslavetb+t+3);
            %         if isempty(find(stick.ID.fuse(i) == stick.link.Ma,1))
            %             BULKdataRB0(fid,stick.ID.fuse(i),stick.ID.fuse(i),stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3));
            %         else
            %             BULKdataRB0T(fid,stick.ID.fuse(i),stick.ID.fuse(i),[stick.slaves.ID.fuse(t),stick.slaves.ID.fuse(t+1),stick.slaves.ID.fuse(t+2),stick.slaves.ID.fuse(t+3),stick.link.RBE2(stick.link.Ma ==stick.ID.fuse(i)).slave]);
            %         end
        end
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------------------------------------------


fprintf(outf, 'done.');
