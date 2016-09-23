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
% Called by:    MainCode.m
% 
% Calls:        BULKdataGRID.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function writeGRID2file(outf, fid, stick, aircraft)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Node definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

fprintf(outf, '\n\tExporting node coordinates...');

% Collect all IDs & nodes
% if isequal(stick.model.canr,1)
%     all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.horr; stick.ID.horl; stick.ID.canr; stick.ID.canl];
%     all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.horrC2, stick.nodes.horlC2, ...
%                                                                                             stick.nodes.canrC2, stick.nodes.canlC2];
% else
%     all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.horr; stick.ID.horl];
%     all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.horrC2, stick.nodes.horlC2]; 
% end
% 
% if isequal(stick.model.horr,1)
%     all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.horr; stick.ID.horl; stick.ID.canr; stick.ID.canl];
%     all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.horrC2, stick.nodes.horlC2, ...
%                                                                                             stick.nodes.canrC2, stick.nodes.canlC2];
% else
all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2 ;stick.ID.canr; stick.ID.canl; stick.ID.horr; stick.ID.horl;stick.ID.tboomsr; stick.ID.tboomsl];
all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.canrC2, stick.nodes.canlC2,stick.nodes.horrC2, stick.nodes.horlC2, stick.nodes.tboomsr, stick.nodes.tboomsl];
% end


% if isequal(aircraft.Vertical_tail.Twin_tail, 1)
%     if isequal(stick.model.canr,1)
%         all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2; stick.ID.horr; stick.ID.horl; stick.ID.canr; stick.ID.canl];
%         all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.horrC2, stick.nodes.horlC2, ...
%             stick.nodes.canrC2, stick.nodes.canlC2];
%     else
%         all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2; stick.ID.horr; stick.ID.horl];
%         all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.horrC2, stick.nodes.horlC2];
%     end
% 
%     if isequal(stick.model.horr,1)
%         all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2; stick.ID.horr; stick.ID.horl; stick.ID.canr; stick.ID.canl];
%         all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.horrC2, stick.nodes.horlC2, ...
%             stick.nodes.canrC2, stick.nodes.canlC2];
%     else
%         all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2; stick.ID.canr; stick.ID.canl];
%         all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.canrC2, stick.nodes.canlC2];
%     end
% end
 

% Recognize repeated nodes
node2cut = [];
for i = 1:length(all_ID)
    for j = (i+1) : length(all_ID)
        if isequal(all_ID(i), all_ID(j))
            node2cut = [node2cut; i];
        end
    end
end

ID2print = all_ID;
ID2print(node2cut) = [];
nodes2print = all_nodes;
nodes2print(:, node2cut) = [];


for k = 1:length(ID2print)
    % GRID card
    BULKdataGRID(fid, ID2print(k), 0, nodes2print(1,k), nodes2print(2,k), nodes2print(3,k), 0, 0, 0);
end

fprintf(outf, 'done.');


