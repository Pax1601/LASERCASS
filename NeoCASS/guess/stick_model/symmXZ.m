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

function [stick] = symmXZ(stick,aircraft)
%--------------------------------------------------------------------------------------------------
%
%
%
% Called by:    Stick_Model.m
%
% Calls:
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Mofified by Travaglini 19/11/2009 Different type ofwing configurations is
% introduced, but is still in working
if isequal(stick.model.winr, 1)
    OFFSET = 50;
    % Left semi-wing is now defined, checking the configuration of wing1
    if aircraft.wing1.configuration == 0 || aircraft.wing1.configuration>2
        stick.model.winl = 1;
        
        % Define ID numbers for CAERO1 panels
%        stick.IDCAERO1.winl = (stick.IDCAERO1.winr(end)+1 : stick.IDCAERO1.winr(end)+length(stick.IDCAERO1.winr))';
        stick.IDCAERO1.winl = stick.IDCAERO1.winr+OFFSET;
        
        % Basic stick model points
        stick.ptos.winl = stick.ptos.winr;
        stick.ptos.winl(2,:) = -stick.ptos.winl(2,:);
        %
        stick.ptos.winlC2 = stick.ptos.winrC2;
        stick.ptos.winlC2(2,:) = -stick.ptos.winlC2(2,:);
        %
        stick.nodes.winlC2 = stick.nodes.winrC2;
        stick.nodes.winlC2(2,:) = -stick.nodes.winlC2(2,:);
        %
        stick.nodes.winl = stick.nodes.winr;
        stick.nodes.winl(2,:) = -stick.nodes.winl(2,:);
        
        %
        stick.nodes.winlC2_thick = stick.nodes.winrC2_thick;
        stick.nodes.winlC2_thick(2,:) = -stick.nodes.winlC2_thick(2,:);
        %
        stick.nodes.winl_thick = stick.nodes.winr_thick;
        stick.nodes.winl_thick(2,:) = -stick.nodes.winl_thick(2,:);
    elseif aircraft.wing1.configuration == 1
        % non symmetric wing
        stick.model.winl = 1;
        % ................................................................MODIFICA........................................................
        % Define ID numbers for CAERO1 panels
%        stick.IDCAERO1.winl = (stick.IDCAERO1.winr(end)+1 : stick.IDCAERO1.winr(end)+length(stick.IDCAERO1.winr))';
        stick.IDCAERO1.winl = stick.IDCAERO1.winr+OFFSET;
        % Basic stick model points
        stick.ptos.winl = stick.ptos.winr;
        stick.ptos.winl(2,:) = -stick.ptos.winl(2,:);
        %
        stick.ptos.winlC2 = stick.ptos.winrC2;
        stick.ptos.winlC2(2,:) = -stick.ptos.winlC2(2,:);
        %
        stick.nodes.winlC2 = stick.nodes.winrC2;
        stick.nodes.winlC2(2,:) = -stick.nodes.winlC2(2,:);
        %
        stick.nodes.winl = stick.nodes.winr;
        stick.nodes.winl(2,:) = -stick.nodes.winl(2,:);
        
        %
        stick.nodes.winlC2_thick = stick.nodes.winrC2_thick;
        stick.nodes.winlC2_thick(2,:) = -stick.nodes.winlC2_thick(2,:);
        %
        stick.nodes.winl_thick = stick.nodes.winr_thick;
        stick.nodes.winl_thick(2,:) = -stick.nodes.winl_thick(2,:);
        
        
    elseif aircraft.wing1.configuration == 2
        % only semi-right wing1 is present
        stick.model.winl = 0;
        
    end
end

if isequal(stick.model.horr, 1)
    
    % Left semi-wing is now defined
    stick.model.horl = 1;
    
    % Define ID numbers for CAERO1 panels
    stick.IDCAERO1.horl = stick.IDCAERO1.horr+OFFSET;
    % Basic stick model points
    stick.ptos.horl = stick.ptos.horr;
    stick.ptos.horl(2,:) = -stick.ptos.horl(2,:);
    %
    stick.ptos.horlC2 = stick.ptos.horrC2;
    stick.ptos.horlC2(2,:) = -stick.ptos.horlC2(2,:);
    %
    stick.nodes.horlC2 = stick.nodes.horrC2;
    stick.nodes.horlC2(2,:) = -stick.nodes.horlC2(2,:);
    %
    stick.nodes.horl = stick.nodes.horr;
    stick.nodes.horl(2,:) = -stick.nodes.horl(2,:);
    
    stick.nodes.horlC2_thick = stick.nodes.horrC2_thick;
    stick.nodes.horlC2_thick(2,:) = -stick.nodes.horlC2_thick(2,:);
    %
    stick.nodes.horl_thick = stick.nodes.horr_thick;
    stick.nodes.horl_thick(2,:) = -stick.nodes.horl_thick(2,:);
    
end



if isequal(stick.model.canr, 1)
    
    % Left semi-wing is now defined
    stick.model.canl = 1;
    
    % Define ID numbers for CAERO1 panels
    stick.IDCAERO1.canl = stick.IDCAERO1.canr+OFFSET;
    % Basic stick model points
    stick.ptos.canl = stick.ptos.canr;
    stick.ptos.canl(2,:) = -stick.ptos.canl(2,:);
    %
    stick.ptos.canlC2 = stick.ptos.canrC2;
    stick.ptos.canlC2(2,:) = -stick.ptos.canlC2(2,:);
    %
    stick.nodes.canlC2 = stick.nodes.canrC2;
    stick.nodes.canlC2(2,:) = -stick.nodes.canlC2(2,:);
    %
    stick.nodes.canl = stick.nodes.canr;
    stick.nodes.canl(2,:) = -stick.nodes.canl(2,:);
    
    
    stick.nodes.canlC2_thick = stick.nodes.canrC2_thick;
    stick.nodes.canlC2_thick(2,:) = -stick.nodes.canlC2_thick(2,:);
    %
    stick.nodes.canl_thick = stick.nodes.canr_thick;
    stick.nodes.canl_thick(2,:) = -stick.nodes.canl_thick(2,:);
    
    
end


if isequal(stick.model.win2r, 1)
    
    if aircraft.wing2.configuration == 0 || aircraft.wing2.configuration>2
        % Left semi-wing is now defined
        stick.model.win2l = 1;
        % Define ID numbers for CAERO1 panels
        offset = stick.IDCAERO1.win2r(1)+50;
        stick.IDCAERO1.win2l = stick.IDCAERO1.win2r+OFFSET;
        % Basic stick model points
        stick.ptos.win2l = stick.ptos.win2r;
        stick.ptos.win2l(2,:) = -stick.ptos.win2l(2,:);
        %
        stick.ptos.win2lC2 = stick.ptos.win2rC2;
        stick.ptos.win2lC2(2,:) = -stick.ptos.win2lC2(2,:);
        %
        stick.nodes.win2lC2 = stick.nodes.win2rC2;
        stick.nodes.win2lC2(2,:) = -stick.nodes.win2lC2(2,:);
        %
        stick.nodes.win2l = stick.nodes.win2r;
        stick.nodes.win2l(2,:) = -stick.nodes.win2l(2,:);
        
        %
        stick.nodes.win2lC2_thick = stick.nodes.win2rC2_thick;
        stick.nodes.win2lC2_thick(2,:) = -stick.nodes.win2lC2_thick(2,:);
        %
        stick.nodes.win2l_thick = stick.nodes.win2r_thick;
        stick.nodes.win2l_thick(2,:) = -stick.nodes.win2l_thick(2,:);
    elseif aircraft.wing2.configuration == 1
        stick.model.win2l = 1;
        
        % Define ID numbers for CAERO1 panels
        stick.IDCAERO1.win2l = stick.IDCAERO1.win2r+OFFSET;
        % Basic stick model points
        stick.ptos.win2l = stick.ptos.win2r;
        stick.ptos.win2l(2,:) = -stick.ptos.win2l(2,:);
        %
        stick.ptos.win2lC2 = stick.ptos.win2rC2;
        stick.ptos.win2lC2(2,:) = -stick.ptos.win2lC2(2,:);
        %
        stick.nodes.win2lC2 = stick.nodes.win2rC2;
        stick.nodes.win2lC2(2,:) = -stick.nodes.win2lC2(2,:);
        %
        stick.nodes.win2l = stick.nodes.win2r;
        stick.nodes.win2l(2,:) = -stick.nodes.win2l(2,:);
        
        %
        stick.nodes.win2lC2_thick = stick.nodes.win2rC2_thick;
        stick.nodes.win2lC2_thick(2,:) = -stick.nodes.win2lC2_thick(2,:);
        %
        stick.nodes.win2l_thick = stick.nodes.win2r_thick;
        stick.nodes.win2l_thick(2,:) = -stick.nodes.win2l_thick(2,:);
    elseif aircraft.wing2.configuration == 2
        stick.model.win2l = 0;
    end
end


if isequal(aircraft.Vertical_tail.present,1) && isequal(aircraft.Vertical_tail.Twin_tail, 1)
    
    % Left semi-wing is now defined
    % stick.model.canl = 1;
    
    % Define ID numbers for CAERO1 panels
    offset = stick.IDCAERO1.vert(1)+50;
    stick.IDCAERO1.vert2 = stick.IDCAERO1.vert+OFFSET;
    % Basic stick model points
    stick.ptos.vert2 = stick.ptos.vert;
    stick.ptos.vert2(2,:) = -stick.ptos.vert(2,:);
    
    
    stick.ptospanel.vert2 = stick.ptospanel.vert;
    stick.ptospanel.vert2(2,:) = -stick.ptospanel.vert(2,:);
    %
    
    %         stick.ptos.vert2C2 = stick.ptos.vertC2;
    %         stick.ptos.vert2C2(2,:) = -stick.ptos.vertC2(2,:);
    %         %
    %         stick.nodes.vert2C2 = stick.nodes.vertC2;
    %         stick.nodes.vert2C2(2,:) = -stick.nodes.vertC2(2,:);
    %
    stick.nodes.vert2 = stick.nodes.vert;
    stick.nodes.vert2(2,:) = -stick.nodes.vert(2,:);
    
    
    %         stick.nodes.vert2C2_thick = stick.nodes.vertC2_thick;
    %         stick.nodes.vert2C2_thick(2,:) = -stick.nodes.vertC2_thick(2,:);
    %
    stick.nodes.vert2_thick = stick.nodes.vert_thick;
    stick.nodes.vert2_thick(2,:) = -stick.nodes.vert_thick(2,:);
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%

% Tail booms

if isequal(stick.model.tboomsr, 1)
    
    % Left semi-wing is now defined
    stick.model.tboomsl = 1;
    
%     % Define ID numbers for CAERO1 panels
%     stick.IDCAERO1.horl = (stick.IDCAERO1.horr(end)+1 : stick.IDCAERO1.horr(end)+length(stick.IDCAERO1.horr))';
    
    % Basic stick model points
    stick.ptos.tboomsl = stick.ptos.tbooms;
    stick.ptos.tboomsl(2,:) = -stick.ptos.tbooms(2,:);
    %
    stick.nodes.tboomsl = stick.nodes.tboomsr;
    stick.nodes.tboomsl(2,:) = -stick.nodes.tboomsr(2,:);
    %
    
    stick.nodes.tboomsl_thick = stick.nodes.tboomsr_thick;
    stick.nodes.tboomsl_thick(2,:) = -stick.nodes.tboomsl_thick(2,:);
    
end


% if isequal(stick.model.winr, 1)
%
%     % Left semi-wing is now defined
%     stick.model.winl = 1;
%
%     % Define ID numbers for CAERO1 panels
%     stick.IDCAERO1.winl = (stick.IDCAERO1.winr(end)+1 : stick.IDCAERO1.winr(end)+length(stick.IDCAERO1.winr))';
%
%     % Basic stick model points
%     stick.ptos.winl = stick.ptos.winr;
%     stick.ptos.winl(2,:) = -stick.ptos.winl(2,:);
%     %
%     stick.ptos.winlC2 = stick.ptos.winrC2;
%     stick.ptos.winlC2(2,:) = -stick.ptos.winlC2(2,:);
%     %
%     stick.nodes.winlC2 = stick.nodes.winrC2;
%     stick.nodes.winlC2(2,:) = -stick.nodes.winlC2(2,:);
%     %
%     stick.nodes.winl = stick.nodes.winr;
%     stick.nodes.winl(2,:) = -stick.nodes.winl(2,:);
%
% end

%
% if isequal(stick.model.horr, 1)
%
%     % Left semi-wing is now defined
%     stick.model.horl = 1;
%
%     % Define ID numbers for CAERO1 panels
%     stick.IDCAERO1.horl = (stick.IDCAERO1.horr(end)+1 : stick.IDCAERO1.horr(end)+length(stick.IDCAERO1.horr))';
%
%     % Basic stick model points
%     stick.ptos.horl = stick.ptos.horr;
%     stick.ptos.horl(2,:) = -stick.ptos.horl(2,:);
%     %
%     stick.ptos.horlC2 = stick.ptos.horrC2;
%     stick.ptos.horlC2(2,:) = -stick.ptos.horlC2(2,:);
%     %
%     stick.nodes.horlC2 = stick.nodes.horrC2;
%     stick.nodes.horlC2(2,:) = -stick.nodes.horlC2(2,:);
%     %
%     stick.nodes.horl = stick.nodes.horr;
%     stick.nodes.horl(2,:) = -stick.nodes.horl(2,:);
%
% end


%
% if isequal(stick.model.canr, 1)
%
%     % Left semi-wing is now defined
%     stick.model.canl = 1;
%
%     % Define ID numbers for CAERO1 panels
%     stick.IDCAERO1.canl = (stick.IDCAERO1.canr(end)+1 : stick.IDCAERO1.canr(end)+length(stick.IDCAERO1.canr))';
%
%     % Basic stick model points
%     stick.ptos.canl = stick.ptos.canr;
%     stick.ptos.canl(2,:) = -stick.ptos.canl(2,:);
%     %
%     stick.ptos.canlC2 = stick.ptos.canrC2;
%     stick.ptos.canlC2(2,:) = -stick.ptos.canlC2(2,:);
%     %
%     stick.nodes.canlC2 = stick.nodes.canrC2;
%     stick.nodes.canlC2(2,:) = -stick.nodes.canlC2(2,:);
%     %
%     stick.nodes.canl = stick.nodes.canr;
%     stick.nodes.canl(2,:) = -stick.nodes.canl(2,:);
%
% end

