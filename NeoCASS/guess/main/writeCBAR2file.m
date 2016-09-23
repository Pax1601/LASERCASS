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
%------------------------------------------------------------------------------------------------------------------------
% writeCBAR2file.m uses BULKdataCBAR.m to define a simple beam element
%
% Called by:    MainCode.m
%
% Calls:        VectOrien.m, BULKdataCBAR.m
%
%   <andreadr@kth.se>
%------------------------------------------------------------------------------------------------------------------------
% Modified by Travaglini 19/11/2009, cahnging the first element orientation
% on htail and vtail. Now the orientations is notchanged because there are
% not added element to extend the structure

function [stick] = writeCBAR2file(outf, fid, stick, geo, aircraft)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Beam definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

fprintf(outf, '\n\tExporting beam definition...');


%------------------------------------------------------------------------------------------------------------------------
% Fuselage
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.fuse, 1)
    
    % Generate orientation vector for fuselage
    [X1, X2, X3] = VectOrien(stick.nodes.fuse);
    stick.CBAR.fuse.v = [X1; X2; X3];
    
    % get rotation matrix
    [R] = rotation_matix_cbar(stick.nodes.fuse, stick.CBAR.fuse.v);
    stick.CBAR.fuse.R = R;
    
    % Set default parameters
    OFT = 'GGG';
    PA = 0;
    PB = 0;
    for i = 1:length(stick.EID.fuse)
        BULKdataCBAR(fid, stick.EID.fuse(i), stick.PID.fuse(i), stick.ID.fuse(i), stick.ID.fuse(i+1),...
            X1(i), X2(i), X3(i), OFT, PA, PB,...
            stick.OFFSET.fuse(1), stick.OFFSET.fuse(2), stick.OFFSET.fuse(3),...
            stick.OFFSET.fuse(1), stick.OFFSET.fuse(2), stick.OFFSET.fuse(3));
    end
    
end
%------------------------------------------------------------------------------------------------------------------------


%------------------------------------------------------------------------------------------------------------------------
% Wing
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.winr, 1)
    
    % Generate orientation vector for right semi-wing
    [X1, X2, X3] = orient_vect_cmps(stick.nodes.winrC2, geo.wing.WING, geo.wing.CAERO1.n);
    stick.CBAR.winr.v = [X1; X2; X3];
    
    % get rotation matrix
    [R] = rotation_matix_cbar(stick.nodes.winrC2, stick.CBAR.winr.v);
    stick.CBAR.winr.R = R;
    
    % Set default parameters
    OFT = 'GGG';
    for i = 1:length(stick.EID.winr)
        BULKdataCBAR(fid, stick.EID.winr(i), stick.PID.wing(i), stick.ID.winr(i), stick.ID.winr(i+1),...
            X1(i), X2(i), X3(i), OFT);
    end
    
    if isequal(stick.model.winl, 1)
        
        % Generate orientation vector for left semi-wing
        %        WING = geo.wing.WING;
        %        WING(2,:) = -WING(2,:);
        %        [X1, X2, X3] = orient_vect_cmps(stick.nodes.winlC2, WING, geo.wing.CAERO1.n);
        %        stick.CBAR.horl.v = [X1; X2; X3];
        
        % get rotation matrix
        %        [R] = rotation_matix_cbar(stick.nodes.winlC2, stick.CBAR.horl.v);
        stick.CBAR.horl.R = R;
        
        for i = 1:length(stick.EID.winl)
            BULKdataCBAR(fid, stick.EID.winl(i), stick.PID.wing(i), stick.ID.winl(i), stick.ID.winl(i+1),...
                X1(i), -X2(i), X3(i), OFT);
        end
    end
    
end
%------------------------------------------------------------------------------------------------------------------------


%------------------------------------------------------------------------------------------------------------------------
% Vertical tail
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.vert, 1)
    
    % Generate orientation vector for right semi-wing
    
    [X1, X2, X3] = orient_vect_cmps(stick.nodes.vert, stick.ptospanel.vert, geo.vtail.CAERO1.n);
    stick.CBAR.vert.v = [X1; X2; X3];
    
    % get rotation matrix
    [R] = rotation_matix_cbar(stick.nodes.vert, stick.CBAR.vert.v);
    stick.CBAR.vert.R = R;
    
    % Set default parameters
    OFT = 'GGG';
    for i = 1:length(stick.EID.vert)
        BULKdataCBAR(fid, stick.EID.vert(i), stick.PID.vert(i), stick.ID.vert(i), stick.ID.vert(i+1),...
            X1(i), X2(i), X3(i), OFT);
    end
    
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        for i = 1:length(stick.EID.vert2)
            BULKdataCBAR(fid, stick.EID.vert2(i), stick.PID.vert(i), stick.ID.vert2(i), stick.ID.vert2(i+1),...
                X1(i), -X2(i), X3(i), OFT);
        end
    end
    
end
%------------------------------------------------------------------------------------------------------------------------



%------------------------------------------------------------------------------------------------------------------------
% Horizontal tail
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.horr, 1)
    
    % Generate orientation vector for right semi-wing
    if isequal(stick.model.vert2, 1)
        [X1, X2, X3] = orient_vect_cmps(stick.nodes.horrC2, geo.htail.WING, geo.htail.CAERO1.n);
        stick.CBAR.horr.v = [X1; X2; X3];
%         stick.CBAR.horr.v(:,1)=[0; -1;0];
        
        % get rotation matrix
        [R] = rotation_matix_cbar(stick.nodes.horrC2, stick.CBAR.horr.v);
        stick.CBAR.horr.R = R;
        
        
        
        % Set default parameters
        OFT = 'GGG';
        PA  = 0;
        PB  = 0;
        for i = 1:length(stick.EID.horr)
%             if i==1
%                 BULKdataCBAR(fid, stick.EID.horr(i), stick.PID.hori(i), stick.ID.horr(i), stick.ID.horr(i+1),...
%                     0, -1, 0, OFT, PA, PB,...
%                     stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
%                     stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
%                 
%                 
%             else
                BULKdataCBAR(fid, stick.EID.horr(i), stick.PID.hori(i), stick.ID.horr(i), stick.ID.horr(i+1),...
                    X1(i), X2(i), X3(i), OFT, PA, PB,...
                    stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
                    stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
%             end
        end
        
        if isequal(stick.model.horl, 1)
            
            % Generate orientation vector for left semi-wing
            %       WING = geo.htail.WING;
            %       WING(2,:) = -WING(2,:);
            %       [X1, X2, X3] = orient_vect_cmps(stick.nodes.horlC2, WING, geo.htail.CAERO1.n);
            %       stick.CBAR.horl.v = [X1; X2; X3];
            
            % get rotation matrix
            %        [R] = rotation_matix_cbar(stick.nodes.horlC2, stick.CBAR.horl.v);
            stick.CBAR.horl.R = R;
            
            for i = 1:length(stick.EID.horl)
%                 if i==1
%                     BULKdataCBAR(fid, stick.EID.horl(i), stick.PID.hori(i), stick.ID.horl(i), stick.ID.horl(i+1),...
%                         0, -1, 0, OFT, PA, PB,...
%                         stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
%                         stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
%                     
%                 else
                    BULKdataCBAR(fid, stick.EID.horl(i), stick.PID.hori(i), stick.ID.horl(i), stick.ID.horl(i+1),...
                        X1(i), -X2(i), X3(i), OFT, PA, PB,...
                        stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
                        stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
%                 end
            end
        end
        
        
        
        
        
        
    else
        [X1, X2, X3] = orient_vect_cmps(stick.nodes.horrC2, geo.htail.WING, geo.htail.CAERO1.n);
        stick.CBAR.horr.v = [X1; X2; X3];
        
        % get rotation matrix
        [R] = rotation_matix_cbar(stick.nodes.horrC2, stick.CBAR.horr.v);
        stick.CBAR.horr.R = R;
        
        
        
        % Set default parameters
        OFT = 'GGG';
        PA  = 0;
        PB  = 0;
        for i = 1:length(stick.EID.horr)
            BULKdataCBAR(fid, stick.EID.horr(i), stick.PID.hori(i), stick.ID.horr(i), stick.ID.horr(i+1),...
                X1(i), X2(i), X3(i), OFT, PA, PB,...
                stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
                stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
        end
        
        if isequal(stick.model.horl, 1)
            
            % Generate orientation vector for left semi-wing
            %       WING = geo.htail.WING;
            %       WING(2,:) = -WING(2,:);
            %       [X1, X2, X3] = orient_vect_cmps(stick.nodes.horlC2, WING, geo.htail.CAERO1.n);
            %       stick.CBAR.horl.v = [X1; X2; X3];
            
            % get rotation matrix
            %        [R] = rotation_matix_cbar(stick.nodes.horlC2, stick.CBAR.horl.v);
            stick.CBAR.horl.R = R;
            
            for i = 1:length(stick.EID.horl)
                BULKdataCBAR(fid, stick.EID.horl(i), stick.PID.hori(i), stick.ID.horl(i), stick.ID.horl(i+1),...
                    X1(i), -X2(i), X3(i), OFT, PA, PB,...
                    stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3),...
                    stick.OFFSET.hori(1), stick.OFFSET.hori(2), stick.OFFSET.hori(3));
            end
        end
        
        
        
    end
end
%------------------------------------------------------------------------------------------------------------------------



%------------------------------------------------------------------------------------------------------------------------
% Canard
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.canr, 1)
    
    % Generate orientation vector for right semi-wing
    [X1, X2, X3] = orient_vect_cmps(stick.nodes.canrC2, geo.canard.WING, geo.canard.CAERO1.n);
    stick.CBAR.canr.v = [X1; X2; X3];
    
    
%     stick.CBAR.canr.v(:,1)=[0; -1;0];
    
    % get rotation matrix
    [R] = rotation_matix_cbar(stick.nodes.canrC2, stick.CBAR.canr.v);
    stick.CBAR.canr.R = R;
    
    % Set default parameters
    OFT = 'GGG';
    for i = 1:length(stick.EID.canr)
%         if i==1
%             BULKdataCBAR(fid, stick.EID.canr(i), stick.PID.canr(i), stick.ID.canr(i), stick.ID.canr(i+1),...
%                 0, -1, 0, OFT);
%         else
            BULKdataCBAR(fid, stick.EID.canr(i), stick.PID.canr(i), stick.ID.canr(i), stick.ID.canr(i+1),...
                X1(i), X2(i), X3(i), OFT);
%         end
        
        
    end
    
    if isequal(stick.model.canl, 1)
        
        % Generate orientation vector for left semi-wing
        %        WING = geo.wing.WING;
        %        WING(2,:) = -WING(2,:);
        %        [X1, X2, X3] = orient_vect_cmps(stick.nodes.winlC2, WING, geo.wing.CAERO1.n);
        %        stick.CBAR.horl.v = [X1; X2; X3];
        
        % get rotation matrix
        %        [R] = rotation_matix_cbar(stick.nodes.winlC2, stick.CBAR.horl.v);
        stick.CBAR.canl.R = R;
        
        for i = 1:length(stick.EID.canl)
%             if i==1
%                 BULKdataCBAR(fid, stick.EID.canl(i-1), stick.PID.canr(i), stick.ID.canl(i), stick.ID.canl(i+1),...
%                     0, -1, 0, OFT);
%             else

                BULKdataCBAR(fid, stick.EID.canl(i), stick.PID.canr(i), stick.ID.canl(i), stick.ID.canl(i+1),...
                    X1(i), -X2(i), X3(i), OFT);
%             end
            
        end
    end
    
end
%--------------------------------------------------------------------------
if isequal(stick.model.tboomsr, 1)
    
    % Generate orientation vector for fuselage
    [X1, X2, X3] = VectOrien(stick.nodes.tboomsr);
    stick.CBAR.tboomsr.v = [X1; X2; X3];
    
    % get rotation matrix
    [R] = rotation_matix_cbar(stick.nodes.tboomsr, stick.CBAR.tboomsr.v);
    stick.CBAR.tboomsr.R = R;
    
    % Set default parameters
    OFT = 'GGG';
    PA = 0;
    PB = 0;
    for i = 1:length(stick.EID.tboomsr)
        BULKdataCBAR(fid, stick.EID.tboomsr(i), stick.PID.tbooms(i), stick.ID.tboomsr(i), stick.ID.tboomsr(i+1),...
            X1(i), X2(i), X3(i), OFT, PA, PB,...
            0, 0, 0,...
            0, 0, 0);
    end
    if isequal(stick.model.tboomsl, 1)
            
            % Generate orientation vector for left semi-wing
            %       WING = geo.htail.WING;
            %       WING(2,:) = -WING(2,:);
            %       [X1, X2, X3] = orient_vect_cmps(stick.nodes.horlC2, WING, geo.htail.CAERO1.n);
            %       stick.CBAR.horl.v = [X1; X2; X3];
            
            % get rotation matrix
            %        [R] = rotation_matix_cbar(stick.nodes.horlC2, stick.CBAR.horl.v);
            stick.CBAR.tboomsl.R = R;
            
            for i = 1:length(stick.EID.tboomsl)
                BULKdataCBAR(fid, stick.EID.tboomsl(i), stick.PID.tbooms(i), stick.ID.tboomsl(i), stick.ID.tboomsl(i+1),...
                    X1(i), -X2(i), X3(i), OFT, PA, PB,...
                    0, 0, 0,...
                    0, 0, 0);
            end
    end
end
%------------------------------------------------------------------------------------------------------------------------





fprintf(outf, 'done.');
