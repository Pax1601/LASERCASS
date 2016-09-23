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
%
% Called by:    MainCode.m
%
% Calls:        VectOrien.m, BULKdataCBAR.m
%
%------------------------------------------------------------------------------------------------------------------------
function [stick] = setCBAR_mod(outf, stick, geo, aircraft)
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
    
    if isequal(stick.model.winl, 1)
        stick.CBAR.horl.R = R;
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
end
%------------------------------------------------------------------------------------------------------------------------



%------------------------------------------------------------------------------------------------------------------------
% Horizontal tail
%------------------------------------------------------------------------------------------------------------------------

if isequal(stick.model.horr, 1)
    
    % Generate orientation vector for right semi-wing
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        [X1, X2, X3] = orient_vect_cmps(stick.nodes.horrC2, geo.htail.WING, geo.htail.CAERO1.n);
        stick.CBAR.horr.v = [X1; X2; X3];
%         stick.CBAR.horr.v(:,1)=[0; -1;0];
        
        % get rotation matrix
        [R] = rotation_matix_cbar(stick.nodes.horrC2, stick.CBAR.horr.v);
        stick.CBAR.horr.R = R;
        
        if isequal(stick.model.horl, 1)
            stick.CBAR.horl.R = R;
        end
    else
        [X1, X2, X3] = orient_vect_cmps(stick.nodes.horrC2, geo.htail.WING, geo.htail.CAERO1.n);
        stick.CBAR.horr.v = [X1; X2; X3];
        
        % get rotation matrix
        [R] = rotation_matix_cbar(stick.nodes.horrC2, stick.CBAR.horr.v);
        stick.CBAR.horr.R = R;
        if isequal(stick.model.horl, 1)
            stick.CBAR.horl.R = R;
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
    [R] = rotation_matix_cbar(stick.nodes.canrC2, stick.CBAR.canr.v);
    stick.CBAR.canr.R = R;
    if isequal(stick.model.canl, 1)
        stick.CBAR.canl.R = R;
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
    if isequal(stick.model.tboomsl, 1)
            stick.CBAR.tboomsl.R = R;
    end
end
%------------------------------------------------------------------------------------------------------------------------
fprintf(outf, 'done.');
