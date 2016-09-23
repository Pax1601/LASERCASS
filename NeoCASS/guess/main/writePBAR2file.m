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
% Export PBAR card
%
%   Author: <andreadr@kth.se>
%
% Called by:    setup_PROPERTY_BAR.m
%
% Calls:        BULKdataPBAR.m
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080724      1.0      A. Da Ronch      Creation
%     091119      1.3.9    Travaglini       Modification
%     120502      2.1.237  Riccobene        Modification
%
% Modified by Travaglini changing wing2 with canard
%*******************************************************************************
function [geo, str, stick] = writePBAR2file(outf, fid, pdcylin, geo, str, stick, aircraft)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Beam properties');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
%
fprintf(outf, '\n\tExporting beam structural properties...');
%
%--------------------------------------------------------------------------------------------------------------------------
% Fuselage
if isequal(stick.model.fuse, 1)
      
  % Beam element mass
  str.fus.Mstick = interp1(geo.fus.x_nodes_1_2_thick, str.fus.m, geo.fus.x_nodes_1_2);
  
  % Beam element area
  str.fus.Astick = interp1(geo.fus.x_nodes_1_2_thick,...
        str.fus.m./(pdcylin.fus.ds*diff(geo.fus.x_nodes_thick)), geo.fus.x_nodes_1_2, 'spline');
  
  % Beam element inertias
  str.fus.I1stick = interp1(geo.fus.x, str.fus.I1, geo.fus.x_nodes_1_2,'linear', 'extrap');
  str.fus.I2stick = interp1(geo.fus.x, str.fus.I2, geo.fus.x_nodes_1_2,'linear', 'extrap');
  str.fus.Jstick  = interp1(geo.fus.x, str.fus.J , geo.fus.x_nodes_1_2,'linear', 'extrap');
  str.fus.K1stick = interp1(geo.fus.x, str.fus.K1, geo.fus.x_nodes_1_2,'linear', 'extrap');
  str.fus.K2stick = interp1(geo.fus.x, str.fus.K2, geo.fus.x_nodes_1_2,'linear', 'extrap');
  
  % Column vectors
  str.fus.Astick  = go2col(str.fus.Astick);
  str.fus.I1stick = go2col(str.fus.I1stick);
  str.fus.I2stick = go2col(str.fus.I2stick);
  str.fus.Jstick  = go2col(str.fus.Jstick);
  str.fus.K1stick = go2col(str.fus.K1stick);
  str.fus.K2stick = go2col(str.fus.K2stick);
  
  for i = 1:length(stick.PID.fuse)
    % PBAR card
    BULKdataPBAR(fid, stick.PID.fuse(i), stick.MAT1.fuse, str.fus.Astick(i), str.fus.I1stick(i), str.fus.I2stick(i),...
        str.fus.Jstick(i), str.fus.NSM.dstr(i),...
        stick.PBAR.fuse.str_rec_coef.C1(i), stick.PBAR.fuse.str_rec_coef.C2(i),...
        stick.PBAR.fuse.str_rec_coef.D1(i), stick.PBAR.fuse.str_rec_coef.D2(i),...
        stick.PBAR.fuse.str_rec_coef.E1(i), stick.PBAR.fuse.str_rec_coef.E2(i),...
        stick.PBAR.fuse.str_rec_coef.F1(i), stick.PBAR.fuse.str_rec_coef.F2(i),...
        str.fus.K1stick(i), str.fus.K2stick(i));
  end
  
end
%--------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------
% Tail booms
if isequal(stick.model.tboomsr, 1)
    % Beam element mass
    str.tbooms.Mstick = interp1(geo.tbooms.x_nodes_1_2_thick, str.tbooms.m, geo.tbooms.x_nodes_1_2);
    
    % Beam element area
    str.tbooms.Astick = interp1(geo.tbooms.x_nodes_1_2_thick,...
        str.tbooms.m./(pdcylin.tbooms.ds*diff(geo.tbooms.x_nodes_thick)), geo.tbooms.x_nodes_1_2, 'spline');
    
    str.tbooms.I1stick = interp1(geo.tbooms.x, str.tbooms.I1, geo.tbooms.x_nodes_1_2, 'linear', 'extrap');
    str.tbooms.I2stick = interp1(geo.tbooms.x, str.tbooms.I2, geo.tbooms.x_nodes_1_2, 'linear', 'extrap');
    str.tbooms.Jstick  = interp1(geo.tbooms.x, str.tbooms.J , geo.tbooms.x_nodes_1_2, 'linear', 'extrap');
    str.tbooms.K1stick = interp1(geo.tbooms.x, str.tbooms.K1, geo.tbooms.x_nodes_1_2, 'linear', 'extrap');
    str.tbooms.K2stick = interp1(geo.tbooms.x, str.tbooms.K2, geo.tbooms.x_nodes_1_2, 'linear', 'extrap');
    
    % Column vectors
    str.tbooms.Astick  = go2col(str.tbooms.Astick);
    str.tbooms.I1stick = go2col(str.tbooms.I1stick);
    str.tbooms.I2stick = go2col(str.tbooms.I2stick);
    str.tbooms.Jstick  = go2col(str.tbooms.Jstick);
    str.tbooms.K1stick = go2col(str.tbooms.K1stick);
    str.tbooms.K2stick = go2col(str.tbooms.K2stick);
    
    for i = 1:length(stick.PID.tbooms)
        % PBAR card
        BULKdataPBAR(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, str.tbooms.Astick(i), str.tbooms.I1stick(i), str.tbooms.I2stick(i),...
            str.tbooms.Jstick(i), str.tbooms.NSM.dstr(i),...
            geo.tbooms.R, 0,...
            0, geo.tbooms.R,...
            -geo.tbooms.R, 0,...
            0, -geo.tbooms.R,...
            str.tbooms.K1stick(i), str.tbooms.K2stick(i));
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------
% Wing
if isequal(stick.model.winr, 1)
    
    % Beam element mass
    str.wing.Mstick = interp1(geo.wing.y_nodes_1_2_thick, str.wing.m, geo.wing.y_nodes_1_2);

    % Beam element area
    % Separate carry-through and cantilever part
    nrcth   = pdcylin.stick.nwing_carryth;
    dy_carr = norm(stick.ptos.winrC2(:, 2)-stick.ptos.winrC2(:, 1))/nrcth;    
    str.wing.Astick              = zeros(numel(geo.wing.y_nodes_1_2), 1);
    str.wing.Astick(1:nrcth)     = str.wing.m(1:nrcth)./(pdcylin.wing.dsw*dy_carr);
    str.wing.Astick(nrcth+1:end) = interp1(geo.wing.y_nodes_1_2_thick(nrcth+1:end),...
        str.wing.mbox./(pdcylin.wing.dsw*diff(geo.wing.y_nodes_thick(nrcth+1:end))), geo.wing.y_nodes_1_2(nrcth+1:end), 'linear', 'extrap');

    % Inertias
    str.wing.I1stick = interp1(geo.wing.y, str.wing.I1, geo.wing.y_nodes_1_2,'linear', 'extrap');
    str.wing.I2stick = interp1(geo.wing.y, str.wing.I2, geo.wing.y_nodes_1_2,'linear', 'extrap');
    str.wing.Jstick  = interp1(geo.wing.y, str.wing.J , geo.wing.y_nodes_1_2,'linear', 'extrap');
    str.wing.K1stick = interp1(geo.wing.y, str.wing.K1, geo.wing.y_nodes_1_2,'linear', 'extrap');
    str.wing.K2stick = interp1(geo.wing.y, str.wing.K2, geo.wing.y_nodes_1_2,'linear', 'extrap');
    
    % Get column vector
    str.wing.Astick  = go2col(str.wing.Astick);
    str.wing.I1stick = go2col(str.wing.I1stick);
    str.wing.I2stick = go2col(str.wing.I2stick);
    str.wing.Jstick  = go2col(str.wing.Jstick);
    str.wing.K1stick = go2col(str.wing.K1stick);
    str.wing.K2stick = go2col(str.wing.K2stick);
    
    % Call PBAR card
    for i = 1:length(stick.PID.wing)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.wing.str_rec_coef.C(1,i);
        C2 = stick.PBAR.wing.str_rec_coef.C(2,i);
        D1 = stick.PBAR.wing.str_rec_coef.D(1,i);
        D2 = stick.PBAR.wing.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        BULKdataPBAR(fid, stick.PID.wing(i), stick.MAT1.wing, str.wing.Astick(i), str.wing.I1stick(i), str.wing.I2stick(i), str.wing.Jstick(i), str.wing.NSM.dstr(i),...
            C1, C2, D1, D2, E1, E2, F1, F2,...
            str.wing.K1stick(i), str.wing.K2stick(i));
        
    end
    
end
%--------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------
% Vertical tail
if isequal(stick.model.vert, 1)
 
    % Beam element mass
    str.vtail.Mstick = interp1(geo.vtail.y_nodes_1_2_thick, str.vtail.mbox, geo.vtail.y_nodes_1_2);
    
    % Beam element area
    str.vtail.Astick = interp1(geo.vtail.y_nodes_1_2_thick,...
        str.vtail.mbox./(pdcylin.vtail.dsw*diff(geo.vtail.y_nodes_thick)), geo.vtail.y_nodes_1_2, 'linear', 'extrap');

    % Inertias
    str.vtail.I1stick = interp1(geo.vtail.y_nodes_thick, str.vtail.I1, geo.vtail.y_nodes_1_2, 'linear', 'extrap');
    str.vtail.I2stick = interp1(geo.vtail.y_nodes_thick, str.vtail.I2, geo.vtail.y_nodes_1_2, 'linear', 'extrap');
    str.vtail.Jstick  = interp1(geo.vtail.y_nodes_thick, str.vtail.J , geo.vtail.y_nodes_1_2, 'linear', 'extrap');
    str.vtail.K1stick = interp1(geo.vtail.y_nodes_thick, str.vtail.K1, geo.vtail.y_nodes_1_2, 'linear', 'extrap');
    str.vtail.K2stick = interp1(geo.vtail.y_nodes_thick, str.vtail.K2, geo.vtail.y_nodes_1_2, 'linear', 'extrap');
    
    % Get column vector
    str.vtail.Astick  = go2col(str.vtail.Astick);
    str.vtail.I1stick = go2col(str.vtail.I1stick);
    str.vtail.I2stick = go2col(str.vtail.I2stick);
    str.vtail.Jstick  = go2col(str.vtail.Jstick);
    str.vtail.K1stick = go2col(str.vtail.K1stick);
    str.vtail.K2stick = go2col(str.vtail.K2stick);
     
    % Call PBAR card
    for i = 1:length(stick.PID.vert)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.vert.str_rec_coef.C(1,i);
        C2 = stick.PBAR.vert.str_rec_coef.C(2,i);
        D1 = stick.PBAR.vert.str_rec_coef.D(1,i);
        D2 = stick.PBAR.vert.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        BULKdataPBAR(fid, stick.PID.vert(i), stick.MAT1.vert, str.vtail.Astick(i), str.vtail.I1stick(i),...
            str.vtail.I2stick(i), str.vtail.Jstick(i), str.vtail.NSM.dstr(i),...
            C1, C2, D1, D2, E1, E2, F1, F2,...
            str.vtail.K1stick(i), str.vtail.K2stick(i));
        
    end
    
%     if isequal(aircraft.Vertical_tail.Twin_tail, 1)
%         for i = 1:length(stick.PID.vert)
%             
%             % give the average since constant properties are assumed over the beam length; construct the cross changing sign
%             C1 = stick.PBAR.vert.str_rec_coef.C(1,i);
%             C2 = stick.PBAR.vert.str_rec_coef.C(2,i);
%             D1 = stick.PBAR.vert.str_rec_coef.D(1,i);
%             D2 = stick.PBAR.vert.str_rec_coef.D(2,i);
%             E1 = -C1;
%             E2 = -C2;
%             F1 = -D1;
%             F2 = -D2;
%             BULKdataPBAR(fid, stick.PID.vert2(i), stick.MAT1.vert, str.vtail.Astick(i), str.vtail.I1stick(i),...
%                 str.vtail.I2stick(i), str.vtail.Jstick(i), str.vtail.NSM.dstr(i),...
%                 C1, C2, D1, D2, E1, E2, F1, F2,...
%                 str.vtail.K1stick(i), str.vtail.K2stick(i));
%             
%         end
%         
%     end
    
end
%--------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------
% Horizontal tail

if isequal(stick.model.horr, 1)
    
    % Check carry-through existence
    if geo.htail.twc > 0.0
        nrcth   = pdcylin.stick.nhtail_carryth;
        dy_carr = norm(stick.ptos.horrC2(:, 2)-stick.ptos.horrC2(:, 1))/nrcth;
        
        % Compute beam element area
        str.htail.Astick              = zeros(numel(geo.htail.y_nodes_1_2), 1);
        str.htail.Astick(1:nrcth)     = str.htail.m(1:nrcth)./(pdcylin.htail.dsw*dy_carr);
        str.htail.Astick(nrcth+1:end) = interp1(geo.htail.y_nodes_1_2_thick(nrcth+1:end),...
            str.htail.mbox./(pdcylin.htail.dsw*diff(geo.htail.y_nodes_thick(nrcth+1:end))), geo.htail.y_nodes_1_2(nrcth+1:end), 'linear', 'extrap');
    else
        % Otherwise compute only cantilever part        
        str.htail.Astick =  interp1(geo.htail.y_nodes_1_2_thick,...
            str.htail.mbox./(pdcylin.htail.dsw*diff(geo.htail.y_nodes_thick)), geo.htail.y_nodes_1_2, 'linear', 'extrap');
    end
    
    % Inertias
    str.htail.I1stick = interp1(geo.htail.y_nodes_thick, str.htail.I1, geo.htail.y_nodes_1_2,'linear', 'extrap');
    str.htail.I2stick = interp1(geo.htail.y_nodes_thick, str.htail.I2, geo.htail.y_nodes_1_2,'linear', 'extrap');
    str.htail.Jstick  = interp1(geo.htail.y_nodes_thick, str.htail.J , geo.htail.y_nodes_1_2,'linear', 'extrap');
    str.htail.K1stick = interp1(geo.htail.y_nodes_thick, str.htail.K1, geo.htail.y_nodes_1_2,'linear', 'extrap');
    str.htail.K2stick = interp1(geo.htail.y_nodes_thick, str.htail.K2, geo.htail.y_nodes_1_2,'linear', 'extrap');
    
    % Get column vector
    str.htail.Astick  = go2col(str.htail.Astick);
    str.htail.I1stick = go2col(str.htail.I1stick);
    str.htail.I2stick = go2col(str.htail.I2stick);
    str.htail.Jstick  = go2col(str.htail.Jstick);
    str.htail.K1stick = go2col(str.htail.K1stick);
    str.htail.K2stick = go2col(str.htail.K2stick);
    
    % Call PBAR card
    for i = 1:length(stick.PID.hori)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.hori.str_rec_coef.C(1,i);
        C2 = stick.PBAR.hori.str_rec_coef.C(2,i);
        D1 = stick.PBAR.hori.str_rec_coef.D(1,i);
        D2 = stick.PBAR.hori.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        BULKdataPBAR(fid, stick.PID.hori(i), stick.MAT1.hori, str.htail.Astick(i), str.htail.I1stick(i), str.htail.I2stick(i), str.htail.Jstick(i), str.htail.NSM.dstr(i),...
            C1, C2, D1, D2, E1, E2, F1, F2,...
            str.htail.K1stick(i), str.htail.K2stick(i));
        
    end
    
end

%--------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------
% Canard
if isequal(stick.model.canr, 1)
    
    % Beam element mass
    str.canard.Mstick = interp1(geo.canard.y_nodes_1_2_thick, str.canard.m, geo.canard.y_nodes_1_2);
    
    % Check carry-through existence
    if geo.canard.twc > 0.0
        nrcth   = pdcylin.stick.ncanard_carryth;
        dy_carr = norm(stick.ptos.canrC2(:, 2)-stick.ptos.canrC2(:, 1))/nrcth;
        
        % Compute beam element area
        str.canard.Astick              = zeros(numel(geo.canard.y_nodes_1_2), 1);
        str.canard.Astick(1:nrcth)     = str.canard.m(1:nrcth)./(pdcylin.canard.dsw*dy_carr);
        str.canard.Astick(nrcth+1:end) = interp1(geo.canard.y_nodes_1_2_thick(nrcth+1:end),...
            str.canard.mbox./(pdcylin.canard.dsw*diff(geo.canard.y_nodes_thick(nrcth+1:end))), geo.canard.y_nodes_1_2(nrcth+1:end), 'linear', 'extrap');
    else
        % Otherwise compute only cantilever part        
        str.canard.Astick =  interp1(geo.canard.y_nodes_1_2_thick,...
            str.canard.mbox./(pdcylin.canard.dsw*diff(geo.canard.y_nodes_thick)), geo.canard.y_nodes_1_2, 'linear', 'extrap');
    end

    % Inertias
    str.canard.I1stick = interp1(geo.canard.y, str.canard.I1, geo.canard.y_nodes_1_2,'linear', 'extrap');
    str.canard.I2stick = interp1(geo.canard.y, str.canard.I2, geo.canard.y_nodes_1_2,'linear', 'extrap');
    str.canard.Jstick  = interp1(geo.canard.y, str.canard.J , geo.canard.y_nodes_1_2,'linear', 'extrap');
    str.canard.K1stick = interp1(geo.canard.y, str.canard.K1, geo.canard.y_nodes_1_2,'linear', 'extrap');
    str.canard.K2stick = interp1(geo.canard.y, str.canard.K2, geo.canard.y_nodes_1_2,'linear', 'extrap');
    
    % Get column vector
    str.canard.Astick  = go2col(str.canard.Astick);
    str.canard.I1stick = go2col(str.canard.I1stick);
    str.canard.I2stick = go2col(str.canard.I2stick);
    str.canard.Jstick  = go2col(str.canard.Jstick);
    str.canard.K1stick = go2col(str.canard.K1stick);
    str.canard.K2stick = go2col(str.canard.K2stick);
    
    % Call PBAR card
    for i = 1:length(stick.PID.canr)
        
        % give the average since constant properties are assumed over the beam length; construct the cross changing sign
        C1 = stick.PBAR.canr.str_rec_coef.C(1,i);
        C2 = stick.PBAR.canr.str_rec_coef.C(2,i);
        D1 = stick.PBAR.canr.str_rec_coef.D(1,i);
        D2 = stick.PBAR.canr.str_rec_coef.D(2,i);
        E1 = -C1;
        E2 = -C2;
        F1 = -D1;
        F2 = -D2;
        BULKdataPBAR(fid, stick.PID.canr(i), stick.MAT1.canr, str.canard.Astick(i), str.canard.I1stick(i), str.canard.I2stick(i), str.canard.Jstick(i), str.canard.NSM.dstr(i),...
            C1, C2, D1, D2, E1, E2, F1, F2,...
            str.canard.K1stick(i), str.canard.K2stick(i));
        
        %
    end
    
end % 
%
%--------------------------------------------------------------------------------------------------------------------------
fprintf(outf, 'done.');
end % end of writePBAR2file.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------------------
function Ps = genstick(P)

[~, col] = size(P);
delta = zeros(col-1,1);
Ps= delta;
for i = 1:col-1
    delta(i,1) = norm( P(:,i+1) - P(:,i) )/2;
    if isequal(i, 1)
        Ps(i,1) = delta(i,1);
    else
        Ps(i,1) = Ps(i-1,1) + delta(i-1,1) + delta(i,1);
    end
    
end

end
%--------------------------------------------------------------------------------------------------------------------------------
