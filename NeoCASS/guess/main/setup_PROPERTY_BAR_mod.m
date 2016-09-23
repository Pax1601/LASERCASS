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
%----------------------------------------------------------------------------------------------------------------------
% Selecting PBAR / PBARSMX card
%
% Called by:    guess.m
%
% Calls:        writePBAR2file.m, writePBARSMX2file.m
%
% CREATED 2008-10-14
%   <andreadr@kth.se>
%----------------------------------------------------------------------------------------------------------------------
function [geo, str, stick] = setup_PROPERTY_BAR_mod(outf, fid, geo, str, stick, aircraft, pdcylin, NITE, WRITE)
% Modified by Travaglini 11/19/2009 replacing wing2 with canard
%-------------------------------------------------------------------------------------------------------------
% fuselage
 if isequal(stick.model.fuse, 1)
     
     % interpolate vertical/horizontal thickness over the nodes for stress recovery
     dz = interp1(geo.fus.thick_dom, geo.fus.thick_ver, stick.PBAR.fuse.str_rec_coef.Coord_thick(1,:), 'linear');
     dy = interp1(geo.fus.thick_dom, geo.fus.thick_hor, stick.PBAR.fuse.str_rec_coef.Coord_thick(1,:), 'linear');
     
     % get row vector
     [dz]  = go2col(dz);
     [dy]  = go2col(dy);
     dz = dz';
     dy = dy';
     
     % the same notation in NASTRAN manual is used
     stick.PBAR.fuse.str_rec_coef.C = zeros(2, 2*(length(geo.fus.x_nodes_thick)-1)); % (Z;Y)
     stick.PBAR.fuse.str_rec_coef.D = zeros(2, 2*(length(geo.fus.x_nodes_thick)-1)); % (Z;Y)
     stick.PBAR.fuse.str_rec_coef.C(1,:) = dz;
     stick.PBAR.fuse.str_rec_coef.D(2,:) = dy;
     
     for i = 1:length(stick.PID.fuse)
         
         % give the average since constant properties are assumed over the
         % beam length; construct the cross changing sign
         stick.PBAR.fuse.str_rec_coef.C1(i,1) = (stick.PBAR.fuse.str_rec_coef.C(1,i*2-1)+stick.PBAR.fuse.str_rec_coef.C(1,i*2))/2;
         stick.PBAR.fuse.str_rec_coef.C2(i,1) = (stick.PBAR.fuse.str_rec_coef.C(2,i*2-1)+stick.PBAR.fuse.str_rec_coef.C(2,i*2))/2;
         stick.PBAR.fuse.str_rec_coef.D1(i,1) = (stick.PBAR.fuse.str_rec_coef.D(1,i*2-1)+stick.PBAR.fuse.str_rec_coef.D(1,i*2))/2;
         stick.PBAR.fuse.str_rec_coef.D2(i,1) = (stick.PBAR.fuse.str_rec_coef.D(2,i*2-1)+stick.PBAR.fuse.str_rec_coef.D(2,i*2))/2;
         stick.PBAR.fuse.str_rec_coef.E1(i,1) = - stick.PBAR.fuse.str_rec_coef.C1(i,1);
         stick.PBAR.fuse.str_rec_coef.E2(i,1) = - stick.PBAR.fuse.str_rec_coef.C2(i,1);
         stick.PBAR.fuse.str_rec_coef.F1(i,1) = - stick.PBAR.fuse.str_rec_coef.D1(i,1);
         stick.PBAR.fuse.str_rec_coef.F2(i,1) = - stick.PBAR.fuse.str_rec_coef.D2(i,1);
         
     end
     
 end
%-------------------------------------------------------------------------------------------------------------

%-------------------------------------------------------------------------------------------------------------
% wing
if isequal(stick.model.winr, 1)
    
    % interpolate chord and thickness over the beam mid-point to have averaged values
    deltaX = spline(geo.wing.y, geo.wing.Zs,  geo.wing.y_nodes_1_2);
    deltaZ = spline(geo.wing.y, geo.wing.tbs, geo.wing.y_nodes_1_2);
    deltaX = deltaX ./ 2;
    deltaZ = deltaZ ./ 2;
    
    % get row vector
    [deltaX]  = go2col(deltaX);
    [deltaZ]  = go2col(deltaZ);
    deltaX = deltaX';
    deltaZ = deltaZ';
    
    % define stress recovery factor
    stick.PBAR.wing.str_rec_coef.C = zeros(2, length(stick.PID.wing)); % (Z;Y)
    stick.PBAR.wing.str_rec_coef.D = zeros(2, length(stick.PID.wing)); % (Z;Y)
    stick.PBAR.wing.str_rec_coef.C(2,:) = deltaX;
    stick.PBAR.wing.str_rec_coef.D(1,:) = deltaZ;
    
end
%-------------------------------------------------------------------------------------------------------------

%-------------------------------------------------------------------------------------------------------------
% vertical tail
if isequal(stick.model.vert, 1)
    
    geo.vtail.ystick = genstick( stick.nodes.vert );
    
    % interpolate chord and thickness over the beam mid-point to have averaged values
    deltaX = spline(geo.vtail.y, geo.vtail.Zs,   geo.vtail.ystick);
    deltaZ = spline(geo.vtail.y, geo.vtail.tbs, geo.vtail.ystick);
    deltaX = deltaX ./ 2;
    deltaZ = deltaZ ./ 2;
    
    % get row vector
    [deltaX]  = go2col(deltaX);
    [deltaZ]  = go2col(deltaZ);
    deltaX = deltaX';
    deltaZ = deltaZ';
    
%     % consider the beam elements not in the exposed surface
%     for i = 1 : stick.vtail.strind - 1
%         deltaX = [deltaX(1), deltaX];
%         deltaZ = [deltaZ(1), deltaZ];
%     end
    
    % define stress recovery factor
    stick.PBAR.vert.str_rec_coef.C = zeros(2, length(stick.PID.vert)); % (Z;Y)
    stick.PBAR.vert.str_rec_coef.D = zeros(2, length(stick.PID.vert)); % (Z;Y)
    stick.PBAR.vert.str_rec_coef.C(2,:) = deltaX;
    stick.PBAR.vert.str_rec_coef.D(1,:) = deltaZ;
    
end
%-------------------------------------------------------------------------------------------------------------

%-------------------------------------------------------------------------------------------------------------
% horizontal tail

if isequal(stick.model.horr, 1)
    
    % interpolate chord and thickness over the beam mid-point to have averaged values
    deltaX = spline(geo.htail.y, geo.htail.Zs,   geo.htail.y_nodes_1_2);
    deltaZ = spline(geo.htail.y, geo.htail.tbs, geo.htail.y_nodes_1_2);
    deltaX = deltaX ./ 2;
    deltaZ = deltaZ ./ 2;
    
    % get row vector
    [deltaX]  = go2col(deltaX);
    [deltaZ]  = go2col(deltaZ);
    deltaX = deltaX';
    deltaZ = deltaZ';
    
    % define stress recovery factor
    stick.PBAR.hori.str_rec_coef.C = zeros(2, length(stick.PID.hori)); % (Z;Y)
    stick.PBAR.hori.str_rec_coef.D = zeros(2, length(stick.PID.hori)); % (Z;Y)
    stick.PBAR.hori.str_rec_coef.C(2,:) = deltaX;
    stick.PBAR.hori.str_rec_coef.D(1,:) = deltaZ;
    
end

%-------------------------------------------------------------------------------------------------------------


%-------------------------------------------------------------------------------------------------------------
% canard
if isequal(stick.model.canr, 1)
    
    % interpolate chord and thickness over the beam mid-point to have averaged values
    deltaX = spline(geo.canard.y, geo.canard.Zs,  geo.canard.y_nodes_1_2);
    deltaZ = spline(geo.canard.y, geo.canard.tbs, geo.canard.y_nodes_1_2);
    deltaX = deltaX ./ 2;
    deltaZ = deltaZ ./ 2;
    
    % get row vector
    [deltaX]  = go2col(deltaX);
    [deltaZ]  = go2col(deltaZ);
    deltaX = deltaX';
    deltaZ = deltaZ';
    
    % define stress recovery factor
    stick.PBAR.canr.str_rec_coef.C = zeros(2, length(stick.PID.canr)); % (Z;Y)
    stick.PBAR.canr.str_rec_coef.D = zeros(2, length(stick.PID.canr)); % (Z;Y)
    stick.PBAR.canr.str_rec_coef.C(2,:) = deltaX;
    stick.PBAR.canr.str_rec_coef.D(1,:) = deltaZ;
    
end
%-------------------------------------------------------------------------------------------------------------
% PBAR / PBARSMX cards
% PBAR card activated
if (WRITE==1)
  if (NITE>1)
    [geo, str, stick] = writePBAR2file(outf, fid, pdcylin, geo, str, stick, aircraft);
  else
    [geo, str, stick] = writePBAR2file_mod(outf, fid, geo, str, stick, aircraft,pdcylin);
  end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end % end of setup_PROPERTY_BAR.m, DO NOT REMOVE

%--------------------------------------------------------------------------------------------------------------------------------
function [Ps] = genstick(P)

[row, col] = size(P);
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

end % end of writePBAR2file.m, DO NOT REMOVE
%--------------------------------------------------------------------------------------------------------------------------------
