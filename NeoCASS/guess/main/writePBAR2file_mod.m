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
%--------------------------------------------------------------------------------------------------------------------------------
% Export PBAR card
%
% Called by:    setup_PROPERTY_BAR.m
%
% Calls:        BULKdataPBAR.m
%
% MODIFIED 2008-07-24
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------------------------------------
function [geo, str, stick] = writePBAR2file_mod(outf, fid, geo, str, stick, aircraft,pdcylin)
% modified by Travaglini 19/11/2009. Now the model is not defined with
% unitary density and null masses, but stiffness and mass are defined
% starting from structural masses computed by Weight and ballance
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n Beam properties');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(outf, '\n\tExporting beam structural properties...');
%--------------------------------------------------------------------------------------------------------------------------
% Fuselage 
if isequal(stick.model.fuse, 1) 
    geo.fus.t = FUSE_t(geo,aircraft.weight_balance.COG(5,4,1)/pdcylin.fus.ds); 
    rr = interp1(geo.fus.x, geo.fus.r, (stick.nodes.fuse(1,1:end-1) + stick.nodes.fuse(1,2:end))*0.5,'linear', 'extrap');
    for i = 1:length(stick.PID.fuse)
        A = 2*pi*rr(i)*geo.fus.t - pi*geo.fus.t^2;
        J1 = pi/4*(rr(i)^4 - (rr(i)-geo.fus.t)^4);
        Jt = 2*J1; 
        BULKdataPBAR(fid, stick.PID.fuse(i), stick.MAT1.fuse, A,J1, J1,...
            Jt, str.fus.NSM.dstr(i),...
            [],[],...
            [],[],...
            [],[],...
            [],[],...
            [],[]);
    end
end
%--------------------------------------------------------------------------------------------------------------------------
% Wing
if isequal(stick.model.winr, 1)
  geo.wing.t = WBOX_t(geo.wing.Zs, geo.wing.tbs,aircraft.weight_balance.COG(1,4,1)/pdcylin.wing.dsw/2,stick.nodes.winrC2');
  coord = interp1(stick.nodes.winrC2(2,:), geo.wing.Zs, (stick.nodes.winrC2(2,1:end-1) + stick.nodes.winrC2(2,2:end))*0.5,'linear', 'extrap');
  tbs  = interp1(stick.nodes.winrC2(2,:),geo.wing.tbs, (stick.nodes.winrC2(2,1:end-1) + stick.nodes.winrC2(2,2:end))*0.5,'linear', 'extrap');
  for i = 1:length(stick.PID.wing)
    A = coord(i)*tbs(i) - (coord(i)-2*geo.wing.t)*(tbs(i)-2*geo.wing.t);
    J1 = 2*coord(i)*geo.wing.t*(tbs(i)*0.5)^2 + 2*geo.wing.t*tbs(i)^3/12;
    J2 = 2*tbs(i)*geo.wing.t*(coord(i)*0.5)^2 + 2*geo.wing.t*coord(i)^3/12;
    J = 4 .*(coord(i)*tbs(i)).^2 /(2.*(coord(i)+tbs(i))./geo.wing.t);
    BULKdataPBAR(fid, stick.PID.wing(i), stick.MAT1.wing, A, J1, J2, J, str.wing.NSM.dstr(i),...
        [], [], [], [], [], [], [], [],...
        [], []);
  end
end
%--------------------------------------------------------------------------------------------------------------------------
% Vertical tail
if isequal(stick.model.vert, 1)
    geo.vtail.t = WBOX_t(geo.vtail.Zs, geo.vtail.tbs,aircraft.weight_balance.COG(4,4,1)/pdcylin.vtail.dsw,stick.nodes.vert');
    coord = interp1(stick.nodes.vert(3,:), geo.vtail.Zs, (stick.nodes.vert(3,1:end-1) + stick.nodes.vert(3,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.vert(3,:),geo.vtail.tbs, (stick.nodes.vert(3,1:end-1) + stick.nodes.vert(3,2:end))*0.5,'linear', 'extrap');
    % Call PBAR card
    for i = 1:length(stick.PID.vert)
        A = coord(i)*tbs(i) - (coord(i)-2*geo.vtail.t)*(tbs(i)-2*geo.vtail.t);
        J1 = 2*coord(i)*geo.vtail.t*(tbs(i)*0.5)^2 + 2*geo.vtail.t*tbs(i)^3/12;
        J2 = 2*tbs(i)*geo.vtail.t*(coord(i)*0.5)^2 + 2*geo.vtail.t*coord(i)^3/12;
        J = 4 .*(coord(i)*tbs(i)).^2 /(2.*(coord(i)+tbs(i))./geo.vtail.t);
        BULKdataPBAR(fid, stick.PID.vert(i), stick.MAT1.vert, A, J1,...
            J2, J, str.vtail.NSM.dstr(i),...
            [], [], [], [], [], [], [], [],...
            [], []);
    end
end
%--------------------------------------------------------------------------------------------------------------------------
% Horizontal tail

if isequal(stick.model.horr, 1)
    geo.htail.t = WBOX_t(geo.htail.Zs, geo.htail.tbs,aircraft.weight_balance.COG(3,4,1)/pdcylin.htail.dsw/2,stick.nodes.horrC2');
    coord = interp1(stick.nodes.horrC2(2,:), geo.htail.Zs, (stick.nodes.horrC2(2,1:end-1) + stick.nodes.horrC2(2,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.horrC2(2,:),geo.htail.tbs,(stick.nodes.horrC2(2,1:end-1) + stick.nodes.horrC2(2,2:end))*0.5,'linear', 'extrap');
    % Call PBAR card
    for i = 1:length(stick.PID.hori)
        A = coord(i)*tbs(i) - (coord(i)-2*geo.htail.t)*(tbs(i)-2*geo.htail.t);
        J1 = 2*coord(i)*geo.htail.t*(tbs(i)*0.5)^2 + 2*geo.htail.t*tbs(i)^3/12;
        J2 = 2*tbs(i)*geo.htail.t*(coord(i)*0.5)^2 + 2*geo.htail.t*coord(i)^3/12;
        J = 4 .*(coord(i)*tbs(i)).^2 /(2.*(coord(i)+tbs(i))./geo.htail.t);
        BULKdataPBAR(fid, stick.PID.hori(i), stick.MAT1.hori, A, J1, J2, J,str.htail.NSM.dstr(i),...
            [], [], [], [], [], [], [], [],...
            [], []);
    end
end
%--------------------------------------------------------------------------------------------------------------------------
% % Canard
if isequal(stick.model.canr, 1)
    geo.canard.t = WBOX_t(geo.canard.Zs, geo.canard.tbs, aircraft.weight_balance.COG(11,4,1)/pdcylin.canard.dsw/2,stick.nodes.canrC2');
    coord = interp1(stick.nodes.canrC2(2,:), geo.canard.Zs, (stick.nodes.canrC2(2,1:end-1) + stick.nodes.canrC2(2,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.canrC2(2,:),geo.canard.tbs, (stick.nodes.canrC2(2,1:end-1) + stick.nodes.canrC2(2,2:end))*0.5,'linear', 'extrap');
    % Call PBAR card
    for i = 1:length(stick.PID.canr)
        A = coord(i)*tbs(i) - (coord(i)-2*geo.canard.t)*(tbs(i)-2*geo.canard.t);
        J1 = 2*coord(i)*geo.canard.t*(tbs(i)*0.5)^2 + 2*geo.canard.t*tbs(i)^3/12;
        J2 = 2*tbs(i)*geo.canard.t*(coord(i)*0.5)^2 + 2*geo.canard.t*coord(i)^3/12;
        J = 4 .*(coord(i)*tbs(i)).^2 /(2.*(coord(i)+tbs(i))./geo.canard.t);
        BULKdataPBAR(fid, stick.PID.canr(i), stick.MAT1.canr, A, J1, J2, J,str.canard.NSM.dstr(i),...
            [], [], [], [], [], [], [], [],...
            [], []);
        %
    end
end % 
%--------------------------------------------------------------------------------------------------------------------------
% Tbooms 
if isequal(stick.model.tboomsr, 1)
%   V = 0.5*aircraft.weight_balance.COG(5,4,1)/pdcylin.tbooms.ds;  % Quale massa ???
%   Per il momento impongo uno spessore pari al 3% del raggio
    geo.tbooms.t = 0.03*geo.tbooms.R;
    for i = 1:length(stick.PID.tbooms)
        A = 2*pi*geo.tbooms.R*geo.tbooms.t - pi*geo.tbooms.t^2;
        J1 = pi/4*(geo.tbooms.R^4 - (geo.tbooms.R-geo.tbooms.t)^4);
        Jt = 2*J1; 
        BULKdataPBAR(fid, stick.PID.tbooms(i), stick.MAT1.tbooms, A,J1, J1,...
            Jt, str.tbooms.NSM.dstr(i),...
            [],[],...
            [],[],...
            [],[],...
            [],[],...
            [],[]);
    end
end
%--------------------------------------------------------------------------------------------------------------------------
fprintf(outf, 'done.');