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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Luca Cavagna
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%*******************************************************************************
% This function determines loads along each component running a rigid 6DOF model

function  [loads, Res] = Load_std(fid, iter, aircraft, pdcylin, geo, str, stick, Res, Man_index)
global beam_model;
%
loads.nv.N = zeros(beam_model.Info.ntrim,1);
loads.nv.V = zeros(beam_model.Info.ntrim,1);
%
loads.nconf = 1;
loads.wing.N = [];
loads.wing.FS = [];
loads.wing.M = [];
loads.wing.Mt = [];
loads.vtail.N = [];
loads.vtail.FS = [];
loads.vtail.M = [];
loads.vtail.Mt = [];
loads.htail.N = [];
loads.htail.FS = [];
loads.htail.M = [];
loads.htail.Mt = [];
loads.canard.N = [];
loads.canard.FS = [];
loads.canard.M = [];
loads.canard.Mt = [];
loads.fus.Nxt = [];
loads.fus.Nxc = [];
loads.fus.Ny = [];
loads.fus.FS = [];
loads.fus.M = [];
loads.fus.Mt = [];
% store maneuver loads
nttot = beam_model.Info.ntrim;
%
if isequal(stick.model.winr, 1)
  loads.wing.man.FS_i = [];
  loads.wing.man.M_i = [];
  loads.wing.man.Mt_i = [];
  nnodes = size(stick.nodes.winrC2_thick,2);
  loads.wing.man.FS = zeros(nnodes, nttot);
  loads.wing.man.M  = zeros(nnodes, nttot);
  loads.wing.man.Mt = zeros(nnodes, nttot);
%
  if (iter==1)
    geo.wing.t = WBOX_t(geo.wing.Zs, geo.wing.tbs, aircraft.weight_balance.COG(1,4,1)/pdcylin.wing.dsw/2, stick.nodes.winrC2_thick');
    coord = interp1(stick.nodes.winrC2_thick(2,:), geo.wing.Zs, (stick.nodes.winrC2_thick(2,1:end-1) + stick.nodes.winrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.winrC2_thick(2,:),geo.wing.tbs, (stick.nodes.winrC2_thick(2,1:end-1) + stick.nodes.winrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    for i=1:length(tbs)
      Mwin(i,1) = coord(i)*tbs(i) - (coord(i)-2*geo.wing.t)*(tbs(i)-2*geo.wing.t);
    end
    Mwin = pdcylin.wing.dsw * stick.wing.Lbeam_thick .* Mwin;
  else
    nc = pdcylin.stick.nwing_carryth_coarse;
    if (nc>0)
      Mwin = [ones(nc,1) .* str.wing.WTC/(2*nc);str.wing.WTBOX];
    else
      Mwin = str.wing.WTBOX;
    end
  end
  Mwin = mass_el2node(Mwin);
end
if isequal(stick.model.fuse, 1)
  loads.fus.man.FS_i = [];
  loads.fus.man.M_i = [];
  loads.fus.man.Mt_i = [];
  nnodes = size(stick.nodes.fuse_thick,2);
  loads.fus.man.FS = zeros(nnodes, nttot);
  loads.fus.man.M  = zeros(nnodes, nttot);
  loads.fus.man.Mt = zeros(nnodes, nttot);
  loads.fus.man.Nxt = zeros(nnodes, nttot);
  loads.fus.man.Nxc = zeros(nnodes, nttot);
%
  if (iter==1)
    geo.fus.t = FUSE_t(geo,aircraft.weight_balance.COG(5,4,1)/pdcylin.fus.ds); 
    rr = interp1(geo.fus.x, geo.fus.r, (stick.nodes.fuse_thick(1,1:end-1) + stick.nodes.fuse_thick(1,2:end))*0.5,'linear', 'extrap');
    for i=1:length(rr)
      Mfus(i,1) = 2*pi*rr(i)*geo.fus.t - pi*geo.fus.t^2;
    end
    Mfus = pdcylin.fus.ds * stick.fus.Lbeam_thick .* Mfus;
  else
    Mfus = str.fus.WTOT;
  end
  Mfus = mass_el2node(Mfus);
end
if isequal(stick.model.vert, 1)
  loads.vtail.man.FS_i = [];
  loads.vtail.man.M_i = [];
  loads.vtail.man.Mt_i = [];
  nnodes = size(stick.nodes.vert_thick,2);
  loads.vtail.man.FS = zeros(nnodes, nttot);
  loads.vtail.man.M  = zeros(nnodes, nttot);
  loads.vtail.man.Mt = zeros(nnodes, nttot);
%
  if (iter==1)
    geo.vtail.t = WBOX_t(geo.vtail.Zs, geo.vtail.tbs, aircraft.weight_balance.COG(4,4,1)/pdcylin.vtail.dsw, stick.nodes.vert_thick');
    coord = interp1(stick.nodes.vert_thick(3,:), geo.vtail.Zs, (stick.nodes.vert_thick(3,1:end-1) + stick.nodes.vert_thick(3,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.vert_thick(3,:),geo.vtail.tbs, (stick.nodes.vert_thick(3,1:end-1) + stick.nodes.vert_thick(3,2:end))*0.5,'linear', 'extrap');
    for i=1:length(tbs)
      Mvtail(i,1) = coord(i)*tbs(i) - (coord(i)-2*geo.vtail.t)*(tbs(i)-2*geo.vtail.t);
    end
    Mvtail = pdcylin.vtail.dsw * stick.vtail.Lbeam_thick .* Mvtail;
  else
    Mvtail = str.vtail.WTBOX;
  end
  Mvtail = mass_el2node(Mvtail);
end
if isequal(stick.model.horr, 1)
  loads.htail.man.FS_i = [];
  loads.htail.man.M_i = [];
  loads.htail.man.Mt_i = [];
  nnodes = size(stick.nodes.horrC2_thick,2);
  loads.htail.man.FS = zeros(nnodes, nttot);
  loads.htail.man.M  = zeros(nnodes, nttot);
  loads.htail.man.Mt = zeros(nnodes, nttot);
%
  if (iter==1)
    geo.htail.t = WBOX_t(geo.htail.Zs, geo.htail.tbs, aircraft.weight_balance.COG(3,4,1)/pdcylin.htail.dsw/2, stick.nodes.horrC2_thick');
    coord = interp1(stick.nodes.horrC2_thick(2,:), geo.htail.Zs, (stick.nodes.horrC2_thick(2,1:end-1) + stick.nodes.horrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.horrC2_thick(2,:),geo.htail.tbs,(stick.nodes.horrC2_thick(2,1:end-1) + stick.nodes.horrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    for i=1:length(tbs)
      Mhtail(i,1) = coord(i)*tbs(i) - (coord(i)-2*geo.htail.t)*(tbs(i)-2*geo.htail.t);
    end
    Mhtail = pdcylin.htail.dsw * stick.htail.Lbeam_thick .* Mhtail;
  else
    nc = pdcylin.stick.nhtail_carryth_coarse;
    if (nc>0)
      Mhtail = [ones(nc,1) .* str.htail.WTC/(2*nc);str.htail.WTBOX];
    else
      Mhtail = str.htail.WTBOX;
    end
  end
  Mhtail = mass_el2node(Mhtail);
end
if isequal(stick.model.canr, 1)
  loads.canard.man.FS_i = [];
  loads.canard.man.M_i = [];
  loads.canard.man.Mt_i = [];
  nnodes = size(stick.nodes.canrC2_thick,2);
  loads.canard.man.FS = zeros(nnodes, nttot);
  loads.canard.man.M  = zeros(nnodes, nttot);
  loads.canard.man.Mt = zeros(nnodes, nttot);
%
  if (iter==1)
    geo.canard.t = WBOX_t(geo.canard.Zs, geo.canard.tbs, aircraft.weight_balance.COG(11,4,1)/pdcylin.canard.dsw/2,stick.nodes.canrC2_thick');
    coord = interp1(stick.nodes.canrC2_thick(2,:), geo.canard.Zs, (stick.nodes.canrC2_thick(2,1:end-1) + stick.nodes.canrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    tbs  = interp1(stick.nodes.canrC2_thick(2,:),geo.canard.tbs, (stick.nodes.canrC2_thick(2,1:end-1) + stick.nodes.canrC2_thick(2,2:end))*0.5,'linear', 'extrap');
    for i=1:length(tbs)
      Mcanard(i,1) = coord(i)*tbs(i) - (coord(i)-2*geo.canard.t)*(tbs(i)-2*geo.canard.t);
    end
    Mcanard = pdcylin.canard.dsw * stick.canr.Lbeam_thick .* Mcanard;
  else
    nc = pdcylin.stick.ncanard_carryth_coarse;
    if (nc>0)
      Mcanard = [ones(nc,1) .* str.canard.WTC/(2*nc);str.canard.WTBOX];
    else
      Mcanard = str.canard.WTBOX;
    end
  end
  Mcanard = mass_el2node(Mcanard);
end
if isequal(stick.model.vert2, 1)
  nnodes = size(stick.nodes.vert2_thick,2);
  loads.vtail2.man.FS = zeros(nnodes, nttot);
  loads.vtail2.man.M  = zeros(nnodes, nttot);
  loads.vtail2.man.Mt = zeros(nnodes, nttot);
end
% safety margin
KS = pdcylin.smartcad.Ks;
%
if (isempty(Man_index))
  MAN_INDEX = [1:beam_model.Info.ntrim];
else
  MAN_INDEX = Man_index;
end
nt = length(unique(MAN_INDEX));
%
fprintf(fid,'\nNumber of flight points: %d.', nt);
fprintf(fid,'\nNumber of maneuvers: %d.', max(MAN_INDEX));
%
for i=1:nt
  mind = find(MAN_INDEX == i);
  for j=1:length(mind)
    fprintf(fid,'\n\nManeuver: %d.\n', beam_model.Aero.Trim.ID(mind(j)));
    outp = solve_free_lin_trim_guess_std(mind(j), Res{i});
    % store aero data for restart
    if (iter==1 && j==1)
      Res{i} = beam_model.Res;
    end
    % get aero loads
    manloads = interf_thick_model(pdcylin, geo, stick, beam_model.Aero, ...
                               0, beam_model.Res.Aero.results);
    ny = beam_model.Res.Aero.RTrim_sol.ACC(2);
    nz = beam_model.Res.Aero.RTrim_sol.ACC(3);
%
    DOTP = beam_model.Res.Aero.RTrim_sol.ACC(4);
    DOTQ = beam_model.Res.Aero.RTrim_sol.ACC(5);
    DOTR = beam_model.Res.Aero.RTrim_sol.ACC(6);
%
    loads.nv.N(mind(j)) = nz/9.81;
    loads.nv.V(mind(j)) = mach_alt2cas(beam_model.Aero.Trim.Mach(mind(j)), beam_model.Aero.Trim.ALT(mind(j)));
    loads.nv.V(mind(j)) = cas2eas(loads.nv.V(mind(j)),  beam_model.Aero.Trim.ALT(mind(j)));
  %
  % add extra loads
  %-------------------------------------------------------
  % WING
    if isequal(stick.model.winr, 1)
      % add structural contribution along wing
      [FSi, Mi] = Str_In_Force_H(geo.wing.y_nodes_thick, stick.nodes.winrC2_thick, Mwin, beam_model.WB.CG, nz, DOTP, DOTQ);
      manloads.winr.FS  = manloads.winr.FS + FSi;
      manloads.winr.M   = manloads.winr.M  + Mi;
      % add fuel contribution along wing
      [FSf, Mf] = Wing_Fuel(aircraft, pdcylin, geo, stick, beam_model.WB.CG, nz, DOTP, DOTQ);
      manloads.winr.FS  = manloads.winr.FS + FSf;
      manloads.winr.M   = manloads.winr.M  + Mf;
      % add engine along wing
      [FSe, Me, Mte] = Wing_Eng(aircraft, pdcylin, geo, stick, beam_model.WB.CG, nz, DOTP, DOTQ);
      manloads.winr.FS  = manloads.winr.FS + FSe;
      manloads.winr.M   = manloads.winr.M  + Me;
      manloads.winr.Mt  = manloads.winr.Mt + Mte;
      % add LG along wing
      % to do
      loads.wing.man.FS(:,mind(j)) = KS .* manloads.winr.FS;
      loads.wing.man.M(:,mind(j))  = KS .* manloads.winr.M;
      loads.wing.man.Mt(:,mind(j)) = KS .* manloads.winr.Mt;
    end

  %-------------------------------------------------------
  % FUSELAGE
    if isequal(stick.model.fuse, 1)
      mass = Fus_payload_mass(aircraft, geo, stick);
  %   differential pressure
      [NxP, Ny] = Fus_Press(aircraft, geo);
      loads.fus.Ny = KS .* Ny;
      NxP = KS .* NxP;
  %   engine thrust
      [NxA] = Fus_Thrust(aircraft, geo);
      NxA = KS .* NxA;
  %   landing
      if ( beam_model.Res.Extra.Fixed(2) && beam_model.Res.Extra.Fixed(3) )
        [Fland, Mland, loads.nv.N(mind(j))] = Fus_Lan(fid, aircraft, geo, nz, mass, beam_model, outp);
        manloads.fus.FS = manloads.fus.FS + Fland;
        manloads.fus.M = manloads.fus.M + Mland;
      else
  %   vertical acceleration
        [Facc, Macc]  = Fus_Acc(aircraft, geo, stick, mass, beam_model.WB.CG, nz, DOTQ);
        manloads.fus.FS = manloads.fus.FS + Facc;
        manloads.fus.M = manloads.fus.M + Macc;
      end 
      loads.fus.man.FS(:,mind(j)) = KS .* manloads.fus.FS;
      loads.fus.man.M(:,mind(j))  = KS .* manloads.fus.M;
      loads.fus.man.Mt(:,mind(j)) = KS .* manloads.fus.Mt;
      IndB = find(geo.fus.r ~= 0); 
      NxB = zeros(length(geo.fus.r),1);  
      NxB(IndB) = manloads.fus.M(IndB) ./ (pi*geo.fus.r(IndB).^2);
      % tensile stress in x direction
      loads.fus.man.Nxt(:,mind(j)) = ( NxB + NxP + NxA .* (NxA >= 0));
      % compressive stress in x direction
      loads.fus.man.Nxc(:,mind(j)) = abs(NxB + abs(NxA) .* (NxA < 0) - ...
                       NxP .* isequal(pdcylin.fact.stab, 1));
    end
    fclose(outp);
  %-------------------------------------------------------
  % VT
    if isequal(stick.model.vert, 1)
      [FSi, Mi] = Str_In_Force_V(geo.vtail.y_nodes_thick, stick.nodes.vert_thick, Mvtail, beam_model.WB.CG, ny, DOTP, DOTR);
      manloads.vert.FS  = manloads.vert.FS + FSi;
      manloads.vert.M   = manloads.vert.M  + Mi;
      loads.vtail.man.FS(:,mind(j)) = KS .* manloads.vert.FS;
      loads.vtail.man.M(:,mind(j)) =  KS .* manloads.vert.M;
      loads.vtail.man.Mt(:,mind(j)) = KS .* manloads.vert.Mt;
    end
  %-------------------------------------------------------
  % HT
    if isequal(stick.model.horr, 1)
      [FSi, Mi] = Str_In_Force_H(geo.htail.y_nodes_thick, stick.nodes.horrC2_thick, Mhtail, beam_model.WB.CG, nz, DOTP, DOTQ);
      manloads.horr.FS  = manloads.horr.FS + FSi;
      manloads.horr.M   = manloads.horr.M  + Mi;
      loads.htail.man.FS(:,mind(j)) = KS .* manloads.horr.FS;
      loads.htail.man.M(:,mind(j)) =  KS .* manloads.horr.M;
      loads.htail.man.Mt(:,mind(j)) = KS .* manloads.horr.Mt;
    end
  %-------------------------------------------------------
  % CANARD
    if isequal(stick.model.canr, 1)
      [FSi, Mi] = Str_In_Force_H(geo.canard.y_nodes_thick, stick.nodes.canrC2_thick, Mcanard, beam_model.WB.CG, nz, DOTP, DOTQ);
      manloads.canr.FS  = manloads.canr.FS + FSi;
      manloads.canr.M   = manloads.canr.M  + Mi;
      loads.canard.man.FS(:,mind(j)) = KS .* manloads.canr.FS;
      loads.canard.man.M(:,mind(j)) =  KS .* manloads.canr.M;
      loads.canard.man.Mt(:,mind(j)) = KS .* manloads.canr.Mt;
    end
  %-------------------------------------------------------
  % TWIN TAIL
  %
    if isequal(stick.model.vert2, 1)
      loads.vtail2.man.FS(:,mind(j)) = KS .* manloads.vert2.FS;
      loads.vtail2.man.M(:,mind(j)) =  KS .* manloads.vert2.M;
      loads.vtail2.man.Mt(:,mind(j)) = KS .* manloads.vert2.Mt;
    end
%
  end % internal loop
end % external loop
%
% take maximum value for each section
% exclude from loads set carrytrough 
if isequal(stick.model.winr, 1)
  offset = pdcylin.stick.nwing_carryth+1;
  [loads.wing.FS, loads.wing.man.FS_i]   = max(abs(loads.wing.man.FS(offset:end,:)), [], 2);
  [loads.wing.M,  loads.wing.man.M_i]     = max(abs(loads.wing.man.M(offset:end,:)), [], 2);
  [loads.wing.Mt, loads.wing.man.Mt_i]   = max(abs(loads.wing.man.Mt(offset:end,:)), [], 2);
  loads.wing.N = zeros(length(loads.wing.FS),1);
end
%
if isequal(stick.model.vert, 1)
  [loads.vtail.FS, loads.vtail.man.FS_i]  = max(abs(loads.vtail.man.FS), [], 2);
  [loads.vtail.M,  loads.vtail.man.M_i]   = max(abs(loads.vtail.man.M), [], 2);
  [loads.vtail.Mt, loads.vtail.man.Mt_i]  = max(abs(loads.vtail.man.Mt), [], 2);
  loads.vtail.N = zeros(length(loads.vtail.FS),1);
end
%
if isequal(stick.model.horr, 1)
  offset = pdcylin.stick.nhtail_carryth+1;
  [loads.htail.FS, loads.htail.man.FS_i]  = max(abs(loads.htail.man.FS(offset:end,:)), [], 2);
  [loads.htail.M,  loads.htail.man.M_i]   = max(abs(loads.htail.man.M(offset:end,:)), [], 2);
  [loads.htail.Mt, loads.htail.man.Mt_i] = max(abs(loads.htail.man.Mt(offset:end,:)), [], 2);
  loads.htail.N = zeros(length(loads.htail.FS),1);
end
%
if isequal(stick.model.canr, 1)
  offset = pdcylin.stick.ncanard_carryth+1;
  [loads.canard.FS, loads.canard.man.FS_i] = max(abs(loads.canard.man.FS(offset:end,:)), [], 2);
  [loads.canard.M,  loads.canard.man.M_i]   = max(abs(loads.canard.man.M(offset:end,:)), [], 2);
  [loads.canard.Mt, loads.canard.man.Mt_i]  = max(abs(loads.canard.man.Mt(offset:end,:)), [], 2);
  loads.canard.N = zeros(length(loads.canard.FS),1);
end
%
if isequal(stick.model.fuse, 1)
  loads.fus.Nxc  = max(loads.fus.man.Nxc, [], 2);
  loads.fus.Nxt  = max(loads.fus.man.Nxt, [], 2);
  [loads.fus.FS, loads.fus.man.FS_i]    = max(abs(loads.fus.man.FS), [], 2);
  [loads.fus.M,  loads.fus.man.M_i]    = max(abs(loads.fus.man.M), [], 2);
  [loads.fus.Mt, loads.fus.man.Mt_i]   = max(abs(loads.fus.man.Mt), [], 2);
end
%
end
%--------------------------------------------------------------------------------------------------
function [FSe, Me, Mte] = Wing_Eng(aircraft, pdcylin, geo, stick, CG, NZ, DOTP, DOTQ)
% Initialize
ne = length(geo.wing.cg) +1;
FSe1 = zeros(ne, 1);
Me1 = zeros(ne, 1);
Mte1 = zeros(ne, 1);
FSe2 = zeros(ne, 1);
Me2 = zeros(ne, 1);
Mte2 = zeros(ne, 1);
%
FS = zeros(ne, 1);
M = zeros(ne, 1);
Mt = zeros(ne, 1);
% Engine1
if (aircraft.engines1.Number_of_engines ~= 0)
  if (aircraft.engines1.Layout_and_config == 0 || aircraft.engines1.Layout_and_config == 1 ||...
    aircraft.engines1.Layout_and_config == 2)
    % Location along global y axis, measured from fuse/wing intersection
%    ye1g = abs(aircraft.engines1.Location_engines_nacelles_on_Y - geo.fus.R);
    xe1g = abs(aircraft.weight_balance.COG(7,1,1));
    ye1g = abs(aircraft.weight_balance.COG(7,2,1));
    % Location along the structural elastic line
    %ye1 = ye1g / (geo.wing.b/2) * geo.wing.bS;
    he1 = (geo.wing.y_nodes_thick <= ye1g);
    index = find(he1==0);
    index = index(1)-1;
    arm = 0;
    dist = zeros(3,1);
    if (index>0)
      dist = aircraft.weight_balance.COG(7,1:3,1) - [stick.nodes.winrC2_thick(1,index), stick.nodes.winrC2_thick(2,index), aircraft.weight_balance.COG(7,3,1)];  
    end
    % weight of engine1 on semiwing
    M1 = (aircraft.weight_balance.COG(7,4,1)* (-NZ - DOTP * (ye1g-CG(2)) + DOTQ * (xe1g-CG(1)))) / aircraft.engines1.Number_of_engines;
    % Shear force and bending moment
    FSe1 = M1 .* he1;
    Me1  = M1 .* he1 .* (ye1g-geo.wing.y_nodes_thick);
    vec = -crossm(dist); arm = vec(2,3);
    Mte1 = (M1 * arm) .* he1;
  end
end
% Engine2
if (aircraft.engines2.Number_of_engines ~= 0)

  if (aircraft.engines2.Layout_and_config == 0 || aircraft.engines2.Layout_and_config == 1 ||...
    aircraft.engines2.Layout_and_config == 2)
    % Location along global y axis, measured from fuse/wing intersection
    xe2g = abs(aircraft.weight_balance.COG(8,1,1));
    ye2g = abs(aircraft.weight_balance.COG(8,2,1));
    % Location along the structural elastic line
%    ye2 = ye2g / (geo.wing.b/2-geo.fus.R) * geo.wing.bS;
    he2 = (geo.wing.y <= ye2g);
    index = find(he2==0);
    index = index(1)-1;
    arm = 0;
    dist = zeros(3,1);
    if (index>0)
      dist = aircraft.weight_balance.COG(8,1:3,1) - [geo.wing.x(index), geo.wing.y(index), aircraft.weight_balance.COG(8,3,1)];  
    end
    % weight of engine1 on semiwing
    M2 = (aircraft.weight_balance.COG(8,4,1)* (-NZ - DOTP * (ye2g-CG(2)) + DOTQ * (xe2g-CG(1)))) / aircraft.engines2.Number_of_engines;
    % Shear force and bending moment
    FSe2 = M2 .*he2;
    Me2  = M2 .*he2 .*(ye2g-geo.wing.y);
    vec = -crossm(dist); arm = vec(2,3);
    Mte2 = (M2*arm) .* he2;
  end
end

FSe = FSe1 + FSe2;
Me  = Me1 + Me2;
Mte = Mte1 + Mte2;
%

%
end
%--------------------------------------------------------------------------------------------------
function [FSf, Mf] = Wing_Fuel(aircraft, pdcylin, geo, stick, CG, NZ, DOTP, DOTQ)
%
% initialize
  offset = pdcylin.stick.nwing_carryth;
  ne = length(geo.wing.cg)+1;
  FSf = zeros(ne, 1);
  Mf =  zeros(ne, 1);
%
  mass_FUEL = aircraft.weight_balance.COG(18,4,1)/2;
  if (mass_FUEL>0)
    dV = geo.wing.V;
    % Compute outboard limit
    bfuel = aircraft.fuel.Outboard_fuel_tank_span * geo.wing.b*0.5;
    % Exclude carry-through (which means central fuel tank)
    ind1 = pdcylin.stick.nwing_carryth+1;
    % Find outboard tank index
    ind2 = find(0.5*(stick.nodes.winrC2_thick(2,1:end-1) + stick.nodes.winrC2_thick(2,2:end))<=bfuel);
    ind2 = ind2(end);
    % Initialize mass vector
    mass_fuel = zeros(size(stick.wing.Lbeam_thick));
    % Compute lumped masses using wing-box volume fraction to split fuel [Kg]
    mass_fuel(ind1:ind2) = mass_FUEL.*(dV(ind1:ind2)./sum(dV(ind1:ind2)));
%
    m = mass_el2node(mass_fuel);
% add ang acceleration contribution
    mang = -m * NZ - DOTP .* m .* (stick.nodes.winrC2_thick(2,:)-CG(2))' ...
           + DOTQ .* m .* (stick.nodes.winrC2_thick(1,:)-CG(1))';
    m = m + mang;
%
    for i=1:length(m)
      FSf(i) = sum(m(end:-1:i));
    end    
%
    for i=ne-1:-1:1
      Mf(i)  = sum(m(end:-1:i) .* (geo.wing.y_nodes_thick(end:-1:i)-geo.wing.y_nodes_thick(i)));
    end
    for i=offset:-1:1
      Mf(i) = Mf(i+1);
    end
  end
end
%--------------------------------------------------------------------------------------------------
function [Fland, Mland, nz] = Fus_Lan(fid, aircraft, geo, acc, mass, beam_model, outp)
%
  VSINK  = beam_model.Aero.Trim.Extra.Value(2);
  STROKE = beam_model.Aero.Trim.Extra.Value(3);
  ETA    = beam_model.Aero.Trim.Extra.Value(4);
  MASS   = beam_model.WB.MCG(1,1);
  g = 9.81;
  nz = 1;
  fcr = acc/g;
  %
  fprintf(fid, '\n\t\t- Landing weight fraction: %g.',   fcr);
  fprintf(fid, '\n\t\t- Landing struct efficiency: %g.', ETA);
  fprintf(fid, '\n\t\t- Landing sink velocity: %g [m/s].',     VSINK);
  fprintf(fid, '\n\t\t- Landing stroke length: %g [m].',     STROKE);

  Fland = zeros(geo.fus.lenx,1);
  Mland = zeros(geo.fus.lenx,1);
  % LG coordinate
  Xcg = aircraft.weight_balance.COG(6,1,1);
  if (Xcg>0) 
    [dCG, pos] = min(abs(geo.fus.x - Xcg));
    % Landing
    nz = (0.5 *(VSINK^2)/g + fcr * STROKE) / (ETA * STROKE) +1;
    % Landing reaction
    nzr = (0.5 *(VSINK^2)/g + fcr * STROKE) / (ETA * STROKE);
    R = nzr * sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1)) * g * fcr;
    fprintf(fid, '\n\t\t- Ground reaction: %g [N].', R);
    fprintf(fid, '\n\t\t- Landing load factor: %g.', nz);
    Jin = diag(aircraft.weight_balance.Imat);
    TETHA = (R * (Xcg - beam_model.WB.CG(1))) / Jin(2);
    fprintf(fid, '\n\t\t- Angular acceleration: %g [rad/s^2].\n', TETHA);
    dweight = mass .* (nz * g) - mass .* (TETHA * (beam_model.WB.CG(1)-geo.fus.x));
    dweight = fcr .* dweight;
    fprintf(outp,'\n\nLANDING MANEUVER\n');
    fprintf(outp, '\n\nCORRECTIONS TO RIGID TRIM\n');
    fprintf(outp,'\n - Z acc:      %g [m/s^2].', nz*9.81);
    fprintf(outp,'\n - Q-DOT:      %g [rad/s^2].', TETHA);
    fprintf(fid,'\n - Z acc:      %g [m/s^2].', nz*9.81);
    fprintf(fid,'\n - Q-DOT:      %g [rad/s^2].', TETHA);
  %------------------------------------
    Fland = -dweight;
    % Include LG reaction
    Fland(pos) = Fland(pos) + R;
    % Bending moment to vertical forces ONLY
    for j = 1:geo.fus.lenx
      Mland(j) = -sum( Fland(1:j).*( geo.fus.x(1:j) - geo.fus.x(j)) );
    end
    Fland = cumsum(Fland);
  end
end
%--------------------------------------------------------------------------------------------------
function [NxA] = Fus_Thrust(aircraft, geo)
% initialize
NxA = zeros(geo.fus.lenx,1);
inertia = zeros(geo.fus.lenx, 1);
% convert engine thrust from [KN] to [N]
scale = 1000;

%--------------------------------------------------------------------------
% Engine1
%
if ~isequal(aircraft.engines1.Number_of_engines, 0) && ~isequal(aircraft.engines1.Max_thrust, 0)
    
    % midpoint coordinate
    Xcg = aircraft.engines1.Location_engines_nacelles_on_X + aircraft.engines1.nacelle_length/2;
    Ycg = aircraft.engines1.Location_engines_nacelles_on_Y;
    Zcg = aircraft.engines1.Location_engines_nacelles_on_Z; 
    % figure out type of mounted propulsion pod
    [XE1] = eng_loc_def(aircraft.engines1.Layout_and_config, Xcg, Ycg, geo.wing.C2(1,1));
    
    % inertia forces ahead and behind engine location caused by engine
    % thrust [N]
    inertia = inertia + scale *aircraft.engines1.Max_thrust *( - (geo.fus.x < XE1) + (geo.fus.x >= XE1) );
    
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Engine2
%
if ~isequal(aircraft.engines2.Number_of_engines, 0) && ~isequal(aircraft.engines2.Max_thrust, 0)
    
    % midpoint coordinate
    Xcg = aircraft.engines2.Location_engines_nacelles_on_X + aircraft.engines2.nacelle_length/2;
    Ycg = aircraft.engines2.Location_engines_nacelles_on_Y;
    Zcg = aircraft.engines2.Location_engines_nacelles_on_Z; 
    % figure out type of mounted propulsion pod
    [XE2] = eng_loc_def(aircraft.engines2.Layout_and_config, Xcg, Ycg, geo.wing.C2(1,1));
    
    % inertia forces ahead and behind engine location caused by engine
    % thrust [N]
    inertia = inertia + scale *aircraft.engines1.Max_thrust *( - (geo.fus.x < XE2) + (geo.fus.x >= XE2) );
    
end
%--------------------------------------------------------------------------
% Fuselage volume distribution over the same domain as inertia forces above
% computed
weight_frc = geo.fus.deltaVolFus;
% domain_weight_frc = ( geo.fus.x(1:end-1) + geo.fus.x(2:end) )./2;
domain_weight_frc = geo.fus.x + [diff(geo.fus.x)/2; 0];
% domain_weight_frc = [geo.fus.x(1); domain_weight_frc; geo.fus.x(end)];
% interpolate over nodes
weight_frc_distr = interp1(domain_weight_frc, weight_frc, geo.fus.x, 'linear', 'extrap');

% Distribuite inertia forces accordingly to fuselage volume distribution
inertia_distr = inertia .*weight_frc_distr;

% Stress resultant in the axial direction
indx = find( geo.fus.P ~= 0 );
loads.fus.NxA(indx) = inertia_distr(indx) ./ geo.fus.P(indx);

end % end of Load_Fus_Acc.m, DO NOT REMOVE

%--------------------------------------------------------------------------------------------------
function [x_eng] = eng_loc_def(layout, Xcg, Ycg, Xc2)
% INPUT
%   layout, from the XML file to define engine type
%   Xcg, engine longitudinal location
%   Ycg, engine lateral location
%   Xc2, wing-fuselage intersection (respect the elastic line)

switch layout
    case 0  % attached by a pylon under or over the WING
        x_eng = Xc2;
    case 1  % on WING nacelle with an inverted U section
        x_eng = Xc2;
    case 2  % on WING nacelle with an elongated O section
        x_eng = Xc2;
    case {3,6}  % FUSELAGE mounted engines attached by pylons
        x_eng = Xcg;
end

end 
%--------------------------------------------------------------------------------------------------
function [Facc, Macc] = Fus_Acc(aircraft, geo, stick, mass, CG, nz, DOTQ)
%
  Facc = zeros(geo.fus.lenx,1);
  Macc = zeros(geo.fus.lenx,1);
  Finer = -mass .* nz + DOTQ * mass .* (geo.fus.x-CG(1));
  % Bending moment to vertical forces ONLY
  for j = 1:geo.fus.lenx
    Macc(j) = -sum( Finer(1:j).*( geo.fus.x(1:j) - geo.fus.x(j)) );
  end
  Facc = cumsum(Finer);
end
%--------------------------------------------------------------------------------------------------
function [m] = Fus_payload_mass(aircraft, geo, stick)
%
  fcr = 1.0;
  Facc = zeros(geo.fus.lenx,1);
  Macc = zeros(geo.fus.lenx,1);
  dweight = zeros(length(geo.fus.x),1);
  % Inertia forces
  for i = 1 : size(aircraft.weight_balance.COG(:,1,1),1)
    switch i
      case 7 %engine 1
          add = 0;
          if (aircraft.engines1.Layout_and_config == 3 || aircraft.engines1.Layout_and_config == 6 )
            [dummy,ind] = min(abs(geo.fus.x - aircraft.weight_balance.COG(i,1,1)));
          else
            dist = abs(geo.fus.x - geo.wing.x_inboard(1));
            [dummy,ind] = min(dist);
          end
          dweight(ind) = dweight(ind) + (aircraft.weight_balance.COG(i,4,1) * fcr);

      case  8 %engine 2
          add = 0;
          if (aircraft.engines2.Layout_and_config == 3 || aircraft.engines2.Layout_and_config == 6 )
            [dummy,ind] = min(abs(geo.fus.x - aircraft.weight_balance.COG(i,1,1)));
          else
            dist = abs(geo.fus.x - geo.wing.x_inboard(1));
            [dummy,ind] = min(dist);
          end
          dweight(ind) = dweight(ind) + (aircraft.weight_balance.COG(i,4,1) * fcr);

      case  {17, 21, 22, 23, 24, 25} 
        add = 0;

      otherwise
        add = 1;
    end
    if (aircraft.weight_balance.COG(i,1,1) ~= 0 && add == 1)
      [dummy,ind] = min(abs(geo.fus.x - aircraft.weight_balance.COG(i,1,1)));
      dweight(ind) = dweight(ind) + (aircraft.weight_balance.COG(i,4,1) * fcr);
    end
  end
%
  dV    = geo.fus.V;
  dSwet  = geo.fus.Swet;
  nm = fieldnames(geo);
  Swettot = 0;
  nodes = geo.fus.x_nodes_thick;
  for k = 1:numel(nm),
      if ~isempty(geo.(nm{k}))
          switch nm{k} 
              case {'fus', 'vtail'}
                  Swettot = Swettot + sum(geo.(nm{k}).Swet);
              case {'wing', 'wing2', 'htail', 'canard', 'tbooms'}
                  Swettot = Swettot + 2*sum(geo.(nm{k}).Swet);
          end
      end
  end
  ne = length(stick.fus.Lbeam_thick);
  mdstr = zeros(ne,1);
  domain = [0; geo.fus.bodl];
  massINT  = aircraft.weight_balance.COG(21,4,1) - paint_weight(Swettot);
  mass_int = massNSM_distr(nodes, domain, massINT, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_int;
%
  massFUR = aircraft.weight_balance.COG(17,4,1);
  mass_fur = massNSM_distr(nodes, domain, massFUR, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_fur;
%
  massBAG = aircraft.weight_balance.COG(25,4,1);
  mass_bag = massNSM_distr(nodes, domain, massBAG, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_bag;
%
  domain = [geo.fus.lengthN; aircraft.cabin.Cabin_length_to_aft_cab];
  massCRE = aircraft.weight_balance.COG(23,4,1);
  mass_cre = massNSM_distr(nodes, domain, massCRE, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_cre;
%
  massPAS = aircraft.weight_balance.COG(24,4,1);
  mass_pas = massNSM_distr(nodes, domain, massPAS, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_pas;
%
  domain = [0; geo.fus.lengthN];
  massPIL = aircraft.weight_balance.COG(22,4,1);
  mass_pil = massNSM_distr(nodes, domain, massPIL, dV, dV, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_pil;
%
  domain = [0; geo.fus.bodl];
  massPNT  = paint_weight(Swettot)*sum(geo.fus.Swet)/Swettot;
  mass_pnt = massNSM_distr(nodes, domain, massPNT, dSwet, dSwet, stick.fus.Lbeam_thick);
  mdstr = mdstr + mass_pnt;
  %
  mbeam = mdstr .* stick.fus.Lbeam_thick;
  m = zeros(ne+1,1);
  for i=1:ne
    v = 0.5 * mbeam(i);
    m(i)   = v + m(i);
    m(i+1) = v + m(i+1);
  end
%
  m = m + dweight;
end
%--------------------------------------------------------------------------------------------------
function m = mass_el2node(mbeam)
%
  ne = length(mbeam);
  m = zeros(ne+1,1);
  for i=1:ne
    v = 0.5 * mbeam(i);
    m(i)   = v + m(i);
    m(i+1) = v + m(i+1);
  end
end
%--------------------------------------------------------------------------------------------------
function [FSf, Mf] = Str_In_Force_H(Y, NODES, DMASS, CG, NZ, DOTP, DOTQ)
%
% initialize
  ne = length(DMASS);
  FSf = zeros(ne, 1);
  Mf =  zeros(ne, 1);
%
  m = DMASS;
% add ang acceleration contribution
  mang = -m * NZ - DOTP .* m .* (NODES(2,:)-CG(2))' ...
         + DOTQ .* m .* (NODES(1,:)-CG(1))';
%
  for i=1:length(mang)
    FSf(i) = sum(m(end:-1:i));
  end    
%
  for i=ne-1:-1:1
    Mf(i)  = sum(mang(end:-1:i) .* (Y(end:-1:i)-Y(i)));
  end
end

%--------------------------------------------------------------------------------------------------
function [FSf, Mf] = Str_In_Force_V(Y, NODES, DMASS, CG, NY, DOTP, DOTR)
%
% initialize
  ne = length(DMASS);
  FSf = zeros(ne, 1);
  Mf =  zeros(ne, 1);
%
  m = DMASS;
% add ang acceleration contribution
  mang = -m * NY + DOTP .* m .* (NODES(2,:)-CG(2))' ...
         - DOTR .* m .* (NODES(1,:)-CG(1))';
%
  for i=1:length(mang)
    FSf(i) = sum(mang(end:-1:i));
  end    
%
  for i=ne-1:-1:1
    Mf(i)  = sum(mang(end:-1:i) .* (Y(end:-1:i)-Y(i)));
  end
end