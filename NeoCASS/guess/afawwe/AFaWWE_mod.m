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
%   Author: <andreadr@kth.se>
%
% Main script to performed Generic Unknown Estimator for Structural Sizing
%
%
% Called by:    guess.m
%
% Calls:        Geo_Fus.m , Geo_Wing.m , Geo_Vtail.m , Geo_Htail.m
%               Load_Fus.m, Load_Wing.m, Load_Vtail.m, Load_Htail.m
%               Str_Fus.m , Str_Wing.m , Str_Vtail.m , Str_Htail.m
%               Regr_Fus.m, Regr_Wing.m, Regr_Vtail.m, Regr_Htail.m,
%               set_stability_der.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080407      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [pdcylin, geo, loads, str, aircraft, optim] = AFaWWE_mod(fid, niter, pdcylin, aircraft, ...
           filename_tech, max_loads, NZ, VEL, stick, optim, geo, ntrim, nconf)
RTOLL = 1.0e-6;
%
%--------------------------------------------------------------------------
% Create structure
%--------------------------------------------------------------------------

% Commented here to keep geometrical information
% geo.fus     = [];     % Geometry for FUSELAGE
% geo.wing    = [];     % Geometry for WING
% geo.vtail   = [];     % Geometry for VERTICAL TAIL
% geo.htail   = [];     % Geometry for HORIZONTAL TAIL
% geo.canard   = [];     % Geometry for CANARD
% geo.tbooms  = [];
%
str.fus     = [];     % Structure for FUSELAGE
str.wing    = [];     % Structure for WING
str.vtail   = [];     % Structure for VERTICAL TAIL
str.htail   = [];     % Structure for HORIZONTAL TAIL
str.canard   = [];     % Structure for WING
str.M       = 0;      % scalar, to determine the mass computed and written in the *.dat file [kg]

%
%--------------------------------------------------------------------------
% User-input definition of engines attachement in case they are originally
% defined as "floating" propulsion pods.
%--------------------------------------------------------------------------
% 
aircraft = user_input_engines_attach(fid, aircraft);

%--------------------------------------------------------------------------
% GEOMETRY MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- GEOMETRY ----------------------------------------------');
fprintf(fid, '\n\t- Creating baseline geometry...');
geo = Geo_Fus(pdcylin, aircraft, geo);
% update radius and perimeter
geo.fus.r = interp1(geo.fus.x, geo.fus.r, stick.nodes.fuse_thick(1,:)', 'linear', 'extrap');
index = find(geo.fus.r<RTOLL);
geo.fus.r(index) = 0.0;
geo.fus.P = interp1(geo.fus.x, geo.fus.P, stick.nodes.fuse_thick(1,:)', 'linear', 'extrap');
geo.fus.P(index) = 0.0;
%
[geo, pdcylin] = Geo_Wing(pdcylin, aircraft, geo);
if isfield(aircraft,'Tailbooms')
  if aircraft.Tailbooms.present
    geo = Geo_Tbooms(pdcylin, aircraft, geo);
    geo.tbooms.x_nodes = stick.nodes.tboomsr_thick(1,:);
  end
end
if aircraft.Vertical_tail.present
  [geo, pdcylin] = Geo_Vtail(pdcylin, aircraft, geo);
end
if aircraft.Horizontal_tail.present
  [geo, pdcylin] = Geo_Htail(pdcylin, aircraft, geo);
end
if aircraft.Canard.present
  [geo, pdcylin] = Geo_Canard(pdcylin, aircraft, geo);
end
if aircraft.wing2.present
  [geo, pdcylin] = Geo_Wing2(pdcylin, aircraft, geo);
else
  pdcylin.stick.model.win2r = 0;
end
fprintf(fid, 'done.');
%--------------------------------------------------------------------------
% LOADS MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- LOADS ----------------------------------------------');
loads = Load_mod(fid, pdcylin, aircraft, geo, max_loads, NZ, VEL, ntrim, nconf);
%--------------------------------------------------------------------------
% STRUCTURAL MODULUS
%--------------------------------------------------------------------------
% 
fprintf(fid,'\n\t-------------------------------------------- SIZING ------------------------------------------------');
if isequal(pdcylin.stick.model.fuse, 1)
    fprintf(fid, '\n\t- Fuselage structural sizing...');
    [str] = Str_Fus(pdcylin, aircraft, geo, loads, str);
    fprintf(fid, '\n\tdone.');
end

if isequal(pdcylin.stick.model.winr, 1)
    fprintf(fid, '\n\t- Wing structural sizing...');
    [str, optim] = Str_Wing(niter, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end

if isequal(pdcylin.stick.model.win2r, 1)
    fprintf(fid, '\n\t- Wing2 structural sizing...');
    [str, optim] = Str_Wing2(niter, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end

if isequal(pdcylin.stick.model.vert, 1)
    fprintf(fid, '\n\t- Vertical tail structural sizing...');
    [str, optim] = Str_Vtail(niter, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end

if isequal(pdcylin.stick.model.horr, 1)
   fprintf(fid, '\n\t- Horizontal tail structural sizing...');
   [str, optim] = Str_Htail(niter, pdcylin, aircraft, geo, loads, str, optim);
   fprintf(fid, '\n\tdone.');
end

if aircraft.Canard.present
    fprintf(fid, '\n\t- Canard structural sizing...');
    [str, optim] = Str_Canr(niter, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end

if isfield(aircraft, 'Tailbooms') && aircraft.Tailbooms.present
    fprintf(fid, '\n\t- Tailbooms structural sizing...');
    str = Str_Tbooms(pdcylin, aircraft, geo, loads, str);
    fprintf(fid, '\n\tdone.');
end

%--------------------------------------------------------------------------
% REGRESSION ANALYSIS MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- REGRESSION --------------------------------------------');
if isequal(pdcylin.stick.model.fuse, 1) 
    fprintf(fid, '\n\t- Fuselage regression equation, ');
    [str] = Regr_Fus(fid, pdcylin, aircraft, geo, loads, str);
end

if isequal(pdcylin.stick.model.winr, 1)
    fprintf(fid, '\n\t- Wing regression equation, ');
    [str] = Regr_Wing(fid, pdcylin, aircraft, geo, loads, str);
end

if isequal(pdcylin.stick.model.vert, 1)
    fprintf(fid, '\n\t- Vertical tail regression equation, ');
    [str] = Regr_Vtail(fid, pdcylin, aircraft, geo, loads, str);
end

if isequal(pdcylin.stick.model.horr, 1)
   fprintf(fid, '\n\t- Horizontal tail regression equation, ');
   [str] = Regr_Htail(fid, pdcylin, aircraft, geo, loads, str);
end

if aircraft.Canard.present
    fprintf(fid, '\n\t- Canard tail regression equation, ');
    [str] = Regr_Canard(fid, pdcylin, aircraft, geo, loads, str);
end

if isfield(aircraft,'Tailbooms')
   if aircraft.Tailbooms.present
       fprintf(fid, '\n\t- Tailbooms regression equation');
       [str] = Regr_Tbooms(fid, pdcylin, aircraft, geo, loads, str);
   end
end

%**********************************************************************************************************************
% Introduce the carrythrough structure in wing and horizontal formulation. Until now parameters have been defined only
% for the exposed surface (i.e. from wing/fuse and HT/VT or HT/fuse intersection). Now the following parameters have 
% been updated to consider the structure within other component. 
%**********************************************************************************************************************

%--------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.winr, 1) && isequal(pdcylin.stick.model.fuse, 1)
    % Number of nodes within half-carrythrough structure
    nrcth = pdcylin.stick.nwing_carryth;
    % GEO
    geo.wing = add_ct_aero(geo.wing, geo.fus.R, pdcylin.stick.nwing_carryth, ...
                pdcylin.stick.nwing_carryth_coarse);
    geo.wing = add_ct_geo(geo.wing, nrcth);
    % STR
    if pdcylin.wing.kcon <=6
        str.wing = add_ct_ardema(str.wing, nrcth);
    else
        switch pdcylin.wing.kcon
            
            case {9}
                pdcylin.optimization_smonoq = 0;
                str.wing = add_ct_9(str.wing, nrcth);
            case {10}
                pdcylin.optimization_smonoq = 0;
                str.wing = add_ct_10(str.wing, nrcth);

        end
    end
    
    % CAERO1
    geo.wing = add_ct_caero(geo.wing, geo.fus.R, pdcylin.stick.model.symmXZ);
    geo.wing.CAERO1.sup_control.frs = [1.0; geo.wing.CAERO1.sup_control.frs];
end


%--------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.horr, 1) && geo.htail.twc > 0.0
    % Number of nodes within half-carrythrough structure
    nrcth = pdcylin.stick.nhtail_carryth;
    % GEO
    geo.htail = add_ct_aero(geo.htail, geo.htail.twc/2, ...
                  pdcylin.stick.nhtail_carryth, pdcylin.stick.nhtail_carryth_coarse);
    geo.htail = add_ct_geo(geo.htail, nrcth);
    % STR
    if pdcylin.htail.kcon <=6
      str.htail = add_ct_ardema(str.htail, nrcth);
    else
        switch pdcylin.htail.kcon
            
            case {9}
                pdcylin.optimization_smonoq = 0;
                str.htail = add_ct_9(str.htail, nrcth);
            case {10}
                pdcylin.optimization_smonoq = 0;
                str.htail = add_ct_10(str.htail, nrcth);
        end
    end
    % CAERO1
    geo.htail = add_ct_caero(geo.htail, geo.htail.twc/2, pdcylin.stick.model.symmXZ);
end

%--------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.canr, 1) && geo.canard.twc > 0.0
    
    % Number of nodes within half-carrythrough structure
    nrcth = pdcylin.stick.ncanard_carryth;
    % GEO
    geo.canard = add_ct_aero(geo.canard, geo.canard.twc/2, pdcylin.stick.ncanard_carryth, ...
                  pdcylin.stick.ncanard_carryth_coarse);
    geo.canard = add_ct_geo(geo.canard, nrcth);
    % STR
    if pdcylin.canard.kcon <=6
      str.canard = add_ct_ardema(str.canard, nrcth);
    else
        switch pdcylin.canard.kcon

            case {8}
                pdcylin.optimization_smonoq = 0;
                str.canard = add_ct_8(str.canard, nrcth);
            case {9}
                pdcylin.optimization_smonoq = 0;
                str.canard = add_ct_9(str.canard, nrcth);
            case {10}
                pdcylin.optimization_smonoq = 0;
                str.canard = add_ct_10(str.canard, nrcth);

        end
    end
    geo.canard = add_ct_caero(geo.canard, geo.canard.twc/2, pdcylin.stick.model.symmXZ);
    
end

end

function res = add_ct(in, nct)
  v = in(1);
  res = [v.*ones(nct,1); in];
end

function in = add_ct_8(in, nrcth)
  in.skin.tskin = add_ct(in.skin.tskin, nrcth);  
end

function in = add_ct_9(in, nrcth)
  in.web.tw     = add_ct(in.web.tw, nrcth);     
  in.skin.tskin = add_ct(in.skin.tskin, nrcth);  
  in.skin.Astr  = add_ct(in.skin.Astr, nrcth);   
  in.skin.Nstr  = add_ct(in.skin.Nstr, nrcth);   
end

function in = add_ct_10(in, nrcth)
  in.web.Acap   = add_ct(in.web.Acap, nrcth);   
  in.web.tw     = add_ct(in.web.tw, nrcth);     
  in.web.B1_cap = add_ct(in.web.B1_cap, nrcth); 
  in.web.T1_cap = add_ct(in.web.T1_cap, nrcth); 
  in.web.B2_cap = add_ct(in.web.B2_cap, nrcth); 
  in.web.T2_cap = add_ct(in.web.T2_cap, nrcth); 
  in.web.Bu_stf = add_ct(in.web.Bu_stf, nrcth); 
  in.web.tu_stf = add_ct(in.web.tu_stf, nrcth); 
  in.web.du_stf = add_ct(in.web.du_stf, nrcth); 
  in.skin.tskin = add_ct(in.skin.tskin, nrcth);  
  in.skin.Astr  = add_ct(in.skin.Astr, nrcth);   
  in.skin.Nstr  = add_ct(in.skin.Nstr, nrcth);
  in.skin.trib  = add_ct(in.skin.trib, nrcth);   
  in.skin.D1_rib= add_ct(in.skin.D1_rib, nrcth);   
  in.skin.D2_rib= add_ct(in.skin.D2_rib, nrcth);   
  in.skin.RP    = add_ct(in.skin.RP, nrcth);   
end

function in = add_ct_ardema(in, nrcth)
  in.tC   = add_ct(in.tC, nrcth);   
  in.tW   = add_ct(in.tW, nrcth);   
  in.dW   = add_ct(in.dW, nrcth);   
  in.tCbar   = add_ct(in.tCbar, nrcth);   
  in.tWbar   = add_ct(in.tWbar, nrcth);   
  in.tgC   = add_ct(in.tgC, nrcth);   
  in.tgW   = add_ct(in.tgW, nrcth);   
end

function in = add_ct_geo(in, nrcth)
  in.Z   = add_ct(in.Z, nrcth); 
  in.Zs   = add_ct(in.Zs, nrcth); 
  in.r   = add_ct(in.r, nrcth); 
  in.rs   = add_ct(in.rs, nrcth); 
  in.incidence   = add_ct(in.incidence, nrcth); 
  in.tbs   = add_ct(in.tbs, nrcth); 
  in.leny = in.leny + nrcth;
  in.box.y = in.y;
  if nrcth == 0
    dycth = norm(in.QC(:,2)-in.QC(:,1));
    in.y = in.y + dycth;
  else
    dycth = norm(in.QC(:,2)-in.QC(:,1)) / nrcth;
    ycth = (0: dycth : nrcth*dycth)';
    in.y = [ycth; ycth(end)+in.y(2:end)];
  end
  in.Swet   = add_ct(in.Swet, nrcth); 
  indexcth = [1; nrcth+1];
  in.index = [indexcth; indexcth(end)+in.index(2:end)-1];
end

function in = add_ct_aero(in, ct_half_len, o1, o2)
  in.QC(2,:) = in.QC(2,:) + ct_half_len;
  QC = in.QC(:,1);
  QC(2) = 0.0;
  in.QC = [QC, in.QC];
  %
  in.C2(2,:) = in.C2(2,:) + ct_half_len;
  C2 = in.C2(:,1);
  C2(2) = 0.0;
  in.C2 = [C2, in.C2];
  %
  in.PANE(2,:) = in.PANE(2,:) + ct_half_len;
  PANE = [in.PANE(:,1), in.PANE(:,1), in.PANE(:,4), in.PANE(:,4)];
  PANE(2,1) = 0.0;
  PANE(2,4) = 0.0;
  in.PANE = [PANE, in.PANE];
  %
  in.WING(2,:) = in.WING(2,:) + ct_half_len;
  in.WING = [PANE, in.WING];
  %
  in.CAERO1.n = [o1; in.CAERO1.n];
  in.CAERO1.n_coarse = [o2; in.CAERO1.n_coarse];
end

function in = add_ct_caero(in, ct_half_len, symmXZ)
  in.CAERO1.dihedral = [0.0; in.CAERO1.dihedral];
  in.CAERO1.chord = [in.CAERO1.chord(1); in.CAERO1.chord];
  in.CAERO1.span = [ct_half_len; in.CAERO1.span];
  in.CAERO1.taper = [in.CAERO1.chord(2)/in.CAERO1.chord(1); in.CAERO1.taper];
  in.CAERO1.sweepQC = [0.0; in.CAERO1.sweepQC];
  in.CAERO1.sweepC2 = [0.0; in.CAERO1.sweepC2];
  in.CAERO1.incidence = [in.CAERO1.incidence(1); in.CAERO1.incidence];
  in.CAERO1.sup_control.frc = [0.0; 0.0; in.CAERO1.sup_control.frc];
  in.CAERO1.sup_control.nme = ['    none'; in.CAERO1.sup_control.nme];
  % AELINK card: set 0
  in.CAERO1.sup_control.typ = [0; in.CAERO1.sup_control.typ];
  in.CAERO1.airfoil = [in.CAERO1.airfoil(1,:); in.CAERO1.airfoil(1,:); in.CAERO1.airfoil];
  if (symmXZ==1)
      in.CAERO1.sup_control.nme = [in.CAERO1.sup_control.nme(1:length(in.CAERO1.n),:);...
          '    none';...
          in.CAERO1.sup_control.nme(length(in.CAERO1.n)+1:end,:)];
  end
end