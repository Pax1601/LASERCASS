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
% 2012-03-21
%
% Main script to performed Generic Unknown Estimator for Structural Sizing
%
% Called by:    guess.m
%
%--------------------------------------------------------------------------------------------------
function [pdcylin, loads, str, aircraft, Res, optim] = AFaWWE_std(fid, pdcylin, aircraft, geo, str, stick, filename_tech, ...
                                                                   ite, Res, optim, Man_index)
global beam_model;
%
%--------------------------------------------------------------------------
% Create structure
%--------------------------------------------------------------------------
%
loads.fus    = [];     % Load for FUSELAGE
loads.wing   = [];     % Load for WING
loads.vtail  = [];     % Load for VERTICAL TAIL
loads.htail  = [];     % Load for HORIZONTAL TAIL
loads.wing2  = [];
loads.canard = [];
%
str.M = 0;      % scalar, to determine the mass computed and written in the *.dat file [kg]
%
%--------------------------------------------------------------------------
% User-input definition of engines attachement in case they are originally
% defined as "floating" propulsion pods.
%--------------------------------------------------------------------------
%
[aircraft] = user_input_engines_attach(fid, aircraft);
%
%--------------------------------------------------------------------------
% LOADS MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- LOADS -------------------------------------------------');
%
[loads, Res] = Load_std(fid, ite, aircraft, pdcylin, geo, str, stick, Res, Man_index);
%
%--------------------------------------------------------------------------
% STRUCTURAL MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- SIZING ------------------------------------------------');
if aircraft.fuselage.present
    fprintf(fid, '\n\t- Fuselage structural sizing...');
    [str] = Str_Fus(pdcylin, aircraft, geo, loads, str);
    geo.fus.x_nodes = stick.nodes.fuse_thick(1,:);
    fprintf(fid, '\n\tdone.');
end
%
if aircraft.wing1.present
    fprintf(fid, '\n\t- Wing structural sizing...');
    [str, optim] = Str_Wing(ite, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end
%
if aircraft.wing2.present
    fprintf(fid, '\n\t- Wing2 structural sizing...');
    [str, optim] = Str_Wing2(ite, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end
%
if  aircraft.Vertical_tail.present
    fprintf(fid, '\n\t- Vertical tail structural sizing...');
    [str, optim] = Str_Vtail(ite, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end
%
if aircraft.Horizontal_tail.present
    fprintf(fid, '\n\t- Horizontal tail structural sizing...');
    [str, optim] = Str_Htail(ite, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end

if aircraft.Canard.present
    fprintf(fid, '\n\tCanard structural sizing...');
    [str, optim] = Str_Canr(ite, pdcylin, aircraft, geo, loads, str, optim);
    fprintf(fid, '\n\tdone.');
end
%
%--------------------------------------------------------------------------
% REGRESSION ANALYSIS MODULUS
%--------------------------------------------------------------------------
%
fprintf(fid,'\n\t-------------------------------------------- REGRESSION --------------------------------------------');
if aircraft.fuselage.present
    fprintf(fid, '\n\t- Fuselage regression equation, ');
    [str] = Regr_Fus(fid, pdcylin, aircraft, geo, loads, str);
end
%
if aircraft.wing1.present
    fprintf(fid, '\n\t- Wing regression equation, ');
    [str] = Regr_Wing(fid, pdcylin, aircraft, geo, loads, str);
end
%
if aircraft.Vertical_tail.present
    fprintf(fid, '\n\t- Vertical tail regression equation, ');
    [str] = Regr_Vtail(fid, pdcylin, aircraft, geo, loads, str);
end
%
if aircraft.Horizontal_tail.present
    fprintf(fid, '\n\t- Horizontal tail regression equation, ');
    [str] = Regr_Htail(fid, pdcylin, aircraft, geo, loads, str);
end
%
if aircraft.Canard.present
    fprintf(fid, '\n\t- Canard tail regression equation, ');
    [str] = Regr_Canard(fid, pdcylin, aircraft, geo, loads, str);
end
%
% Update STR struct
%
%--------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.winr, 1) && isequal(pdcylin.stick.model.fuse, 1)
  nrcth = pdcylin.stick.nwing_carryth;
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
end
%--------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.horr, 1) && geo.htail.twc > 0.0
  nrcth = pdcylin.stick.nhtail_carryth;
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
end
%--------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.canr, 1) && geo.canard.twc > 0.0
  nrcth = pdcylin.stick.ncanard_carryth;
  if pdcylin.canard.kcon <=6
    str.canard = add_ct_ardema(str.canard, nrcth);
  else
    switch pdcylin.canard.kcon
      case {9}
        pdcylin.optimization_smonoq = 0;
         str.canard = add_ct_9(str.canard, nrcth);
      case {10}
        pdcylin.optimization_smonoq = 0;
         str.canard = add_ct_10(str.canard, nrcth);
    end
  end
end

end

function res = add_ct(in, nct)
  v = in(1);
  res = [v.*ones(nct,1); in];
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