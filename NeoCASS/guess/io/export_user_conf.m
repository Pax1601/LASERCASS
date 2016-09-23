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
% Given output GUESS file, this function creates a mass configuration without 
% re-running the sizing module.
% Output: 
%        output ASCII filename
% Inputs: 
%        WFUEL_PERC: wing fuel tank percentage
%        FUEL_M: wing fuel mass (leave 0 to use XML value)
%        WFUEL_SPAN: 2x1 array with inboard and outboard span fractions
%
%        CFUEL_M: central fuel mass (leave 0 to use XML value)
%        CFUEL_PERC: central tank percentage
%
%        PASS_M: passengers mass (leave 0 to use XML value)
%        PASS_PERC:  passengers percentage (NPAXCONFx1)
%        PASS_SPAN: NPAXCONFx2
%        The first value gives the offset from seats beginning 
%        (summed to geo.fus.lengthN)
%        The second value gives the percentage of cabin length used 
%        (% of aircraft.cabin.Cabin_length_to_aft_cab)
%
%        BAGG_M: baggage mass (leave 0 to use XML value)
%        BAGG_PERC:  baggage percentage
%        BAGG_SPAN: 
%        The first value gives the offset from baggage compartment beginning 
%        (summed to geo.fus.bodl*aircraft.Baggage.Baggage_apex_per_fuselgt)
%        The second value gives the percentage of compartment used 
%        (% of aircraft.Baggage.Baggage_combined_length)
%        geo.fus.bodl is the same  as aircraft.fuselage.Total_fuselage_length
%        geo.fus.lengthN is the same  as aircraft.fuselage.Nose_length
% Example 1:
%
% export_user_conf('test.inc', 'test_guess.mat', 0, 0.5, [0.1 0.75], 
%                   2000, 1, 
%                   9492, 1, [0 1],
%                   3000, [0.73 0.27], [0 1/18; 15 1/18])
% will load test_guess.mat file produced by GUESS and export the configuration in test.inc file 
% to be included in SMARTCAD main file.
%
% The default value for WING FUEL will be used (0 given) and 50% of that value will be loaded (0.5) 
% and stored in wing tank between 10% and 75% of the span (0.1 0.75).
%
% The CENTRAL TANK will load 2000 Kg and used all of them (1).
%
% PASSENGERS mass will be set to 9492 kg and all (1) of them will be placed along the designed
% compartment aisle ([0 1], no offset and all the fuselage aisle used)
%
% Two configurations for BAGGAGE (payload) will be set:
% The total value to load is 3000 Kg.
% 1) 73% of 3000 will be stored between the origin of the compartment (0=no offset) along a 
% length of 1 meter (in this case the aircraft.Baggage.Baggage_combined_length is 18)
% 2) the remainin 27% will be stored from the origin of the comparment + 15 meters for a length of 1 meter as above
%
% The same can ba applied to pax loading.
%
% Example 2:
%
% export_user_conf('test.inc', 'test_guess.mat', 0, 0.5, [0.1 0.75], 
%                   2000, 0.3, 
%                   0, 0.5, [11 0.5],
%                   3000, [0.73 0.27], [0 1/18; 15 1/18])
% will load test_guess.mat file produced by GUESS and export the configuration in test.inc file 
% to be included in SMARTCAD main file.
%
% The default value for WING FUEL will be used (0 given) and 50% of that value will be loaded (0.5) 
% and stored in wing tank between 10% and 75% of the span (0.1 0.75).
%
% The CENTRAL TANK will be set to 2000 Kg and 30% of this value will be loaded (0.3).
%
% PASSENGERS mass will be set to the default value (0) and half (1) of them will be placed along the remainig half of the designed
% compartment aisle.
% For this aircraft, the aisle starts at geo.fus.lengthN = 3.0 and has a length of aircraft.cabin.Cabin_length_to_aft_cab = 25.
% Thus the half aisle has an offset of 11 m and uses half of the available aisle (goes from 14 to 28 m).
%
% Two configurations for BAGGAGE (payload) will be set:
% The total value to load is 3000 Kg.
% 1) 73% of 3000 will be stored between the origin of the compartment (0=no offset) along a 
% length of 1 meter (in this case the aircraft.Baggage.Baggage_combined_length is 18)
% 2) the remainin 27% will be stored from the origin of the comparment + 15 meters for a length of 1 meter as above
%
%
function export_user_conf(output, guess_file, FUEL_M,  WFUEL_PERC, WFUEL_SPAN, ...
                                               CFUEL_M, CFUEL_PERC, ...
                                               PASS_M,  PASS_PERC,  PASS_SPAN, ...
                                               BAGG_M,  BAGG_PERC,  BAGG_SPAN)

fid = 1;

if exist(guess_file,'file')
% 
  load(guess_file); 
  geo   = guess_model.geo;
  stick = guess_model.stick_model;
  str   = guess_model.str_prop;
  aircraft = guess_model.aircraft;
  pdcylin  = guess_model.pdcylin;
%
  WFUEL_SPAN = sort(WFUEL_SPAN);
  NPASSCONF = length(PASS_PERC);
  NBAGGCONF = length(BAGG_PERC);
%
% SUMMARY
%
  fprintf(fid,'\n\n\t------------- XML data  ----------------------------');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tWing tank fuel weight [Kg]:       %g.', aircraft.weight_balance.COG(18,4,1));
  fprintf(fid,'\n\tWing tank inboard span fraction:  %g%%.', pdcylin.MassConf.WfuelStart*100);
  fprintf(fid,'\n\tWing tank outboard span fraction: %g%%.', pdcylin.MassConf.WfuelArrive*100);
%
  fprintf(fid,'\n\n\tPassengers weight [Kg]:           %g.', aircraft.weight_balance.COG(24,4,1));
  fprintf(fid,'\n\tCabin initial position [m]:       %g.', geo.fus.lengthN);
  fprintf(fid,'\n\tCabin length [m]:                 %g.', aircraft.cabin.Cabin_length_to_aft_cab);
%
  fprintf(fid,'\n\n\tBaggage weight [Kg]:              %g.', aircraft.weight_balance.COG(25,4,1));
  fprintf(fid,'\n\tBaggage apex [m]:                 %g.', geo.fus.bodl*aircraft.Baggage.Baggage_apex_per_fuselgt);
  fprintf(fid,'\n\tBaggage length [m]:               %g.', aircraft.Baggage.Baggage_combined_length);
%
  fprintf(fid,'\n\n\tCentral tank fuel weight [Kg]:    %g.', aircraft.weight_balance.COG(19,4,1));
  fprintf(fid,'\n\n\tAux. tank fuel weight [Kg]:       %g.', aircraft.weight_balance.COG(20,4,1));
%
% OVERWRITE WITH USER DEFINED VALUES
%
  if (FUEL_M)
    aircraft.weight_balance.COG(18,4,1) = FUEL_M;
  end
  if (CFUEL_M)
    aircraft.weight_balance.COG(19,4,1) = CFUEL_M;
  end
  if (PASS_M)
    aircraft.weight_balance.COG(24,4,1) = PASS_M;
  end
  if (BAGG_M)
    aircraft.weight_balance.COG(25,4,1) = BAGG_M;
  end
%
  fprintf(fid,'\n\n\t------------- User configuration  ------------------');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tWing tank percentage:             %g%%.', WFUEL_PERC*100);
  fprintf(fid,'\n\tWing tank inboard span fraction:  %g%%.', WFUEL_SPAN(1)*100);
  fprintf(fid,'\n\tWing tank outboard span fraction: %g%%.', WFUEL_SPAN(2)*100);
%
% SET PASSENGERS LOAD CONFS
%
  PASS_M = zeros(NPASSCONF,1);
  for i=1:NPASSCONF
    fprintf(fid,'\n\n\tPassengers percentage:            %g%%.', PASS_PERC(i)*100);
    PASS_SPAN(i,1) = geo.fus.lengthN + PASS_SPAN(i,1);
    fprintf(fid,'\n\tCabin initial position [m]:       %g.', PASS_SPAN(i,1));
    fprintf(fid,'\n\tCabin length used[m]:             %g.', PASS_SPAN(i,2) * aircraft.cabin.Cabin_length_to_aft_cab );
    PASS_SPAN(i,2) = PASS_SPAN(i,2) * aircraft.cabin.Cabin_length_to_aft_cab + PASS_SPAN(i,1);
    PASS_M(i) = aircraft.weight_balance.COG(24,4,1) * PASS_PERC(i);
  end
%
% SET BAGGAGE LOAD CONFS
%
  BAGG_M = zeros(NPASSCONF,1);
  for i=1:NBAGGCONF
    fprintf(fid,'\n\n\tBaggage percentage:               %g%%.', BAGG_PERC(i)*100);
    BAGG_SPAN(i,1) = geo.fus.bodl*aircraft.Baggage.Baggage_apex_per_fuselgt + BAGG_SPAN(i,1);
    fprintf(fid,'\n\tBaggage apex [m]:                 %g.', BAGG_SPAN(i,1));
    fprintf(fid,'\n\tBaggage length used [m]:          %g.', BAGG_SPAN(i,2) * aircraft.Baggage.Baggage_combined_length);
    BAGG_SPAN(i,2) = BAGG_SPAN(i,2) * aircraft.Baggage.Baggage_combined_length + BAGG_SPAN(i,1);
    BAGG_M(i) = aircraft.weight_balance.COG(25,4,1) * BAGG_PERC(i);
  end
%
  fprintf(fid,'\n\n\tCentral tank percentage:          %g%%.', CFUEL_PERC*100);
%
  OEW =  guess_model.stick_model.OEW;
  OEWCG = guess_model.stick_model.OEW_CG;
%
  aircraft.weight_balance.COG(18,4,1) = aircraft.weight_balance.COG(18,4,1) * WFUEL_PERC;
  aircraft.weight_balance.COG(19,4,1) = aircraft.weight_balance.COG(19,4,1) * CFUEL_PERC;
  aircraft.weight_balance.COG(24,4,1) = sum(PASS_M);
  aircraft.weight_balance.COG(25,4,1) = sum(BAGG_M);
%
% USER DEFINED WEIGHTS
%
  fprintf(fid,'\n\n\tWing tank fuel weight:            %g.', aircraft.weight_balance.COG(18,4,1));
  fprintf(fid,'\n\tCentral tank fuel weight:         %g.', aircraft.weight_balance.COG(19,4,1));
  fprintf(fid,'\n\tAux. tank fuel weight:            %g.', aircraft.weight_balance.COG(20,4,1));
  fprintf(fid,'\n\tPassengers weight:                %g.', aircraft.weight_balance.COG(24,4,1));
  fprintf(fid,'\n\tBaggage weight:                   %g.', aircraft.weight_balance.COG(25,4,1));
%
% WING TANK
% 
  pdcylin.MassConf.WfuelStart  = WFUEL_SPAN(1);
  pdcylin.MassConf.WfuelArrive = WFUEL_SPAN(2);
  pdcylin.MassConf.Wfuel       = WFUEL_PERC;
%
  fp = fopen(output, 'w');
  value = aircraft.weight_balance.COG(18,4,1);
  fprintf(fp, ['\n$ Wing Fuel: ',         num2str(WFUEL_PERC  *100),'%% (', num2str(value),' Kg)']);
  value = aircraft.weight_balance.COG(19,4,1);
  fprintf(fp, ['\n$ Central Fuel: ',      num2str(CFUEL_PERC  *100),'%% (', num2str(value),' Kg)']);
  value = aircraft.weight_balance.COG(24,4,1);
  fprintf(fp, ['\n$ Passengers: ',        num2str(PASS_PERC   *100),'%% (', num2str(value),' Kg)']);
  value = aircraft.weight_balance.COG(25,4,1);
  fprintf(fp, ['\n$ Baggage: ',           num2str(BAGG_PERC*100),'%% (', num2str(value),' Kg)']);
%
%  str = Add_NSM(fid, pdcylin, aircraft, geo, stick, str);
  str = Add_NSM_CONF(fp, pdcylin, aircraft, geo, stick, str, PASS_M, PASS_SPAN, BAGG_M, BAGG_SPAN);
% update CG
  aircraft.weight_balance.COG(18,1:3,1) = str.fuel.wing.CG;
  aircraft.weight_balance.COG(19,1:3,1) = str.fuel.ctank.CG;
  aircraft.weight_balance.COG(24,1:3,1) = str.pax.fus.CG;
  aircraft.weight_balance.COG(25,1:3,1) = str.bagg.fus.CG;
%
  fprintf(fid,'\n\n\t------------- Item CG [m] from nose -----------------');
  fprintf(fid,'\n\t');
%
  fprintf(fid,'\n\tWing tank:                        %g.', aircraft.weight_balance.COG(18,1,1));
  fprintf(fid,'\n\tCentral tank:                     %g.', aircraft.weight_balance.COG(19,1,1));
  fprintf(fid,'\n\tAux. tank:                        %g.', aircraft.weight_balance.COG(20,1,1));
  fprintf(fid,'\n\tPassengers:                       %g.', aircraft.weight_balance.COG(24,1,1));
  fprintf(fid,'\n\tBaggage:                          %g.', aircraft.weight_balance.COG(25,1,1));
%
  MAC    = stick.ref.C_mac;
  XLEMAC = stick.ref.XLE_C_mac; 
%
  item = [18 19 20 24 25];
  nitem = length(item);
  WEI = OEW + sum(aircraft.weight_balance.COG(item,4,1));
  CG = (OEW .* OEWCG + sum(repmat(aircraft.weight_balance.COG(item,4,1),1,3).* aircraft.weight_balance.COG(item,1:3,1),1))/WEI;
  fprintf(fid,'\n\n\t------------- Aircraft Weight [Kg] -----------------');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tFinal weight:                     %g.\n', WEI);
  fprintf(fid,'\n\t------------- Aircraft Balance [m] from nose --------');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tLongitudinal CG:                  %g.\n', CG(1));
  fprintf(fid,'\n\t------------- Aircraft Mean Aerodynamic Chord [m] ---');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tChord:                            %g.', MAC);
  fprintf(fid,'\n\tApex:                             %g.', XLEMAC);
  fprintf(fid,'\n\t');
%
  fprintf(fid,'\n\t------------- Aircraft Balance wrt MAC --------------');
  fprintf(fid,'\n\t');
  fprintf(fid,'\n\tCG at MAC %6.2f%%', 100*(CG(1)-XLEMAC)/MAC);
%
  fprintf(fid,'\ndone.\n');
%
% find suport NODE
%
NODES = [stick.nodes.fuse';stick.nodes.winr';stick.nodes.winl'];
IDNODES = [stick.ID.fuse',stick.ID.winr',stick.ID.winl'];
[dummyCG,idSuport] = min((CG(1)-NODES(:,1)).^2+...
    (CG(2)-NODES(:,2)).^2+...
    (CG(3)-NODES(:,3)).^2);
fprintf(fp, '\n$ SUPORT FOR CURRENT CONFIGURATION\n');
BULKdataSUPORT(fp,IDNODES(idSuport));
fprintf(fp, '$\n');
%
fclose(fp);
%
else
  error(['File ', guess_file,' does not exist. Please check.']);
end
%
%
end
%
function str = Add_NSM_CONF(fid, pdcylin, aircraft, geo, stick, str, PASS_M, PASS_SPAN, BAGG_M, BAGG_SPAN)
%
PASSNCONF = length(PASS_M);
BAGGNCONF = length(BAGG_M);
%
MTOTBG  = 0;
MTOTFU  = 0;
MTOTCFU = 0;
MTOTPAX = 0;
% Counter for lumped masses
IDconm = 0;

%**********************************************************************************************************************
% Auxiliary fuel tank
%**********************************************************************************************************************
% Fuel mass in auxiliary tanks
M = aircraft.weight_balance.COG(20,4,1);
MTOTAUX = M;
% Set coordinates
Xcg = aircraft.weight_balance.COG(20,1,1);
Ycg = aircraft.weight_balance.COG(20,2,1);
Zcg = aircraft.weight_balance.COG(20,3,1);
% Second moments of inertia
I11 = 0; I21 = 0; I22 = 0;
I31 = 0; I32 = 0; I33 = 0;

if M > 0.0 && Xcg > 0.0
    % Update counter
    IDconm = IDconm+1;
    % Set CONM2 parameters
    [IDclc, X1offset, X2offset, X3offset] = CONM2_setup(Xcg, Ycg, Zcg, ALLndsXZ, ALLIDsXZ);
    % CONM2 card
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
    fprintf(fid, ['\n$ Lumped masses: AUXILIARY TANK ',num2str(MTOTAUX), ' Kg']);
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    BULKdataCONM2(fid, IDconm, IDclc, 0, M, X1offset, X2offset, X3offset, I11, I21, I22, I31, I32, I33);
    
    % Save in the total mass
    str.M = str.M + aircraft.weight_balance.COG(20,4,1);
    
end
%**********************************************************************************************************************
% BAGGAGE
%**********************************************************************************************************************
CG = zeros(3,1);
TOT = 1;
str.bagg.fus.CG  = zeros(1,3);
if isequal(pdcylin.stick.model.fuse, 1)
  for nconf=1:BAGGNCONF
    if BAGG_M(nconf)> 0
      mthick = 0.5*(stick.nodes.fuse_thick(1,1:end-1) + stick.nodes.fuse_thick(1,2:end));
      ind1 = find(mthick>=BAGG_SPAN(nconf,1));
      ind1 = ind1(1);
      ind2 = find(mthick<=BAGG_SPAN(nconf,2));
      ind2 = ind2(end);
      GEOCG = mean(BAGG_SPAN(nconf,:));
      NODES = stick.nodes.fuse;
      mass_bagg_thick = zeros(length(stick.fus.Lbeam_thick),1);
      mass_bagg_thick(ind1:ind2) = (BAGG_M(nconf)) .*(stick.fus.Lbeam_thick(ind1:ind2)./sum(stick.fus.Lbeam_thick(ind1:ind2)));
      mass_reg1 = zeros(length(stick.fus.Lbeam),1);
      for i=1:length(stick.fus.Lbeam)
        ind1 = find(mthick>=stick.nodes.fuse(1,i));
        ind1 = ind1(1);
        ind2 = find(mthick<=stick.nodes.fuse(1,i+1));
        ind2 = ind2(end);
        mass_reg1(i) = sum(mass_bagg_thick(ind1:ind2));
      end
%      Jt_regX = (0.25 .* mass_reg1) .*...
%      (stick.PBAR.fuse.str_rec_coef.D2.^2 + stick.PBAR.fuse.str_rec_coef.C1.^2);
%      Jt_regY = 1/12 .* mass_reg1 .* stick.fus.Lbeam.^2;
%      Jt_regZ = 1/12 .* mass_reg1 .* stick.fus.Lbeam.^2;
      % nodal mass
      mass_reg2 = zeros(length(stick.PID.fuse)+1, 1);
%      Jt_reg2 = mass_reg2;
%      Iglobal = zeros(3,3,length(stick.PID.fuse)+1);

      mass_reg2(1) = mass_reg1(1) / 2;
%      Jt_reg2X(1) = Jt_regX(1) / 2 ;
%      Jt_reg2Y(1) = Jt_regY(1) / 2 ;
%      Jt_reg2Z(1) = Jt_regZ(1) / 2 ;
%      Iglobal(:,:,1) = stick.CBAR.fuse.R(:,:,1) * [ Jt_reg2X(1) 0 0; 0 Jt_reg2Y(1) 0; 0 0 Jt_reg2Z(1)] * stick.CBAR.fuse.R(:,:,1)';

      for i = 2 : length(stick.PID.fuse)
        mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
 %       Jt_reg2X(i) = (Jt_regX(i-1)+Jt_regX(i)) / 2;
 %       Jt_reg2Y(i) = (Jt_regY(i-1)+Jt_regY(i)) / 2;
 %       Jt_reg2Z(i) = (Jt_regZ(i-1)+Jt_regZ(i)) / 2;
 %       Iglobal(:,:,i) = (stick.CBAR.fuse.R(:,:,i-1) * [ Jt_regX(i-1) 0 0; 0 Jt_regY(i-1) 0; 0 0 Jt_regZ(i-1)] * stick.CBAR.fuse.R(:,:,i-1)' + ...
 %         stick.CBAR.fuse.R(:,:,i) * [ Jt_regX(i) 0 0; 0 Jt_regY(i) 0; 0 0 Jt_regZ(i)] * stick.CBAR.fuse.R(:,:,i)') * 0.5;
      end
      mass_reg2(end) = mass_reg1(end) / 2;
 %     Jt_reg2X(end) = Jt_regX(end) / 2 ;
 %     Jt_reg2Y(end) = Jt_regY(end) / 2 ;
 %     Jt_reg2Z(end) = Jt_regZ(end) / 2 ;
 %     Iglobal(:,:,end) = stick.CBAR.fuse.R(:,:,end)' * [ Jt_reg2X(end) 0 0; 0 Jt_reg2Y(end) 0; 0 0 Jt_reg2Z(end)] * stick.CBAR.fuse.R(:,:,end);
      mass_reg2 = massCONC_opt(stick.nodes.fuse(1,:), BAGG_SPAN(nconf,:), BAGG_M(nconf), GEOCG, mass_reg2');
      MTOTBG = sum(mass_reg2);
      TOT = TOT + MTOTBG;
      fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
      fprintf(fid, ['\n$ Lumped masses: BAGGAGE ',num2str(MTOTBG), ' Kg']);
      ind1 = find(mass_reg1);
      fprintf(fid, ['\n$ Fuselage beams: from ', num2str(ind1(1)), ' to ', num2str(ind1(end))]);
      fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
      %    CONM2 card
      for i = 1:length(stick.PID.fuse)+1
        if mass_reg2(i)>0
          IDconm = IDconm + 1;
%          BULKdataCONM2(fid, IDconm, stick.ID.fuse(i), 0, mass_reg2(i), 0, 0, 0, Iglobal(1,1,i), Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), Iglobal(3,2,i), Iglobal(3,3,i));
          BULKdataCONM2(fid, IDconm, stick.ID.fuse(i), 0, mass_reg2(i), 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          CG = CG + (NODES(:,i)* mass_reg2(i));
        end
      end
    end
  end
  MTOTBG = TOT;
  str.bagg.fus.CG = CG'./TOT;
end
%**********************************************************************************************************************
% PASSENGERS
%**********************************************************************************************************************
CG = zeros(3,1);
TOT = 1;
str.pax.fus.CG  = zeros(1,3);
if isequal(pdcylin.stick.model.fuse, 1)
  for nconf=1:PASSNCONF
    if PASS_M(nconf)>0
      mthick = 0.5*(stick.nodes.fuse_thick(1,1:end-1) + stick.nodes.fuse_thick(1,2:end));
      ind1 = find(mthick>=PASS_SPAN(nconf,1));
      ind1 = ind1(1);
      ind2 = find(mthick<=PASS_SPAN(nconf,2));
      ind2 = ind2(end);
      GEOCG = mean(PASS_SPAN(nconf,:));
      mass_pax_thick = zeros(length(stick.fus.Lbeam_thick),1);
      mass_pax_thick(ind1:ind2) = (PASS_M(nconf)) .*(stick.fus.Lbeam_thick(ind1:ind2)./sum(stick.fus.Lbeam_thick(ind1:ind2)));
      NODES = stick.nodes.fuse;
      mass_reg1 = zeros(length(stick.fus.Lbeam),1);
      for i=1:length(stick.fus.Lbeam)
        ind1 = find(mthick>=stick.nodes.fuse(1,i));
        ind1 = ind1(1);
        ind2 = find(mthick<=stick.nodes.fuse(1,i+1));
        ind2 = ind2(end);
        mass_reg1(i) = sum(mass_pax_thick(ind1:ind2));
      end
%      Jt_regX = (0.25 .* mass_reg1) .*...
%      (stick.PBAR.fuse.str_rec_coef.D2.^2 + stick.PBAR.fuse.str_rec_coef.C1.^2);
%      Jt_regY = 1/12 .* mass_reg1 .* stick.fus.Lbeam.^2;
%      Jt_regZ = 1/12 .* mass_reg1 .* stick.fus.Lbeam.^2;
      % nodal mass
      mass_reg2 = zeros(length(stick.PID.fuse)+1, 1);
%      Jt_reg2 = mass_reg2;
%      Iglobal = zeros(3,3,length(stick.PID.fuse)+1);

      mass_reg2(1) = mass_reg1(1) / 2;
%      Jt_reg2X(1) = Jt_regX(1) / 2 ;
%      Jt_reg2Y(1) = Jt_regY(1) / 2 ;
%      Jt_reg2Z(1) = Jt_regZ(1) / 2 ;
%      Iglobal(:,:,1) = stick.CBAR.fuse.R(:,:,1) * [ Jt_reg2X(1) 0 0; 0 Jt_reg2Y(1) 0; 0 0 Jt_reg2Z(1)] * stick.CBAR.fuse.R(:,:,1)';

      for i = 2 : length(stick.PID.fuse)
        mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
%        Jt_reg2X(i) = (Jt_regX(i-1)+Jt_regX(i)) / 2;
%        Jt_reg2Y(i) = (Jt_regY(i-1)+Jt_regY(i)) / 2;
%        Jt_reg2Z(i) = (Jt_regZ(i-1)+Jt_regZ(i)) / 2;
%        Iglobal(:,:,i) = (stick.CBAR.fuse.R(:,:,i-1) * [ Jt_regX(i-1) 0 0; 0 Jt_regY(i-1) 0; 0 0 Jt_regZ(i-1)] * stick.CBAR.fuse.R(:,:,i-1)' + ...
%          stick.CBAR.fuse.R(:,:,i) * [ Jt_regX(i) 0 0; 0 Jt_regY(i) 0; 0 0 Jt_regZ(i)] * stick.CBAR.fuse.R(:,:,i)') * 0.5;
      end
      mass_reg2(end) = mass_reg1(end) / 2;
%      Jt_reg2X(end) = Jt_regX(end) / 2 ;
%      Jt_reg2Y(end) = Jt_regY(end) / 2 ;
%      Jt_reg2Z(end) = Jt_regZ(end) / 2 ;
%      Iglobal(:,:,end) = stick.CBAR.fuse.R(:,:,end)' * [ Jt_reg2X(end) 0 0; 0 Jt_reg2Y(end) 0; 0 0 Jt_reg2Z(end)] * stick.CBAR.fuse.R(:,:,end);
      mass_reg2 = massCONC_opt(stick.nodes.fuse(1,:), PASS_SPAN(nconf,:), PASS_M(nconf), GEOCG, mass_reg2');
      MTOTPAX = sum(mass_reg2);
      TOT = TOT + MTOTPAX;
      fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
      fprintf(fid, ['\n$ Lumped masses: PASSENGERS ',num2str(MTOTPAX), ' Kg']);
      ind1 = find(mass_reg1);
      fprintf(fid, ['\n$ Fuselage beams: from ', num2str(ind1(1)), ' to ', num2str(ind1(end))]);
      fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
      %    CONM2 card
      for i = 1:length(stick.PID.fuse)+1
        if mass_reg2(i)>0
          IDconm = IDconm + 1;
%          BULKdataCONM2(fid, IDconm, stick.ID.fuse(i), 0, mass_reg2(i), 0, 0, 0, Iglobal(1,1,i), Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), Iglobal(3,2,i), Iglobal(3,3,i));
          BULKdataCONM2(fid, IDconm, stick.ID.fuse(i), 0, mass_reg2(i), 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          CG = CG + (NODES(:,i)* mass_reg2(i));
        end
      end
    end
  end
  str.pax.fus.CG = CG'./TOT;
  MTOTPAX = TOT;
end
%
%**********************************************************************************************************************
% WING TANKS FUEL
%**********************************************************************************************************************
%
FUELCG = zeros(3,1); 
if aircraft.weight_balance.COG(18,4,1)>0
%
  if isequal(pdcylin.stick.model.winr, 1)
    bfuel  = pdcylin.MassConf.WfuelArrive * geo.wing.b*0.5;
    afuel  = pdcylin.MassConf.WfuelStart  * geo.wing.b*0.5;
    dV    = geo.wing.V;
    mthick = 0.5*(stick.nodes.winrC2_thick(2,1:end-1) + stick.nodes.winrC2_thick(2,2:end));
    ind1 = find(mthick>=afuel);
    ind1 = ind1(1);
    ind2 = find(mthick<=bfuel);
    ind2 = ind2(end);
    mass_fuel_thick = zeros(length(stick.wing.Lbeam_thick),1);
    mass_fuel_thick(ind1:ind2) = (aircraft.weight_balance.COG(18,4,1)/2) .*(dV(ind1:ind2)./sum(dV(ind1:ind2)));
      NODESR = stick.nodes.winr;
      if isequal(stick.model.symmXZ, 1)
        NODESL = stick.nodes.winl;
      end
      mass_reg1 = zeros(length(stick.wing.Lbeam),1);
      for i=1:length(stick.wing.Lbeam)
        ind1 = find(mthick>=stick.nodes.winrC2(2,i));
        ind1 = ind1(1);
        ind2 = find(mthick<=stick.nodes.winrC2(2,i+1));
        ind2 = ind2(end);
        mass_reg1(i) = sum(mass_fuel_thick(ind1:ind2));
      end
    
    Jt_regX = mass_reg1'./(stick.PBAR.wing.str_rec_coef.D(1,:).*stick.PBAR.wing.str_rec_coef.C(2,:)*4) .*...
        1/12 .* ( ( (stick.PBAR.wing.str_rec_coef.D(1,:)*2).^3 .* (stick.PBAR.wing.str_rec_coef.C(2,:)*2)) +...
        ((stick.PBAR.wing.str_rec_coef.C(2,:)*2).^3 .* (stick.PBAR.wing.str_rec_coef.D(1,:)*2))  );
    Jt_regY = 1/12 .* mass_reg1 .* stick.wing.Lbeam.^2;
    Jt_regZ = 1/12 .* mass_reg1 .* stick.wing.Lbeam.^2;
    % nodal mass
    mass_reg2 = zeros(length(stick.PID.wing)+1, 1);
    Jt_reg2X = mass_reg2;
    Iglobal = zeros(3,3,length(stick.PID.wing)+1);
    Jt_reg2X(1) = Jt_regX(1) / 2 ;
    Jt_reg2Y(1) = Jt_regY(1) / 2 ;
    Jt_reg2Z(1) = Jt_regZ(1) / 2 ;
    Iglobal(:,:,1) = stick.CBAR.winr.R(:,:,1) * [ Jt_reg2X(1) 0 0; 0 Jt_reg2Y(1) 0; 0 0 Jt_reg2Z(1)] * stick.CBAR.winr.R(:,:,1)';
    mass_reg2(1) = mass_reg1(1) / 2;
    for i = 2 : length(stick.PID.wing)
      mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
      Jt_reg2X(i) = (Jt_regX(i-1)+Jt_regX(i)) / 2;
      Jt_reg2Y(i) = (Jt_regY(i-1)+Jt_regY(i)) / 2;
      Jt_reg2Z(i) = (Jt_regZ(i-1)+Jt_regZ(i)) / 2;
      Iglobal(:,:,i) = (stick.CBAR.winr.R(:,:,i-1) * [ Jt_regX(i-1) 0 0; 0 Jt_regY(i-1) 0; 0 0 Jt_regZ(i-1)] * stick.CBAR.winr.R(:,:,i-1)' + ...
          stick.CBAR.winr.R(:,:,i) * [ Jt_regX(i) 0 0; 0 Jt_regY(i) 0; 0 0 Jt_regZ(i)] * stick.CBAR.winr.R(:,:,i)') * 0.5;
    end
    mass_reg2(end) = mass_reg1(end) / 2;
    Jt_reg2X(end) = Jt_regX(end) / 2 ;
    Jt_reg2Y(end) = Jt_regY(end) / 2 ;
    Jt_reg2Z(end) = Jt_regZ(end) / 2 ;
    Iglobal(:,:,end) = stick.CBAR.winr.R(:,:,end)' * [ Jt_reg2X(end) 0 0; 0 Jt_reg2Y(end) 0; 0 0 Jt_reg2Z(end)] * stick.CBAR.winr.R(:,:,end);
    MTOTFU = sum(mass_reg2);
    if isequal(stick.model.symmXZ, 1)
        MTOTFU = 2*MTOTFU;
    end
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
    fprintf(fid, ['\n$ Lumped masses: WING TANK FUEL ',num2str(MTOTFU), ' Kg']);
    ind1 = find(mass_reg1);
    fprintf(fid, ['\n$ Wing beams: from ', num2str(ind1(1)), ' to ', num2str(ind1(end))]);
    fprintf(fid, ['\n$ Span fraction: from ', num2str(pdcylin.MassConf.WfuelStart*100), ' to ', num2str(pdcylin.MassConf.WfuelArrive*100),' %%']);
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    %
    % CONM2 card
    %
    offset = zeros(3,1);
    for i = 1:length(stick.PID.wing)+1
      if mass_reg2(i) > 0.0
        IDconm = IDconm + 1;
        BULKdataCONM2(fid, IDconm, stick.ID.winr(i), 0, mass_reg2(i), offset(1), offset(2), offset(3), Iglobal(1,1,i), Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), Iglobal(3,2,i), Iglobal(3,3,i));
        FUELCG = FUELCG + (NODESR(:,i) + offset) * mass_reg2(i);
      end
    end
    % symmetry
    if isequal(stick.model.symmXZ, 1)
      for i = 1:length(stick.PID.wing)+1
        if mass_reg2(i) > 0.0
          IDconm = IDconm + 1;
          BULKdataCONM2(fid, IDconm, stick.ID.winl(i), 0, mass_reg2(i), offset(1), -offset(2), offset(3), Iglobal(1,1,i), -Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), -Iglobal(3,2,i), Iglobal(3,3,i));
          FUELCG = FUELCG + (NODESL(:,i) + offset) * mass_reg2(i);
        end
      end
    end
    FUELCG = FUELCG ./ MTOTFU;
  end
end
str.fuel.wing.CG = FUELCG; 
%**********************************************************************************************************************
% CENTRAL TANK FUEL
%**********************************************************************************************************************
CFTANKCG = zeros(3,1);
if aircraft.weight_balance.COG(19,4,1) > 0
    if isequal(pdcylin.stick.model.winr, 1)
         
      mass_reg1 = zeros(length(stick.PID.wing), 1);
      for i = 1:pdcylin.stick.nwing_carryth
        Lcth = sum(stick.wing.Lbeam(1:pdcylin.stick.nwing_carryth));
        mass_reg1(i) = (aircraft.weight_balance.COG(19,4,1)/2) * stick.wing.Lbeam(i) / Lcth; 
      end
      NODESR = stick.nodes.winr;
      if isequal(stick.model.symmXZ, 1)
        NODESL = stick.nodes.winl;
      end
   Jt_regX = mass_reg1'./(stick.PBAR.wing.str_rec_coef.D(1,:).*stick.PBAR.wing.str_rec_coef.C(2,:)*4) .*...
        1/12 .* ( ( (stick.PBAR.wing.str_rec_coef.D(1,:)*2).^3 .* (stick.PBAR.wing.str_rec_coef.C(2,:)*2)) +...
        ((stick.PBAR.wing.str_rec_coef.C(2,:)*2).^3 .* (stick.PBAR.wing.str_rec_coef.D(1,:)*2))  );
    Jt_regY = 1/12 .* mass_reg1 .* stick.wing.Lbeam.^2;
    Jt_regZ = 1/12 .* mass_reg1 .* stick.wing.Lbeam.^2;
    % nodal mass
    mass_reg2 = zeros(length(stick.PID.wing)+1, 1);
    Jt_reg2X = mass_reg2;
    Iglobal = zeros(3,3,length(stick.PID.wing)+1);
    Jt_reg2X(1) = Jt_regX(1) / 2 ;
    Jt_reg2Y(1) = Jt_regY(1) / 2 ;
    Jt_reg2Z(1) = Jt_regZ(1) / 2 ;
    Iglobal(:,:,1) = stick.CBAR.winr.R(:,:,1) * [ Jt_reg2X(1) 0 0; 0 Jt_reg2Y(1) 0; 0 0 Jt_reg2Z(1)] * stick.CBAR.winr.R(:,:,1)';
    mass_reg2(1) = mass_reg1(1) / 2;
    for i = 2 : length(stick.PID.wing)
      mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
      Jt_reg2X(i) = (Jt_regX(i-1)+Jt_regX(i)) / 2;
      Jt_reg2Y(i) = (Jt_regY(i-1)+Jt_regY(i)) / 2;
      Jt_reg2Z(i) = (Jt_regZ(i-1)+Jt_regZ(i)) / 2;
      Iglobal(:,:,i) = (stick.CBAR.winr.R(:,:,i-1) * [ Jt_regX(i-1) 0 0; 0 Jt_regY(i-1) 0; 0 0 Jt_regZ(i-1)] * stick.CBAR.winr.R(:,:,i-1)' + ...
          stick.CBAR.winr.R(:,:,i) * [ Jt_regX(i) 0 0; 0 Jt_regY(i) 0; 0 0 Jt_regZ(i)] * stick.CBAR.winr.R(:,:,i)') * 0.5;
    end
    mass_reg2(end) = mass_reg1(end) / 2;
    Jt_reg2X(end) = Jt_regX(end) / 2 ;
    Jt_reg2Y(end) = Jt_regY(end) / 2 ;
    Jt_reg2Z(end) = Jt_regZ(end) / 2 ;
    Iglobal(:,:,end) = stick.CBAR.winr.R(:,:,end)' * [ Jt_reg2X(end) 0 0; 0 Jt_reg2Y(end) 0; 0 0 Jt_reg2Z(end)] * stick.CBAR.winr.R(:,:,end);
    MTOTCFU = sum(mass_reg2);
    if isequal(stick.model.symmXZ, 1)
        MTOTCFU = 2*MTOTCFU;
    end
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
    fprintf(fid, ['\n$ Lumped masses: CENTRAL TANK FUEL ',num2str(MTOTCFU), ' Kg']);
    ind1 = find(mass_reg1);
    fprintf(fid, ['\n$ Wing beams: from ', num2str(ind1(1)), ' to ', num2str(ind1(end))]);
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
    %
    % CONM2 card
    %
    offset = zeros(3,1);
    for i = 1:length(stick.PID.wing)+1
      if mass_reg2(i) > 0.0
        IDconm = IDconm + 1;
        BULKdataCONM2(fid, IDconm, stick.ID.winr(i), 0, mass_reg2(i), offset(1), offset(2), offset(3), Iglobal(1,1,i), Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), Iglobal(3,2,i), Iglobal(3,3,i));
        CFTANKCG = CFTANKCG + (NODESR(:,i) + offset) * mass_reg2(i);
      end
    end
    % symmetry
    if isequal(stick.model.symmXZ, 1)
      for i = 1:length(stick.PID.wing)+1
        if mass_reg2(i) > 0.0
          IDconm = IDconm + 1;
          BULKdataCONM2(fid, IDconm, stick.ID.winl(i), 0, mass_reg2(i), offset(1), -offset(2), offset(3), Iglobal(1,1,i), -Iglobal(2,1,i), Iglobal(2,2,i), Iglobal(3,1,i), -Iglobal(3,2,i), Iglobal(3,3,i));
          CFTANKCG = CFTANKCG + (NODESL(:,i) + offset) * mass_reg2(i);
        end
      end
    end
  end
  CFTANKCG  = CFTANKCG./MTOTCFU;
end
str.fuel.ctank.CG = CFTANKCG; 
%
% Print summary on file
%
fprintf(fid,  '\n$ Summary of secondary masses (as CONM2): ');
fprintf(fid, ['\n$ Wing tank: ', num2str(MTOTFU)]);
fprintf(fid, ['\n$ Central tank: ', num2str(MTOTCFU)]);
fprintf(fid, ['\n$ Auxiliary tank: ', num2str(MTOTAUX)]);
fprintf(fid, ['\n$ Passengers: ', num2str(MTOTPAX)]);
fprintf(fid, ['\n$ Baggage: ', num2str(MTOTBG)]);
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
%
end