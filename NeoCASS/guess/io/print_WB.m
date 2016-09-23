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
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%                                          Creation
%
%
%
%*******************************************************************************
%
% function   print_WB(fid, str, aircraft, pdcylin, stick, geo, ref)
%
%   DESCRIPTION: Print to video a mass resume after structural sizing
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************
function aircraft = print_WB(fid, str, aircraft, pdcylin, stick, geo, ref, OEW, xcgOEW)

%      tbar_F: [21x1 double]
%      tbar_S: [21x1 double]
%     tbar_SC: [21x1 double]
%     tbar_ST: [21x1 double]
%     tbar_SG: [21x1 double]
%     tbar_SB: [21x1 double]
%      tbar_B: [21x1 double]
%     tbar_Af: [21x1 double]
%           d: [21x1 double]
%          WI: [21x1 double]
%          WS: [21x1 double]
%          WP: [21x1 double]
%        WTOT: [21x1 double]
%          CG: 33.3401
%           m: [21x1 double]
%          I1: [21x1 double]
%          I2: [21x1 double]
%           J: [21x1 double]
%          K1: [21x1 double]
%          K2: [21x1 double]
%         NSM: [1x1 struct]
fprintf(fid,'\n\t-------------------------------------------- SUMMARY -----------------------------------------------');

% Fuselage
fprintf(fid,'\n\t------------- Fuselage [Kg] -------------------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tIdeal structural mass  %6.2f', sum(str.fus.WI));
%fprintf(fid,'\n\tStructural mass        %6.2f', sum(str.fus.WS));
%fprintf(fid,'\n\tPrimary structure mass %6.2f', sum(str.fus.WP));
fprintf(fid,'\n\tTotal structure mass   %6.2f', sum(str.fus.WTOT));
fprintf(fid,'\n\t');

% Semi-wingbox
fprintf(fid,'\n\t------------- Semi-wingbox [Kg] ---------------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tBending material mass  %6.2f', sum(str.wing.WBEND));
fprintf(fid,'\n\tShear material mass    %6.2f', sum(str.wing.WSHEAR));
if isfield(str.wing, 'WTORQUE') % True if using kcon = 9, otherwise skip
    fprintf(fid,'\n\tTorsion material mass    %6.2f', sum(str.wing.WTORQUE));
end
fprintf(fid,'\n\tPredicted wingbox mass %6.2f', sum(str.wing.WBEND)+sum(str.wing.WSHEAR));
fprintf(fid,'\n\tActual wingbox mass    %6.2f', sum(str.wing.WBOX));
fprintf(fid,'\n\t');

% Carrythrough
fprintf(fid,'\n\t------------- Wing Carrythrough [Kg] ----------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tBending material mass   %6.2f', str.wing.WBENDC);
fprintf(fid,'\n\tShear material mass     %6.2f', str.wing.WSHEARC);
fprintf(fid,'\n\tTorsion material mass   %6.2f', str.wing.WTORSIONC);
fprintf(fid,'\n\tCarryThrough mass       %6.2f', str.wing.WC);
fprintf(fid,'\n\tFinal CarryThrough mass %6.2f', str.wing.WTC);
fprintf(fid,'\n\t');

% Wingbox
fprintf(fid,'\n\t------------- Wing [Kg] -----------------------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tIdeal structural mass         %6.2f', 2*sum(str.wing.WBOX));
fprintf(fid,'\n\tStructural mass               %6.2f', 2*sum(str.wing.WSBOX));
fprintf(fid,'\n\tPrimary structure mass        %6.2f', 2*sum(str.wing.WPBOX));
fprintf(fid,'\n\tTotal structure mass          %6.2f', 2*sum(str.wing.WTBOX));
fprintf(fid,'\n\tTotal structure including CT  %6.2f', 2*sum(str.wing.WTBOX)+str.wing.WTC);
fprintf(fid,'\n\t');

if aircraft.Canard.present
    
    % canard
    fprintf(fid,'\n\t------------- Canard [Kg] ---------------------------');
    fprintf(fid,'\n\t');
    fprintf(fid,'\n\tIdeal structural mass             %6.2f', 2*sum(str.canard.WBOX));
    fprintf(fid,'\n\tStructural mass                   %6.2f', 2*sum(str.canard.WSBOX));
    fprintf(fid,'\n\tPrimary structure mass            %6.2f', 2*sum(str.canard.WPBOX));
    fprintf(fid,'\n\tTotal structure mass              %6.2f', 2*sum(str.canard.WTBOX));
    fprintf(fid,'\n\tTotal structure mass including CT %6.2f',2*sum(str.canard.WTBOX)+str.canard.WTC);
    fprintf(fid,'\n\t');
    
end

if aircraft.Vertical_tail.present
    
    fprintf(fid,'\n\t------------- Vertical tail [Kg] --------------------');
    fprintf(fid,'\n\t');
    fprintf(fid,'\n\tIdeal structural mass             %6.2f', sum(str.vtail.WBOX));
    fprintf(fid,'\n\tStructural mass                   %6.2f', sum(str.vtail.WSBOX));
    fprintf(fid,'\n\tPrimary structure mass            %6.2f', sum(str.vtail.WPBOX));
    fprintf(fid,'\n\tTotal structure mass              %6.2f', sum(str.vtail.WTBOX));
    fprintf(fid,'\n\tTotal structure mass including CT %6.2f',sum(str.vtail.WTBOX)+str.vtail.WTC);
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
      fprintf(fid,'\n\tWeight referred to one fin.');
    end
    fprintf(fid,'\n\t');
    
end

if aircraft.Horizontal_tail.present
    
    fprintf(fid,'\n\t------------- Horizontal tail [Kg] ------------------');
    fprintf(fid,'\n\t');
    fprintf(fid,'\n\tIdeal structural mass              %6.2f', 2*sum(str.htail.WBOX));
    fprintf(fid,'\n\tStructural mass                    %6.2f', 2*sum(str.htail.WSBOX));
    fprintf(fid,'\n\tPrimary structure mass             %6.2f', 2*sum(str.htail.WPBOX));
    fprintf(fid,'\n\tTotal structure mass               %6.2f', 2*sum(str.htail.WTBOX));
    fprintf(fid,'\n\tTotal structure mass including  CT %6.2f', 2*sum(str.htail.WTBOX)+str.htail.WTC);
    fprintf(fid,'\n\t');
    
end

if aircraft.Tailbooms.present
    
    fprintf(fid,'\n\t------------- Tailbooms [Kg] ------------------------');
    fprintf(fid,'\n\t');
    if (aircraft.Tailbooms.symmetry)
      fprintf(fid,'\n\tTotal structure mass   %6.2f', 2*sum(str.tbooms.WTOT));
    else
      fprintf(fid,'\n\tTotal structure mass   %6.2f', sum(str.tbooms.WTOT));
    end
    fprintf(fid,'\n\t');
    
end
%
fprintf(fid,'\n\t------------- Item Weights [Kg] ---------------------');
fprintf(fid,'\n\t');
% airframe weight
%TOTW = sum(aircraft.weight_balance.COG([1,2,3,4,5,10,11,12],4,1));
%
fprintf(fid,'\n\tFuselage                           %6.2f', aircraft.weight_balance.COG(5,4,1));
fprintf(fid,'\n\tWing                               %6.2f', aircraft.weight_balance.COG(1,4,1));
if aircraft.Horizontal_tail.present
  fprintf(fid,'\n\tHorizontal tail                    %6.2f', aircraft.weight_balance.COG(3,4,1));
end
if aircraft.Vertical_tail.present
  if isequal(aircraft.Vertical_tail.Twin_tail, 1)
    fprintf(fid,'\n\tVertical tail                      %6.2f', 2*aircraft.weight_balance.COG(4,4,1));
  else
    fprintf(fid,'\n\tVertical tail                      %6.2f', aircraft.weight_balance.COG(4,4,1));
  end
end
if aircraft.Tailbooms.present
  fprintf(fid,'\n\tTailbooms                          %6.2f', aircraft.weight_balance.COG(12,4,1));
end
if aircraft.Canard.present
  fprintf(fid,'\n\tCanard                             %6.2f', aircraft.weight_balance.COG(11,4,1));
end
fprintf(fid,'\n\tInterior                           %6.2f', aircraft.weight_balance.COG(21,4,1));
fprintf(fid,'\n\tSystems                            %6.2f', aircraft.weight_balance.COG(17,4,1));
fprintf(fid,'\n\tNose landing gear                  %6.2f', aircraft.weight_balance.COG(9,4,1));
fprintf(fid,'\n\tMain landing gear                  %6.2f', aircraft.weight_balance.COG(6,4,1));
fprintf(fid,'\n\tEngines1                           %6.2f', aircraft.weight_balance.COG(7,4,1));
fprintf(fid,'\n\tEngines2                           %6.2f', aircraft.weight_balance.COG(8,4,1));
fprintf(fid,'\n\tPilots                             %6.2f', aircraft.weight_balance.COG(22,4,1));
fprintf(fid,'\n\tCrew                               %6.2f', aircraft.weight_balance.COG(23,4,1));
fprintf(fid,'\n\n\tPassengers                         %6.2f', aircraft.weight_balance.COG(24,4,1));
fprintf(fid,'\n\tBaggage                            %6.2f', aircraft.weight_balance.COG(25,4,1));
fprintf(fid,'\n\tCentral tank                       %6.2f', aircraft.weight_balance.COG(19,4,1));
%     TOTW +(TSYSWEI - ALNDGWEI + mantwei + OPSIWEI)  +  interior                            + AUXLG + MLG                                + CREW ...+ ENGINE
%OEW = TOTW + aircraft.weight_balance.COG(17,4,1)      + aircraft.weight_balance.COG(21,4,1) + aircraft.weight_balance.COG(9,4,1) + aircraft.weight_balance.COG(6,4,1) + ...
%      (aircraft.weight_balance.COG(22,4,1) + aircraft.weight_balance.COG(23,4,1)) + ...
%      (aircraft.weight_balance.COG(7,4,1) + aircraft.weight_balance.COG(8,4,1));
%ioew = [1 2 3 4 5 10 11 12 17 21 9 6 22 23 7 8];
%xcgOEW = dot(aircraft.weight_balance.COG(ioew,1,1), aircraft.weight_balance.COG(ioew,4,1)) / OEW; 
% OEW + PAX
MZFW = OEW + (aircraft.weight_balance.COG(24,4,1) + aircraft.weight_balance.COG(25,4,1));
%-------------------------------------------------------------------------------------------------------------------------
% UPDATE CG for PAX
%aircraft.weight_balance.COG(24,1:3,1) = pax_CG(aircraft, pdcylin, stick, geo, aircraft.weight_balance.COG(24,4,1), ...
%                                          geo.fus.lengthN, geo.fus.lengthN+aircraft.cabin.Cabin_length_to_aft_cab);
% UPDATE CG for BAGGAGE
%aircraft.weight_balance.COG(25,1:3,1) = pax_CG(aircraft, pdcylin, stick, geo, aircraft.weight_balance.COG(25,4,1), ...
%                                          geo.fus.bodl*aircraft.Baggage.Baggage_apex_per_fuselgt, ...
%                                          geo.fus.bodl*aircraft.Baggage.Baggage_apex_per_fuselgt + aircraft.Baggage.Baggage_combined_length);
imzfw = [24 25];
xcgMZFW = (OEW*xcgOEW + dot(aircraft.weight_balance.COG(imzfw,1,1), aircraft.weight_balance.COG(imzfw,4,1))) / MZFW; 
%-------------------------------------------------------------------------------------------------------------------------
% UPDATE CG for FUEL
aircraft.weight_balance.COG(18,1:3,1) = fuel_CG(fid, aircraft, pdcylin, stick, geo);
aircraft.weight_balance.COG(19,1:3,1) = fuel_ctank_CG(fid, aircraft, pdcylin, stick, geo);
%-------------------------------------------------------------------------------------------------------------------------
fprintf(fid,'\n\tAux. tank                          %6.2f', aircraft.weight_balance.COG(20,4,1));
MTOW = MZFW + aircraft.weight_balance.COG(18,4,1) + aircraft.weight_balance.COG(19,4,1) + aircraft.weight_balance.COG(20,4,1);
imtow = [18 19 20];
xcgMTOW = (MZFW*xcgMZFW + dot(aircraft.weight_balance.COG(imtow,1,1), aircraft.weight_balance.COG(imtow,4,1)))/ MTOW; 
%
% MAC is computed on actual aerodynamic mesh, uncomment the following
% command to switch to use *.xml geometry
% [MAC, ~, XLEMAC, ~] = MAC_wing(aircraft.wing1, aircraft.fuselage.Total_fuselage_length);
%

%CG_FUSE = fuseCG(fid, aircraft, pdcylin, stick, geo, [0; geo.fus.bodl]);
%CG_CREW = fuseCG(fid, aircraft, pdcylin, stick, geo, [geo.fus.lengthN; aircraft.cabin.Cabin_length_to_aft_cab]);
%CG_PIL =  fuseCG(fid, aircraft, pdcylin, stick, geo, [0; geo.fus.lengthN]);
% do not update this item. mass is correctly otimized to result in CG WB value
%aircraft.weight_balance.COG(21,1,1) = CG_FUSE; aircraft.weight_balance.COG(17,1,1) = CG_FUSE;
%aircraft.weight_balance.COG(22,1,1) = CG_PIL;
%aircraft.weight_balance.COG(23,1,1) = CG_CREW;
%
fprintf(fid,'\n\n\t------------- Item CG [m] from nose -----------------');
fprintf(fid,'\n\t');
%
fprintf(fid,'\n\tFuselage                           %6.2f', aircraft.weight_balance.COG(5,1,1));
fprintf(fid,'\n\tWing                               %6.2f', aircraft.weight_balance.COG(1,1,1));
if aircraft.Horizontal_tail.present
  fprintf(fid,'\n\tHorizontal tail                    %6.2f', aircraft.weight_balance.COG(3,1,1));
end
if aircraft.Vertical_tail.present
  fprintf(fid,'\n\tVertical tail                      %6.2f', aircraft.weight_balance.COG(4,1,1));
end
if aircraft.Tailbooms.present
  fprintf(fid,'\n\tTailbooms                          %6.2f', aircraft.weight_balance.COG(12,1,1));
end
if aircraft.Canard.present
  fprintf(fid,'\n\tCanard                             %6.2f', aircraft.weight_balance.COG(11,1,1));
end
fprintf(fid,'\n\tInterior                           %6.2f', aircraft.weight_balance.COG(21,1,1));
fprintf(fid,'\n\tSystems                            %6.2f', aircraft.weight_balance.COG(17,1,1));
fprintf(fid,'\n\tNose landing gear                  %6.2f', aircraft.weight_balance.COG(9,1,1));
fprintf(fid,'\n\tMain landing gear                  %6.2f', aircraft.weight_balance.COG(6,1,1));
fprintf(fid,'\n\tEngines1                           %6.2f', aircraft.weight_balance.COG(7,1,1));
fprintf(fid,'\n\tEngines2                           %6.2f', aircraft.weight_balance.COG(8,1,1));
fprintf(fid,'\n\tPilots                             %6.2f', aircraft.weight_balance.COG(22,1,1));
fprintf(fid,'\n\tCrew                               %6.2f\n', aircraft.weight_balance.COG(23,1,1));
%
fprintf(fid,'\n\tWing tank                          %6.2f', aircraft.weight_balance.COG(18,1,1));
fprintf(fid,'\n\tCentral tank                       %6.2f', aircraft.weight_balance.COG(19,1,1));
fprintf(fid,'\n\tAux. tank                          %6.2f', aircraft.weight_balance.COG(20,1,1));
fprintf(fid,'\n\tPassengers                         %6.2f', aircraft.weight_balance.COG(24,1,1));
fprintf(fid,'\n\tBaggage                            %6.2f', aircraft.weight_balance.COG(25,1,1));
%
MAC    = ref.C_mac;
XLEMAC = ref.XLE_C_mac; 
%
fprintf(fid,'\n\n\t------------- Aircraft Weights [Kg] -----------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tOperative Empty Weight  (OEW)  %6.2f', OEW);
fprintf(fid,'\n\tMax Zero Fuel Weight    (MZFW) %6.2f', MZFW);
fprintf(fid,'\n\tMaximum Take Off Weight (MTOW) %6.2f', MTOW);
fprintf(fid,'\n\t');
%
fprintf(fid,'\n\t------------- Aircraft Balance [m] from nose --------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tLongitudinal Operative Empty Weight  CG %6.2f', xcgOEW);
fprintf(fid,'\n\tLongitudinal Max Zero Fuel Weight    CG %6.2f', xcgMZFW);
fprintf(fid,'\n\tLongitudinal Maximum Take Off Weight CG %6.2f', xcgMTOW);
fprintf(fid,'\n\t');
%
fprintf(fid,'\n\t------------- Aircraft MAC [m] ----------------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tWing mean aerodynamic chord MAC  %6.2f', MAC);
fprintf(fid,'\n\tWing mean aerodynamic chord apex %6.2f', XLEMAC);
fprintf(fid,'\n\t');
%
fprintf(fid,'\n\t------------- Aircraft Balance wrt MAC --------------');
fprintf(fid,'\n\t');
fprintf(fid,'\n\tLongitudinal Operative Empty Weight  CG at MAC %6.2f%%', 100*(xcgOEW-XLEMAC)/MAC);
fprintf(fid,'\n\tLongitudinal Max Zero Fuel Weight    CG at MAC %6.2f%%', 100*(xcgMZFW-XLEMAC)/MAC);
fprintf(fid,'\n\tLongitudinal Maximum Take Off Weight CG at MAC %6.2f%%', 100*(xcgMTOW-XLEMAC)/MAC);
%
fprintf(fid,'\n');
%
end
%--------------------------------------------------------------
function PAXCG = pax_CG(aircraft, pdcylin, stick, geo, MASS, ap1, ap2)
%
PAXCG = zeros(3,1);
if MASS>0
    if isequal(pdcylin.stick.model.fuse, 1)
    mthick = 0.5*(stick.nodes.fuse_thick(1,1:end-1) + stick.nodes.fuse_thick(1,2:end));
    dV = geo.fus.V;
    ind1 = find(mthick>=ap1);
    ind1 = ind1(1);
    ind2 = find(mthick<=ap2);
    ind2 = ind2(end);
    mass_pax_thick = zeros(length(stick.fus.Lbeam_thick),1);
    mass_pax_thick(ind1:ind2) = (MASS) .*(stick.fus.Lbeam_thick(ind1:ind2)./sum(stick.fus.Lbeam_thick(ind1:ind2)));
    NODES = stick.nodes.fuse;
    mass_reg1 = zeros(length(stick.fus.Lbeam),1);
    for i=1:length(stick.fus.Lbeam)
        ind1 = find(mthick>=stick.nodes.fuse(1,i));
        ind1 = ind1(1);
        ind2 = find(mthick<=stick.nodes.fuse(1,i+1));
        ind2 = ind2(end);
        mass_reg1(i) = sum(mass_pax_thick(ind1:ind2));
    end
    % nodal mass
    mass_reg2 = zeros(length(stick.PID.fuse)+1, 1);

    mass_reg2(1) = mass_reg1(1) / 2;

    for i = 2 : length(stick.PID.fuse)
      mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
    end
    mass_reg2(end) = mass_reg1(end) / 2;
    MTOTPAX = sum(mass_reg2);
    for i = 1:length(stick.PID.fuse)+1
      if mass_reg2(i)>0
        PAXCG = PAXCG + (NODES(:,i)* mass_reg2(i));
      end
    end
    PAXCG = PAXCG ./MTOTPAX;
  end
end
end
%
function FUELCG = fuel_CG(fid, aircraft, pdcylin, stick, geo)
FUELCG = zeros(3,1); 
if aircraft.weight_balance.COG(18,4,1)>0
%
  if isequal(pdcylin.stick.model.winr, 1)
    bfuel  = aircraft.fuel.Outboard_fuel_tank_span * geo.wing.b * 0.5;
    afuel  = geo.fus.R;
    dV    = geo.wing.V;
    mthick = 0.5*(stick.nodes.winrC2_thick(2,1:end-1) + stick.nodes.winrC2_thick(2,2:end));
    ind1 = find(mthick>=afuel);
    ind1 = ind1(1);
    ind2 = find(mthick<=bfuel);
    if isempty(ind2)
      fprintf(1, '\n\t### Warning: fuel tank span too short. No fuel loaded in wings.');
    else
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
        ind2 = find(mthick<stick.nodes.winrC2(2,i+1));
        ind2 = ind2(end);
        mass_reg1(i) = sum(mass_fuel_thick(ind1:ind2));
      end
      % nodal mass
      mass_reg2 = zeros(length(stick.PID.wing)+1, 1);
      mass_reg2(1) = mass_reg1(1) / 2;
      for i = 2 : length(stick.PID.wing)
        mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
      end
      mass_reg2(end) = mass_reg1(end) / 2;
      MTOTFU = sum(mass_reg2);
      if isequal(stick.model.symmXZ, 1)
          MTOTFU = 2*MTOTFU;
      end
      fprintf(fid, '\n\tWing tank                          %6.2f', MTOTFU);
      fprintf(fid, ['\n\tFuel wing span fraction from ', num2str(afuel/geo.wing.b*2*100), ' to ', num2str(aircraft.fuel.Outboard_fuel_tank_span*100),' %%']);
      ROT = stick.CBAR.winr.R;
      ROT(:,:,end+1) = stick.CBAR.winr.R(:,:,end);
      CHORD = stick.PBAR.wing.str_rec_coef.C(2,:);
      CHORD(:,end+1) = stick.PBAR.wing.str_rec_coef.C(2,end);
      offset = zeros(3,1);
      for i = 1:length(stick.PID.wing)+1
        if mass_reg2(i) > 0.0
          FUELCG = FUELCG + (NODESR(:,i) + offset) * mass_reg2(i);
        end
      end
      % symmetry
      if isequal(stick.model.symmXZ, 1)
        for i = 1:length(stick.PID.wing)+1
          if mass_reg2(i) > 0.0
            FUELCG = FUELCG + (NODESL(:,i) + offset) * mass_reg2(i);
          end
        end
      end
      FUELCG = FUELCG ./ MTOTFU;
      end
    end
  end
end

function FUELCG = fuel_ctank_CG(fid, aircraft, pdcylin, stick, geo)

FUELCG = zeros(3,1);
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
    % nodal mass
    mass_reg2 = zeros(length(stick.PID.wing)+1, 1);
    mass_reg2(1) = mass_reg1(1) / 2;
    for i = 2 : length(stick.PID.wing)
      mass_reg2(i) = (mass_reg1(i-1)+mass_reg1(i)) / 2;
    end
    mass_reg2(end) = mass_reg1(end) / 2;
    MTOTCFU = sum(mass_reg2);
    if isequal(stick.model.symmXZ, 1)
      MTOTCFU = 2*MTOTCFU;
    end
    offset = zeros(3,1);
    for i = 1:length(stick.PID.wing)+1
      if mass_reg2(i) > 0.0
        FUELCG = FUELCG + (NODESR(:,i) + offset) * mass_reg2(i);
      end
    end
%   symmetry
    if isequal(stick.model.symmXZ, 1)
      for i = 1:length(stick.PID.wing)+1
        if mass_reg2(i) > 0.0
          FUELCG = FUELCG + (NODESL(:,i) + offset) * mass_reg2(i);
        end
      end
    end
  end
FUELCG = FUELCG ./MTOTCFU;
end
end
%
function CG = fuseCG(fid, aircraft, pdcylin, stick, geo, domain)
CG = 0;
nodes = geo.fus.x_nodes;
% Interpolation over beam elements
dV    = spline(geo.fus.x_nodes_1_2_thick, geo.fus.V, geo.fus.x_nodes_1_2);
V = dV;
% Paint is already considered inside Interior but will be accounted for
% separately for each aircraft's component
mass_int = massNSM_distr(nodes, domain, 1, dV, V, stick.fus.Lbeam);
mass_int = mass_int.*stick.fus.Lbeam;
arm = [0; cumsum(stick.fus.Lbeam)];
CG = sum(meancouple(arm).* mass_int);

end