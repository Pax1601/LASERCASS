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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna DIAPM
%***********************************************************************************************************************
% Cruise altitude (m) HCRU
% Minimum cruise mach number MCRU
% Max ceiling altitude HMAX
% CLMAX: clean maximum lift coefficient
% CLMAXLAND: all flaps down maximum lift coefficient
% CLALPHAD: clean lift curve slope
% USERSREF: reference surface used for aero database
% FLAPTO: flap deflection for TO configuration (degs)
% FLAPLAND: flap deflection for landing configuration (degs)
% VSINK: sink velocity at landing
% STROKE: sink velocity at landing
% LNDGEFF: landing gear efficiency
% flag for joined wing
% filename_aircraft: main XML file
% filename_trim: output file with TRIM cards
% ManCheck: logical array selecting maneuvers to be performed (length=8)
%
% Output
% Maneuver points ID, Mach and altitude saved in model.EnvPoints by GUI
%
function MAN = EASA_CAS_25(fid, HCRU, MCRU, HMAX,...
                          CLMAX, CLMAXTO, CLMAXLAND, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                          VSINK, STROKE, LNDGEFF, jwflag, swflag, ...
                          filename_aircraft, filename_trim, ManCheck, CHECK_NORM)
%
MAN = [];
%------------------------------------------------------------------------------------------------------------------
%
fprintf(fid, '\n\tLoading main XML file...');
aircraft = neocass_xmlwrapper(filename_aircraft);
pdcylin = struct('user_input', struct, 'experienced_user_input', struct);
if isfield(aircraft,'user_input')
    pdcylin.user_input = aircraft.user_input;
else
    fprintf('\n\tField user_input is not present in structure');
end
if isfield(aircraft,'experienced_user_input')
    pdcylin.experienced_user_input = aircraft.experienced_user_input;
else
    fprintf('\n\tField experienced_user_input is not present in structure');
end
aircraft.Vertical_tail.longitudinal_location = aircraft.Vertical_tail.x;
aircraft.Vertical_tail.vertical_location = aircraft.Vertical_tail.z;

aircraft.Wing1.longitudinal_location = aircraft.Wing1.x;
aircraft.Wing1.vertical_location = aircraft.Wing1.z;

if aircraft.Wing2.present
    aircraft.Wing2.longitudinal_location = aircraft.Wing2.x;
    aircraft.Wing2.vertical_location = aircraft.Wing2.z;
end
winglet = aircraft.Wing1.winglet;
pdcylin = setup_tech_conversion(pdcylin,aircraft);
pdcylin = setup_sma_defaults(pdcylin);
aircraft = setup_geofile_conversion(aircraft); 
aircraft.winglet = winglet;
if aircraft.Canard.present
    pdcylin.stick.model.canr = 1;
end
if ~aircraft.Horizontal_tail.present
    pdcylin.stick.model.horr = 0;
end
if aircraft.Vertical_tail.present
    pdcylin.stick.model.vert = 1;
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
      pdcylin.stick.model.twin = 1;
    else
      pdcylin.stick.model.twin = 0;
    end
else
    pdcylin.stick.model.vert = 0;
    pdcylin.stick.model.twin = 0;
end
fprintf(fid, '\n\tdone.');
geo.fus     = [];     % Geometry for FUSELAGE
geo.wing    = [];     % Geometry for WING
geo.vtail   = [];     % Geometry for VERTICAL TAIL
geo.htail   = [];     % Geometry for HORIZONTAL TAIL
geo.wing2   = [];     % Geometry for WING2
geo.canard  = [];     % Geometry for CANARD
%
%------------------------------------------------------------------------------------------------------------------
% GEOMETRY MODULUS
%------------------------------------------------------------------------------------------------------------------
%
fprintf(fid, '\n\tDetermining geometry parameters...');
[geo] = Geo_Fus(pdcylin, aircraft, geo);
[geo,pdcylin] = Geo_Wing(pdcylin, aircraft, geo);
if (isfield(aircraft,'Tailbooms') && aircraft.Tailbooms.present)
    [geo] = Geo_Tbooms(pdcylin, aircraft, geo);
    pdcylin.stick.model.tboomsr = 1;
else
  pdcylin.stick.model.tboomsr = 0;
end
if aircraft.Vertical_tail.present
    [geo,pdcylin] = Geo_Vtail(pdcylin, aircraft, geo);
end
if aircraft.Horizontal_tail.present   
    [geo,pdcylin] = Geo_Htail(pdcylin, aircraft, geo); 
end
if aircraft.Canard.present
    [geo,pdcylin] = Geo_Canard(pdcylin, aircraft, geo);
end
if aircraft.wing2.present
    [geo,pdcylin] = Geo_Wing2(pdcylin, aircraft, geo);
else
    pdcylin.stick.model.win2r = 0;
end
fprintf(fid, '\n\tdone.');
%------------------------------------------------------------------------------------------------------------------
%
%
aircraft.Joined_wing.present = jwflag;
aircraft.Strut_wing.present = swflag;
%
[stick, geo] = Stick_Model(pdcylin, aircraft, geo);
all_ID    = [stick.ID.fuse; stick.ID.winr; stick.ID.winl; stick.ID.vert; stick.ID.vert2 ;stick.ID.canr; stick.ID.canl; stick.ID.horr; stick.ID.horl;stick.ID.tboomsr; stick.ID.tboomsl];
all_nodes = [stick.nodes.fuse, stick.nodes.winrC2, stick.nodes.winlC2, stick.nodes.vert, stick.nodes.vert2, stick.nodes.canrC2, stick.nodes.canlC2,stick.nodes.horrC2, stick.nodes.horlC2, stick.nodes.tboomsr, stick.nodes.tboomsl];
distance = all_nodes - meshgrid([aircraft.weight_balance.COG(27,1,1),aircraft.weight_balance.COG(27,2,1),aircraft.weight_balance.COG(27,3,1)], 1:size(all_nodes,2) )';
distance = distance(1,:).^2 + distance(2,:).^2 + distance(3,:).^2;
[dummy, indCG] = min(distance);
%
% Reference values
[RHO0, p0, T0, A0, mu0] = ISA_h(0);
REFS = aircraft.wing1.area;
REFB = aircraft.wing1.span;
REFC = REFS/REFB;
%
if ( ((USERSREF<0) && ((CLMAX+CLMAXLAND+CLALPHAD)>-3)) || (USERSREF<=0) )
  fprintf('\n\t ### Warning: no value for reference surface given.');
  fprintf('\n\t     Value set to %g [m^2].', REFS);
  USERSREF = REFS;
end
%
%------------------------------------------------------------------------------------------------------------------
%
% Conversion
LBS2KG  = 0.45;
FT2M    = 0.3049;
KNTS2MS = 0.514444;
KLAND = 1.0; % factor to find land speed given minimum speed at landing conf.
%
MASTER_LABEL = {};
%
AMPOINTS =  []; IDA = [];
AHPOINTS =  [];
BMPOINTS =  []; IDB = [];
BHPOINTS =  [];
CMPOINTS =  []; IDC = [];
CHPOINTS =  [];
DMPOINTS =  []; IDD = [];
DHPOINTS =  [];
FMPOINTS =  []; IDF = [];
FHPOINTS =  [];
SMPOINTS =  []; IDS = [];
SHPOINTS =  [];
LMPOINTS =  []; IDL = [];
LHPOINTS =  [];
EOMPOINTS = []; IDEO = [];
EOHPOINTS =  [];
%
VSINK1 = 10 * FT2M; 
VSINK2 = 6 * FT2M; 
%
if STROKE == 0
  STROKE = 0.6;
end
if LNDGEFF==0
  LNDGEFF = 0.8;
end
%
cont = 0;
AILP = 0;
FLAPP = 0;
ELEVP = 0;
ELEVCP = 0;
RUDDP = 0;
% Wing controls
if (aircraft.wing1.present)
  MINW  = 0.01*aircraft.wing1.span*0.5;
  if (geo.wing.span_outboard > MINW)
    if (aircraft.wing1.aileron.present==1)
      cont = cont+1; MASTER_LABEL{cont} = 'aileronr'; AILP = 1;
    end
  end
  if (geo.wing.span_inboard > MINW)
    if (aircraft.wing1.flap.present==1)
       cont = cont+1; MASTER_LABEL{cont} = 'flap1r'; FLAPP = 1;
    end
  end
  if (geo.wing.span_midboard > MINW)
    if (aircraft.wing1.flap.present==1)
      cont = cont+1; MASTER_LABEL{cont} = 'flap2r'; FLAPP = 1;
    end
  end
end
% Horizontal tail
if (aircraft.Horizontal_tail.present)
  MINHT = 0.01*aircraft.Horizontal_tail.span*0.5;
  if (geo.htail.span_inboard > MINHT)
    if (aircraft.Horizontal_tail.Elevator.present)
      cont = cont+1; MASTER_LABEL{cont} = 'elev1r'; ELEVP = 1;
    end
  elseif(geo.htail.span_outboard > MINHT)
    if (aircraft.Horizontal_tail.Elevator.present)
      cont = cont+1; MASTER_LABEL{cont} = 'elev2r'; ELEVP = 2;
    end
  end
end
% Canard
if (aircraft.Canard.present)
  MINCN = 0.01*aircraft.Canard.span*0.5;
  if (geo.canard.span_inboard > MINCN)
    if (aircraft.Canard.Elevator.present)
      cont = cont+1; MASTER_LABEL{cont} = 'elevC1r'; ELEVCP = 1;
    end
  elseif(geo.canard.span_outboard > MINCN)
    if (aircraft.Canard.Elevator.present)
      cont = cont+1; MASTER_LABEL{cont} = 'elevC2r'; ELEVCP = 1;
    end
  end
end
% VTail controls
if aircraft.Vertical_tail.present
  MINVT = 0.01*aircraft.Vertical_tail.span; 
  if (geo.vtail.span_inboard > MINVT)
    if (aircraft.Vertical_tail.Rudder.present)
      cont = cont+1; MASTER_LABEL{cont} = 'rudder1'; RUDDP = 1;
    end
  elseif(geo.vtail.span_outboard > MINVT)
    if (aircraft.Vertical_tail.Rudder.present)
      cont = cont+1; MASTER_LABEL{cont} = 'rudder2'; RUDDP = 1;
    end
  end
end
%------------------------------------------------------------------------------------------------------------------
% Assess aircraft
M = sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1));
MLB = M/LBS2KG;
npax = aircraft.cabin.Passenger_accomodation;
%
if (CHECK_NORM==1)
  if (aircraft.engines1.Propulsion_type==0)
    if (MLB < 12500)
      fprintf('\n\t ### Warning: MTOW too low for CAS25 jet aircraft (%d pax).');
    else
      if (npax<10)
        fprintf('\n\t ### Warning: too few passengers (%d pax) for CAS 25 jet aircraft.', npax);
      end
    end
  else
    if (MLB < 19000)
      fprintf('\n\t ### Warning: MTOW too low for CAS25 propeller-driven aircraft.');
    else
      if (npax<19)
        fprintf('\n\t ### Warning: too few passengers (%d pax) for CAS25 propeller-driven aircraft.', npax);
      end
    end
  end
end
%
REFSFT2 = REFS/FT2M^2;
%
WS = MLB / REFSFT2;
%------------------------------------------------------------------------------------------------------------------
% Design Cruise Mach number MC
%
[RHOCRU, pCRU, TCRU, ACRU, muCRU] = ISA_h(HCRU);
%
VKCMIN = mach_alt2cas(MCRU, HCRU);
VCMIN = cas2eas(VKCMIN, HCRU);
%
MC = 1.06 * MCRU;
VKC = mach_alt2cas(MC, HCRU);
VC = cas2eas(VKC, HCRU);
%------------------------------------------------------------------------------------------------------------------
% DIVE VD
if (WS>20)
  VDFAC = interp1([20 100], [1.4 1.35], WS, 'linear', 'extrap');
else
  VDFAC = 1.4;
end
VD = VDFAC*VCMIN;
MD = (VD/VC)*(MC/1.25);
% find dive height HD
found = 0;
for h=1:10:20000
  VDCAS = eas2cas(VD, h);
  mn = cas_alt2mach(VDCAS, h);
  if (mn >= MD)
    found = 1;
    break;
  end
end
if (found == 0)
  error('Unable to estimate dive altitude HD.');
end
HD = h;
%
%------------------------------------------------------------------------------------------------------------------
[PLACVC, PLACHC, PLACVD, PLACHD] = placard(100, HCRU, MC, HD, MD);
%------------------------------------------------------------------------------------------------------------------
% Maximum design load factor
Nmax = (2.1 + 24000/(MLB+10000));
if (Nmax<2.5)
  Nmax = 2.5;
  fprintf('\n\t ### Warning: maximum load factor Nmax will be set to %g.', Nmax);
end
if (Nmax>3.8)
  Nmax = 3.8;
  fprintf('\n\t ### Warning: maximum load factor Nmax will be limited to %g.', Nmax);
end

% Minimum design load factor
Nmin = 1;
%------------------------------------------------------------------------------------------------------------------
if (FLAPLAND<0)
  FLAPLAND = 0;
  fprintf('\n\t ### Flap deflection set to null value.');
end
%------------------------------------------------------------------------------------------------------------------
% Flap speed VF
VFLAND = -1;
VFTO = -1;
MFLAND = -1;
MFTO = -1;
VF = -1; 
MF = -1;
%
vset = 0;
if (CLMAXTO<0)
  fprintf('\n\t ### CLMAXTO non provided. VFTO not defined.');
else
  VFTO = sqrt(M * 9.81 / (0.5 * RHO0 * USERSREF * CLMAXTO));
  MFTO = VFTO/A0;
  vset = 1;
end
if (CLMAXLAND<0)
  fprintf('\n\t ### CLMAXLAND non provided. VFLAND not defined.');
else
  VFLAND = sqrt(M * 9.81 * pdcylin.trdata.clan / (0.5 * RHO0 * USERSREF * CLMAXLAND));
  MFLAND = VFLAND/A0;
  vset = 1;
end
if (vset==1) 
  VF = max(1.6*VFTO, 1.8*VFLAND);
  MF = VF/A0;
else
  fprintf('\n\t ### CLMAXTO and CLMAXLAND non provided. VF not defined.');
end
%------------------------------------------------------------------------------------------------------------------
% Maneuvering speed VA
if (CLMAX<0)
  fprintf('\n\t ### CLMAX not provided. VA and VS not defined.');
  VS = -1;
  VA = -1;
  MA = -1;
  MS = -1;
else
  VS = sqrt(M * 9.81 / (0.5 * RHO0 * USERSREF * CLMAX));
  VA = VS * sqrt(Nmax);
  if (VA > VC)
    fprintf('\n\t ### Warning: VA exceeds VC.');
  end
  MA = VA/A0;
  MS = VS/A0;
end
%------------------------------------------------------------------------------------------------------------------
% Rough Air gust VB
% Gust velocity
% Determine load factor ng due to gust at VC and set VB = sqrt(ng) * VS
%
VGUST_VF = 25 * FT2M;
%
VGUST_VC = 56 * FT2M;
VGUST_VC15 = 44 * FT2M;
VGUST_VC50 = 26 * FT2M;
%
VGUST_VB = 56 * FT2M;
VGUST_VB15 = 44 * FT2M;
VGUST_VB50 = 26 * FT2M;
%
VGUST_VD = 0.5 * VGUST_VC;
VGUST_VD15 = 0.5 * VGUST_VC15;
VGUST_VD50 = 0.5 * VGUST_VC50;
%
[RHO15, p15, T15, A15, mu15] = ISA_h(15000 * FT2M);
[RHO50, p50, T50, A50, mu50] = ISA_h(50000 * FT2M);
%
if (CLALPHAD<0)
  fprintf('\n\t ### CLALPHAD non provided. VB not defined.');
  VB = -1;
  MB = -1;
else
  M = sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1));
  mu_g = 2 * M / (REFC*USERSREF*CLALPHAD*RHO0);
  Kg = (0.88 * mu_g) / (5.3 + mu_g);
  %
  VG   = VGUST_VC * FT2M;
  DELTAN = 0.5 * RHO0 * (USERSREF/M) * CLALPHAD * Kg * VG;
  NGW = DELTAN * VC;
  NG = (1 + NGW/9.81);
  VBC = sqrt(NG) * VS;
  %
  KEAS = 1;
  % gust rough speed
  VG   = VGUST_VB * FT2M;
  % solve for VB (nonlinear system)
  if (VA<0)
    VB = VC/3;
  else
    VB = VA;
  end
  toll = 1;
  DELTAN = 0.5 * RHO0 * (USERSREF) * CLALPHAD * Kg * VG;
  K2 = 0.5*RHO0*USERSREF*CLMAX;
  while (toll>1.0e-5)
    NGW = DELTAN * VB;
    RES = 9.81 * M + NGW - K2*VB^2;
    dV = -RES/(DELTAN - 2*K2*VB);
    VB = VB + dV;
    toll = abs(dV);
  end
  VB = max(VB, VBC);
  MB = VB/A0;
  if (VC < VB + 1.32 * KNTS2MS * VGUST_VB)
    fprintf('\n\t ### Warning: margin between VC and VB is too limited (CAS 25.335 2).');
    fprintf('\n\t ###          VC should be raised or VB reduced.');
  end
end
%
%------------------------------------------------------------------------------------------------------------------
fprintf(fid, '\n\n\t SPEEDS (EAS):');
fprintf(fid, '\n\t - Maneuvering          VA:     %g [m/s].', VA);
fprintf(fid, '\n\t - Rough air            VB:     %g [m/s].', VB);
fprintf(fid, '\n\t - Design min cruise    VC:     %g [m/s].', VCMIN);
fprintf(fid, '\n\t - Design cruise        VMO:    %g [m/s].', VC);
fprintf(fid, '\n\t - Design dive          VD:     %g [m/s].', VD);
fprintf(fid, '\n\t - Stall clean conf.    VS:     %g [m/s].', VS);
fprintf(fid, '\n\t - Stall take-off conf. VFTO:   %g [m/s].', VFTO);
fprintf(fid, '\n\t - Stall landing conf.  VFLAND: %g [m/s].', VFLAND);
fprintf(fid, '\n\t      Percentage MTOW used: %g%%', pdcylin.trdata.clan*100);
fprintf(fid, '\n\t - Flap design          VF:     %g [m/s].', VF);
%
fprintf(fid, '\n\n\t MACH:');
fprintf(fid, '\n\t - Design cruise MMO:  %g.', MC);
fprintf(fid, '\n\t - Design dive    MD:  %g.', MD);
%
fprintf(fid, '\n\n\t LOAD FACTORS:');
fprintf(fid, '\n\t - Max nz: %g.', Nmax);
fprintf(fid, '\n\t - Min nz: %g.', -Nmin);
%
fprintf(fid, '\n\n\t REFERENCE DATA used:');
fprintf(fid, '\n\t - Design weight: %g [N]', M*9.81);
fprintf(fid, '\n\t - Reference surface: %g [m^2]', REFS);
fprintf(fid, '\n\t - Reference chord: %g [m]'    , REFC);
fprintf(fid, '\n\t - Reference span: %g [m]'     , REFB);
fprintf(fid, '\n\t - Reference cruise altitude: %g [m]', HCRU);
fprintf(fid, '\n\t - Max ceiling altitude: %g [m]', HMAX);
fprintf(fid, '\n\t - User-defined reference surface for CLMAX, CLMAXLAND, CLALPHAD: %g [m^2]', USERSREF);
%
trim_count = 1;
nm = length(MASTER_LABEL);
%
fprintf(fid, '\n');
if (VA<0)
  fprintf(fid, '\n\t Maneuvers at VA will not be considered.');
end  
if (VB<0)
  fprintf(fid, '\n\t Maneuvers at VB will not be considered.');
end  
if (VF<0)
  fprintf(fid, '\n\t Maneuvers at VF will not be considered.');
end  
if (VS<0)
  fprintf(fid, '\n\t Maneuvers at VS will not be considered.');
end  
%
if (isequal(ManCheck(1),1)) 
  fprintf(fid, '\n\t Maneuvers at maximum positive load factor required.');
end
if (isequal(ManCheck(2),1)) 
  fprintf(fid, '\n\t Maneuvers with pitch-control inputs required.');
end
if (isequal(ManCheck(3),1)) 
  fprintf(fid, '\n\t Maneuvers with yaw-control inputs required.');
end
if (isequal(ManCheck(4),1)) 
  fprintf(fid, '\n\t Maneuvers with roll-control inputs required.');
end
if (isequal(ManCheck(5),1)) 
  fprintf(fid, '\n\t Static gust required.');
end
if (isequal(ManCheck(6),1)) 
  fprintf(fid, '\n\t Taildown landing required.');
end
if (isequal(ManCheck(7),1)) 
  fprintf(fid, '\n\t Engine out required.');
end
if (isequal(ManCheck(8),1)) 
  fprintf(fid, '\n\t Maneuvers with high lift devices required.');
end
%
MAN_INDEX = [];
MAN_H = [];
MAN_M = [];
%
fid2 = fopen(filename_trim, 'w');
fprintf(fid2, '$\n$ SPEEDS (EAS):');
fprintf(fid2, '\n$ - Maneuvering          VA:  %g [m/s].', VA);
fprintf(fid2, '\n$ - Rough air            VB:  %g [m/s].', VB);
fprintf(fid2, '\n$ - Design min cruise    VC:  %g [m/s].', VCMIN);
fprintf(fid2, '\n$ - Design cruise       VMO:  %g [m/s].', VC);
fprintf(fid2, '\n$ - Design dive          VD:  %g [m/s].', VD);
fprintf(fid2, '\n$ - Stall clean conf.    VS:  %g [m/s].', VS);
fprintf(fid2, '\n$ - Stall landing conf. VFLAND: %g [m/s].', VFLAND);
fprintf(fid2, '\n$      Percentage MTOW used: %g%%', pdcylin.trdata.clan*100);
fprintf(fid2, '\n$ - Flap design          VF:  %g [m/s].', VF);
%
fprintf(fid2, '\n$\n$ MACH:');
fprintf(fid2, '\n$ - Design cruise MMO:  %g.', MC);
fprintf(fid2, '\n$ - Design dive    MD:  %g.', MD);
%
fprintf(fid2, '\n$\n$ LOAD FACTORS:');
fprintf(fid2, '\n$ - Max nz: %g.', Nmax);
fprintf(fid2, '\n$ - Min nz: %g.', -Nmin);
%
fprintf(fid2, '\n$\n$ REFERENCE DATA used:');
fprintf(fid2, '\n$ - Design weight: %g [N]', M*9.81);
fprintf(fid2, '\n$ - Reference surface: %g [m^2]', REFS);
fprintf(fid2, '\n$ - Reference chord: %g [m]'    , REFC);
fprintf(fid2, '\n$ - Reference span: %g [m]'     , REFB);
fprintf(fid2, '\n$ - Reference cruise altitude: %g [m]', HCRU);
fprintf(fid2, '\n$ - Max ceiling altitude: %g [m]', HMAX);
fprintf(fid2, '\n$ - User-defined reference surface for CLMAX, CLMAXLAND, CLALPHAD: %g [m^2]\n$', USERSREF);
%
if (isequal(ManCheck(1),1)) 
  %------------------------------------------------------------------------------------------------------------------
  % MAX LOAD FACTOR at VD
  %------------------------------------------------------------------------------------------------------------------
  ACCMAN = 9.81 * Nmax;
  setm = 0;
  switch (ELEVP)
    case{0}
    if (ELEVCP==1)
      LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
    end
    case{1}
      LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
    case{2}
      LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
  end
  if ~setm
    error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
  end
%
  fprintf(fid2,'\nTRIM=   %d', trim_count);
  fprintf(fid2,'\n$ Sec. 25.303');
  fprintf(fid2,'\n$ Maximum load factor at VD, nz = %g, z=0m', Nmax);
  fprintf(fid2,'\n$ VD: %g m/s', PLACVD(1,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVD(1,2),0);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
  DMPOINTS = [DMPOINTS; MN];
  IDD = [IDD; trim_count];
  DHPOINTS = [DHPOINTS; 0.0];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MN, MAN_INDEX, MAN_H, MAN_M);
  %------------------------------------------------------------------------------------------------------------------
  trim_count = trim_count+1;
  fprintf(fid2,'\nTRIM=   %d', trim_count);
  fprintf(fid2,'\n$ Sec. 25.303');
  fprintf(fid2,'\n$ Maximum load factor at VD, nz = %g, z=%gm', Nmax, HD);
  [v, index] = max(PLACVD(:,3)); 
  fprintf(fid2,'\n$ VD: %g m/s', PLACVD(index,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVD(index,2),HD);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '%c', cnvt2_8chs(HD));
  fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
  DMPOINTS = [DMPOINTS; MN];
  IDD = [IDD; trim_count];
  DHPOINTS = [DHPOINTS; HD];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(HD, MN, MAN_INDEX, MAN_H, MAN_M);
  %------------------------------------------------------------------------------------------------------------------
  % MAX LOAD FACTOR at VC
  %------------------------------------------------------------------------------------------------------------------
  trim_count = trim_count+1;
  fprintf(fid2,'\nTRIM=   %d', trim_count);
  fprintf(fid2,'\n$ Sec. 25.303');
  fprintf(fid2,'\n$ Maximum load factor at VC, nz = %g, z=0m', Nmax);
  fprintf(fid2,'\n$ VC: %g m/s', PLACVC(1,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVC(1,2),0);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
  CMPOINTS = [CMPOINTS; MN];
  IDC = [IDC; trim_count];
  CHPOINTS = [CHPOINTS; 0.0];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MN, MAN_INDEX, MAN_H, MAN_M);
  %------------------------------------------------------------------------------------------------------------------
  trim_count = trim_count+1;
  fprintf(fid2,'\nTRIM=   %d', trim_count);
  fprintf(fid2,'\n$ Sec. 25.303');
  fprintf(fid2,'\n$ Maximum load factor at VC, nz = %g, z=%gm', Nmax, HCRU);
  [v, index] = max(PLACVC(:,3)); 
  fprintf(fid2,'\n$ VC: %g m/s', PLACVC(index,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVC(index,2),HCRU);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '%c', cnvt2_8chs(HCRU));
  fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
  CMPOINTS = [CMPOINTS; MN];
  IDC = [IDC; trim_count];
  CHPOINTS = [CHPOINTS; HCRU];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(HCRU, MN, MAN_INDEX, MAN_H, MAN_M);
end
%------------------------------------------------------------------------------------------------------------------
% HIGH LIFT DEVICES
%------------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(8),1) && (VF>0)) 
  if (FLAPP==1) 
    ROTUP = zeros(nm,1);
    LHEAD = 4; SURF{1} = 'flap';
    for k=1:nm
      [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
      if (strfind(LABLIstr,SURF{1})==1)
        ROTUP(k) = FLAPLAND;
      end
    end
    setm = 0;
    switch (ELEVP)
      case{0}
      if (ELEVCP==1)
        LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
      end
      case{1}
        LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
      case{2}
        LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
    end
    if ~setm
      error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
    end
    trim_count = trim_count+1;
    ACCMAN = 2 * 9.81;
    fprintf(fid2,'\nTRIM=   %d', trim_count);
    fprintf(fid2,'\n$ Sec. 25.349 a1');
    fprintf(fid2,'\n$ Cruise at VF = max(1.6 VFTO, 1.8 VFLAND), nz = 2, z=0m');
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    fprintf(fid2, '%c', cnvt2_8chs(MF));
    fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 1);
    FMPOINTS = [FMPOINTS; MF];
    FHPOINTS = [FHPOINTS; 0.0];
    IDF = [IDF; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MF, MAN_INDEX, MAN_H, MAN_M);
    %
    trim_count = trim_count+1;
    ACCMAN = 9.81;
    fprintf(fid2,'\nTRIM=   %d ', trim_count);
    fprintf(fid2,'\n$ Sec. 25.349 a2');
    fprintf(fid2,'\n$ Gust for Landing conf. at VF = max(1.6 VFTO, 1.8 VFLAND), z=0m');
    fprintf(fid2,'\n$  VF: %g m/s', VF);
    fprintf(fid2,'\n$  VFTO: %g m/s', VFTO);
    fprintf(fid2,'\n$ VFLAND: %g m/s', VFLAND);
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    fprintf(fid2, '%c', cnvt2_8chs(MF));
    fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    fprintf(fid2,'VGUST   ');
    fprintf(fid2, '%c', cnvt2_8chs(VGUST_VF));
    fprintf(fid2,'\n        ');
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 0);
    FMPOINTS = [FMPOINTS; MF];
    FHPOINTS = [FHPOINTS; 0.0];
    IDF = [IDF; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MF, MAN_INDEX, MAN_H, MAN_M);
  end
end
%------------------------------------------------------------------------------------------------------------------
% HOR. BALANCING
%------------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(2),1) && (VA>0)) 
  setm = 0;
  switch (ELEVP)
    case{0}
    if (ELEVCP==1)
      LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
      rot_up = abs(aircraft.Canard.Elevator.limit_deflection_up);
      rot_down = abs(aircraft.Canard.Elevator.limit_deflection_down);
      if rot_up<eps
          error('Canard elevator maximum deflection has null value. Please check xml file.');
      end    
      if rot_down<eps
          error('Canard elevator minimum deflection has null value. Please check xml file.');
      end    
    end
    case{1}
      LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
      rot_up = abs(aircraft.Horizontal_tail.Elevator.limit_deflection_down);
      rot_down = abs(aircraft.Horizontal_tail.Elevator.limit_deflection_up);
      if rot_up<eps
          error('Elevator maximum deflection has null value. Please check xml file.');
      end    
      if rot_down<eps
          error('Elevator minimum deflection has null value. Please check xml file.');
      end    
    case{2}
      LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
      rot_up = abs(aircraft.Horizontal_tail.Elevator.limit_deflection_down);
      rot_down = abs(aircraft.Horizontal_tail.Elevator.limit_deflection_up);
      if rot_up<eps
          error('Elevator maximum deflection has null value. Please check xml file.');
      end    
      if rot_down<eps
          error('Elevator minimum deflection has null value. Please check xml file.');
      end    
  end
  if ~setm
    error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
  end
%
  ROTUP = zeros(length(MASTER_LABEL),1);
  ROTDW = zeros(length(MASTER_LABEL),1);
  for k=1:nm
    [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
    if (strfind(LABLIstr,SURF{1})==1)
      ROTUP(k) = rot_up;
      ROTDW(k) = rot_down;
    end
  end
  for k=1:2
    if k==1
      ACCMAN = 9.81;
      ACCANGMAN = (39 * KNTS2MS / VA) * (Nmax-1.5) * Nmax;
    else
      ACCMAN = Nmax * 9.81;
      ACCANGMAN = -(26 * KNTS2MS / VA) * (Nmax-1.5) * Nmax; 
    end
    if (VA>0)
      trim_count = trim_count+1;
      fprintf(fid2,'\nTRIM=   %d', trim_count);
      fprintf(fid2,'\n$ Sec. 25.331 b1');
      fprintf(fid2,'\n$ Sudden aft movement of pitch control at VA=sqrt(nmax) * VS , z=0m');
      fprintf(fid2,'\n$  VA: %g m/s', VA);
      fprintf(fid2,'\n$  VS: %g m/s', VS);
      fprintf(fid2,'\n$ Determine pitch control and angle of attack');
      fprintf(fid2,'\n$ Check control deflection is between %g and %g', -rot_down, rot_up);
      fprintf(fid2,'\nTRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(trim_count));
      fprintf(fid2, '1       ');
      fprintf(fid2, '%c', cnvt2_8chs(MA));
      AHPOINTS = [AHPOINTS; 0.0];
      AMPOINTS = [AMPOINTS; MA];
      IDA = [IDA; trim_count];
      [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
      fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
      fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
      fprintf(fid2,'\n        URDD4   0.0     BANK    0.0     URDD6   0.0     CLIMB   0.0     ');
      fprintf(fid2,'\n        HEAD    0.0     URDD3   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
      fprintf(fid2,'URDD5   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCANGMAN));
      print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
    end
    for i=2:3
      trim_count = trim_count+1;
      fprintf(fid2,'\nTRIM=   %d', trim_count);
      fprintf(fid2,'\n$ Sec. 23.331 b1');
      switch (i)
        case 2
          fprintf(fid2,'\n$ Sudden aft movement of pitch control at VC, z=0m');
          fprintf(fid2,'\n$  VC: %g m/s', PLACVC(1,3));
        case 3
          fprintf(fid2,'\n$ Sudden aft movement of pitch control at VD, z=0m');
          fprintf(fid2,'\n$  VD: %g m/s', PLACVD(1,3));
      end
      fprintf(fid2,'\n$ Determine pitch control and angle of attack');
      fprintf(fid2,'\n$ Check control deflection is between %g and %g', -rot_down, rot_up);
      fprintf(fid2,'\nTRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(trim_count));
      fprintf(fid2, '1       ');
      switch (i)
        case 2
            MN = cas_alt2mach(PLACVC(1,2),0);
            fprintf(fid2, '%c', cnvt2_8chs(MN));
            CHPOINTS = [CHPOINTS; 0.0];
            CMPOINTS = [CMPOINTS; MN];
            IDC = [IDC; trim_count];
        case 3
            MN = cas_alt2mach(PLACVD(1,2),0);
            fprintf(fid2, '%c', cnvt2_8chs(MN));
            DHPOINTS = [DHPOINTS; 0.0];
            DMPOINTS = [DMPOINTS; MN];
            IDD = [IDD; trim_count];
      end
      [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MN, MAN_INDEX, MAN_H, MAN_M);
      fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
      fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
      fprintf(fid2,'\n        URDD4   0.0     BANK    0.0     URDD6   0.0     CLIMB   0.0     ');
      fprintf(fid2,'\n        HEAD    0.0     URDD3   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
      fprintf(fid2,'URDD5   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCANGMAN));
      print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 1);
    end
  end
end
%
%------------------------------------------------------------------------------------------------------------------
% Vertical tail and rudder
%------------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(3),1) && (VS>0)) 
  if (RUDDP==1)
    rot = abs(aircraft.Vertical_tail.Rudder.limit_deflection);
    if rot<eps
      error('Rudder maximum deflection has null value. Please check xml file.')
    end
    ROTUP = zeros(nm,1);
    LHEAD = 6; SURF{1} = 'rudder';
    for k=1:nm
      [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
      if (strfind(LABLIstr,SURF{1})==1)
        ROTUP(k) = rot;
      end
    end
    setm = 0;
    switch (ELEVP)
      case{0}
      if (ELEVCP==1)
        LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
      end
      case{1}
        LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
      case{2}
        LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
    end
    if ~setm
      error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
    end
    ACCMAN = 9.81;
    for ndef=1:2
      if ndef == 2  
        ROTUP = -ROTUP;
      end
      for i=1:2
        trim_count = trim_count+1;
        fprintf(fid2,'\nTRIM=   %d', trim_count);
        fprintf(fid2,'\n$ Sec. 25.349 a');
        if i==1
          fprintf(fid2,'\n$ Sideslip maneuver at max rudder deflection at VA=sqrt(nmax) * VS, z=0m');
          fprintf(fid2,'\n$  VA: %g m/s', VA);
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        else
          fprintf(fid2,'\n$ Sideslip maneuver at max rudder deflection at VS, z=0m');
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        end
        fprintf(fid2,'\n$ Determine pitch control, angle of attack, lateral acceleration, roll and yaw accelerations');
        fprintf(fid2,'\nTRIM    ');
        fprintf(fid2, '%c', cnvt2_8chs(trim_count));
        fprintf(fid2, '0       ');
        if i==1
          fprintf(fid2, '%c', cnvt2_8chs(MA));
          AHPOINTS = [AHPOINTS; 0.0];
          AMPOINTS = [AMPOINTS; MA];
          IDA = [IDA; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
        else
          fprintf(fid2, '%c', cnvt2_8chs(MS));
          SHPOINTS = [SHPOINTS; 0.0];
          SMPOINTS = [SMPOINTS; MS];
          IDS = [IDS; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MS, MAN_INDEX, MAN_H, MAN_M);
        end
        fprintf(fid2, '0.0     HEAD    0.0     ROLL    0.0     ');
        fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     SIDES   0.0     THRUST  0.0     ');
        fprintf(fid2,'\n        BANK    0.0     CLIMB   0.0     URDD5   0.0     URDD3   ');
        fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
        fprintf(fid2,'\n        ');
        print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 0);
      %
        SIDES = 1.5 * 15;
        trim_count = trim_count+1;
        fprintf(fid2,'\nTRIM=   %d', trim_count);
        fprintf(fid2,'\n$ Sec. 25.351');
        if i==1
          fprintf(fid2,'\n$ Sideslip maneuver at overswing sideslip angle %g deg at VA=sqrt(nmax) * VS, z=0m', SIDES);
          fprintf(fid2,'\n$  VA: %g m/s', VA);
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        else
          fprintf(fid2,'\n$ Sideslip maneuver at overswing sideslip angle %g deg at VS, z=0m', SIDES);
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        end
        fprintf(fid2,'\n$ Determine pitch control, angle of attack, lateral acceleration, roll and yaw accelerations');
        fprintf(fid2,'\nTRIM    ');
        fprintf(fid2, '%c', cnvt2_8chs(trim_count));
        fprintf(fid2, '0       ');
        if i==1
          fprintf(fid2, '%c', cnvt2_8chs(MA));
          AHPOINTS = [AHPOINTS; 0.0];
          AMPOINTS = [AMPOINTS; MA];
          IDA = [IDA; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
        else
          fprintf(fid2, '%c', cnvt2_8chs(MS));
          SHPOINTS = [SHPOINTS; 0.0];
          SMPOINTS = [SMPOINTS; MS];
          IDS = [IDS; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MS, MAN_INDEX, MAN_H, MAN_M);
        end
        fprintf(fid2, '0.0     HEAD    0.0     ROLL    0.0     ');
        fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     SIDES   ');
        fprintf(fid2, '%c', cnvt2_8chs(SIDES));
        fprintf(fid2,'THRUST  0.0     ');
        fprintf(fid2,'\n        BANK    0.0     CLIMB   0.0     URDD5   0.0     URDD3   ');
        fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
        fprintf(fid2,'\n        ');
        print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 0);
    %
        SIDES = 15;
        trim_count = trim_count+1;
        fprintf(fid2,'\nTRIM=   %d', trim_count);
        fprintf(fid2,'\n$ Sec. 25.351');
        if i==1
          fprintf(fid2,'\n$ Sideslip maneuver at sideslip angle %g at VA=sqrt(nmax) * VS, z=0m', SIDES);
          fprintf(fid2,'\n$  VA: %g m/s', VA);
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        else
          fprintf(fid2,'\n$ Sideslip maneuver at sideslip angle %g at VS, z=0m', SIDES);
          fprintf(fid2,'\n$  VS: %g m/s', VS);
        end
        fprintf(fid2,'\n$ Determine pitch control, angle of attack, lateral acceleration, roll and yaw accelerations');
        fprintf(fid2,'\nTRIM    ');
        fprintf(fid2, '%c', cnvt2_8chs(trim_count));
        fprintf(fid2, '0       ');
        if i==1
          fprintf(fid2, '%c', cnvt2_8chs(MA));
          AHPOINTS = [AHPOINTS; 0.0];
          AMPOINTS = [AMPOINTS; MA];
          IDA = [IDA; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
        else
          fprintf(fid2, '%c', cnvt2_8chs(MS));
          SHPOINTS = [SHPOINTS; 0.0];
          SMPOINTS = [SMPOINTS; MS];
          IDS = [IDS; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MS, MAN_INDEX, MAN_H, MAN_M);
        end
        fprintf(fid2, '0.0     HEAD    0.0     ROLL    0.0     ');
        fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     SIDES   ');
        fprintf(fid2, '%c', cnvt2_8chs(SIDES));
        fprintf(fid2,'THRUST  0.0     ');
        fprintf(fid2,'\n        BANK    0.0     CLIMB   0.0     URDD5   0.0     URDD3   ');
        fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
        fprintf(fid2,'\n        ');
        print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
      end
    end % ndefl
  end
end
%------------------------------------------------------------------------------------------------------------------
% Ailerons
%------------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(4),1) && (VS>0)) 
  if (AILP==1)
    rot = aircraft.wing1.aileron.limit_deflection_up;
    if rot<eps
      error('Aileron maximum deflection has null value. Please check xml file.');
    end
    ROTUP = zeros(nm,1);
    LHEAD = 7; SURF{1} = 'aileron';
    for k=1:nm
      [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
      if (strfind(LABLIstr,SURF{1})==1)
        ROTUP(k) = rot;
      end
    end
    setm = 0;
    switch (ELEVP)
      case{0}
      if (ELEVCP==1)
        LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
      end
      case{1}
        LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
      case{2}
        LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
    end
    if ~setm
      error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
    end
    ACCMAN = 9.81;
    for ndef = 1:2
      if ndef==2
        ROTUP = -ROTUP;
      end
      trim_count = trim_count+1;
      fprintf(fid2,'\nTRIM=   %d', trim_count);
      fprintf(fid2,'\n$ Sec. 23.349');
      fprintf(fid2,'\n$ Aileron abrupt maximum deflection at VA=sqrt(nmax) * VS, z=0m');
      fprintf(fid2,'\n$  VA: %g m/s', VA);
      fprintf(fid2,'\n$  VS: %g m/s', VS);
      fprintf(fid2,'\n$ Determine pitch control, angle of attack, lateral acceleration, roll and yaw accelerations');
      fprintf(fid2,'\nTRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(trim_count));
      fprintf(fid2, '0       ');
      fprintf(fid2, '%c', cnvt2_8chs(MA));
      fprintf(fid2, '0.0     HEAD    0.0     ROLL    0.0     ');
      fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     SIDES   0.0     THRUST  0.0');
      fprintf(fid2,'\n        BANK    0.0     CLIMB   0.0     URDD5   0.0     URDD3   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
      fprintf(fid2,'\n        ');
      print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 0);
      ACCMAN = 9.81;
      rot = aircraft.wing1.aileron.limit_deflection_up;
      ROTUP = zeros(nm,1);
      LHEAD = 7; SURF{1} = 'aileron';
      for k=1:nm
        [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
        if (strfind(LABLIstr,SURF{1})==1)
          ROTUP(k) = rot;
        end
      end
      setm = 0;
      switch (ELEVP)
        case{0}
        if (ELEVCP==1)
          LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
        end
        case{1}
          LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
        case{2}
          LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
      end
      if ~setm
        error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
      end
      AMPOINTS = [AMPOINTS; MA];
      IDA = [IDA; trim_count];
      AHPOINTS = [AHPOINTS; 0.0];
      [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
      %
      trim_count = trim_count+1;
      ACCMAN = 2/3 * 9.81 * Nmax;
      fprintf(fid2,'\nTRIM=   %d', trim_count);
      fprintf(fid2,'\n$ Sec. 25.349');
      fprintf(fid2,'\n$ Aileron abrupt deflection + 2/3 max load factor at VA=sqrt(nmax) * VS, z=0m', trim_count);
      fprintf(fid2,'\n$  VA: %g m/s', VA);
      fprintf(fid2,'\n$  VS: %g m/s', VS);
      fprintf(fid2,'\n$ Determine pitch control, angle of attack, lateral acceleration, roll and yaw accelerations');
      fprintf(fid2,'\nTRIM    ');
      fprintf(fid2, '%c', cnvt2_8chs(trim_count));
      fprintf(fid2, '0       ');
      fprintf(fid2, '%c', cnvt2_8chs(MA));
      fprintf(fid2, '0.0     HEAD    0.0     ROLL    0.0     ');
      fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     SIDES   0.0     THRUST  0.0');
      fprintf(fid2,'\n        BANK    0.0     CLIMB   0.0     URDD5   0.0     URDD3   ');
      fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
      fprintf(fid2,'\n        ');
      print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 0);
      AMPOINTS = [AMPOINTS; MA];
      AHPOINTS = [AHPOINTS; 0.0];
      IDA = [IDA; trim_count];
      [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MA, MAN_INDEX, MAN_H, MAN_M);
    end
  end
end
%------------------------------------------------------------------------------------------------------------------
% GUST
%------------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(5),1)) 
  ACCMAN = 9.81;  
  setm = 0;
  switch (ELEVP)
    case{0}
    if (ELEVCP==1)
      LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
    end
    case{1}
      LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
    case{2}
      LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
  end
  if ~setm
    error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
  end

  %
  trim_count = trim_count+1;
  fprintf(fid2,'\nTRIM=   %d \n$ Gust at VC, z=0m', trim_count);
  fprintf(fid2,'\n$ Sec. 25.341');
  fprintf(fid2,'\n$ VC: %g m/s', PLACVC(1,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVC(1,2),0);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  fprintf(fid2,'VGUST   ');
  fprintf(fid2, '%c', cnvt2_8chs(VGUST_VC));
  fprintf(fid2,'\n        ');
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
  CMPOINTS = [CMPOINTS; MN];
  CHPOINTS = [CHPOINTS; 0.0];
  IDC = [IDC; trim_count];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MN, MAN_INDEX, MAN_H, MAN_M);
  %
  if (HMAX>15000* FT2M)
    trim_count = trim_count+1;
    fprintf(fid2,'\nTRIM=   %d \n$ Gust at VC at z=15000ft', trim_count);
    fprintf(fid2,'\n$ Sec. 25.341');
    index = find(PLACHC>= 15000 * FT2M);
    fprintf(fid2,'\n$ VC: %g m/s', PLACVC(index(1),3));
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    MN = cas_alt2mach(PLACVC(index(1),2),0);
    fprintf(fid2, '%c', cnvt2_8chs(MN));
    fprintf(fid2, '%c', cnvt2_8chs(15000 * FT2M));
    fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    fprintf(fid2,'VGUST   ');
    fprintf(fid2, '%c', cnvt2_8chs(VGUST_VC15));
    fprintf(fid2,'\n        ');
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
    CMPOINTS = [CMPOINTS; MN];
    CHPOINTS = [CHPOINTS; 15000 * FT2M];
    IDC = [IDC; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(15000 * FT2M, MN, MAN_INDEX, MAN_H, MAN_M);
  end
  %
  if (HMAX>50000* FT2M)
    trim_count = trim_count+1;
    fprintf(fid2,'\nTRIM=   %d \n$ Gust at VC, z=50000ft', trim_count);
    fprintf(fid2,'\n$ Sec. 25.341');
    index = find(PLACHC>= 50000 * FT2M);
    fprintf(fid2,'\n$ VC: %g m/s', PLACVC(index(1),3));
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    MN = cas_alt2mach(PLACVC(index(1),2),0);
    fprintf(fid2, '%c', cnvt2_8chs(MN));
    fprintf(fid2, '%c', cnvt2_8chs(50000 * FT2M));
    fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    fprintf(fid2,'VGUST   ');
    fprintf(fid2, '%c', cnvt2_8chs(VGUST_VC50));
    fprintf(fid2,'\n        ');
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
    CMPOINTS = [CMPOINTS; MN];
    CHPOINTS = [CHPOINTS; 50000 * FT2M];
    IDC = [IDC; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(50000 * FT2M, MN, MAN_INDEX, MAN_H, MAN_M);
  end
  %
  trim_count = trim_count+1;
  fprintf(fid2,'\nTRIM=   %d \n$ Gust at VD, z=0m', trim_count);
  fprintf(fid2,'\n$ Sec. 25.341');
  fprintf(fid2,'\n$ VD: %g m/s', PLACVD(1,3));
  fprintf(fid2,'\n$ Determine pitch control and angle of attack');
  fprintf(fid2,'\nTRIM    ');
  fprintf(fid2, '%c', cnvt2_8chs(trim_count));
  fprintf(fid2, '1       ');
  MN = cas_alt2mach(PLACVD(1,2),0);
  fprintf(fid2, '%c', cnvt2_8chs(MN));
  fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
  fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
  fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
  fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
  fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
  fprintf(fid2,'VGUST   ');
  fprintf(fid2, '%c', cnvt2_8chs(VGUST_VD));
  fprintf(fid2,'\n        ');
  print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
  DMPOINTS = [DMPOINTS; MN];
  DHPOINTS = [DHPOINTS; 0.0];
  IDD = [IDD; trim_count];
  [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MN, MAN_INDEX, MAN_H, MAN_M);
  %
  if (HMAX>15000* FT2M)
    trim_count = trim_count+1;
    fprintf(fid2,'\nTRIM=   %d \n$ Gust at VD at z=15000ft', trim_count);
    fprintf(fid2,'\n$ Sec. 25.341');
    index = find(PLACHD>= 15000 * FT2M);
    fprintf(fid2,'\n$ VD: %g m/s', PLACVD(index(1),3));
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    MN = cas_alt2mach(PLACVD(index(1),2),0);
    fprintf(fid2, '%c', cnvt2_8chs(MN));
    fprintf(fid2, '%c', cnvt2_8chs(15000 * FT2M));
    fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    fprintf(fid2,'VGUST   ');
    fprintf(fid2, '%c', cnvt2_8chs(VGUST_VD15));
    fprintf(fid2,'\n        ');
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
    DMPOINTS = [DMPOINTS; MN];
    DHPOINTS = [DHPOINTS; 15000 * FT2M];
    IDD = [IDD; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(15000 * FT2M, MN, MAN_INDEX, MAN_H, MAN_M);
  end
  %
  if (HMAX>50000* FT2M)
    trim_count = trim_count+1;
    fprintf(fid2,'\nTRIM=   %d \n$ Gust at VD, z=50000ft', trim_count);
    fprintf(fid2,'\n$ Sec. 25.341');
    index = find(PLACHD>= 50000 * FT2M);
    fprintf(fid2,'\n$ VC: %g m/s', PLACVD(index(1),3));
    fprintf(fid2,'\n$ Determine pitch control and angle of attack');
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    MN = cas_alt2mach(PLACVD(index(1),2),0);
    fprintf(fid2, '%c', cnvt2_8chs(MN));
    fprintf(fid2, '%c', cnvt2_8chs(50000 * FT2M));
    fprintf(fid2, 'SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(ACCMAN));
    fprintf(fid2,'VGUST   ');
    fprintf(fid2, '%c', cnvt2_8chs(VGUST_VD50));
    fprintf(fid2,'\n        ');
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, zeros(nm,1), 0);
    DMPOINTS = [DMPOINTS; MN];
    DHPOINTS = [DHPOINTS; 50000 * FT2M];
    IDD = [IDD; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(50000 * FT2M, MN, MAN_INDEX, MAN_H, MAN_M);
  end
end
%-------------------------------------------------------------------------------------------------------------
% TAILDOWN LANDING
%-------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(6),1) && (VFLAND>0)) 
  % Get LG coordinates
  Xcg = aircraft.weight_balance.COG(6,1,1);
  Ycg = abs(aircraft.weight_balance.COG(6,2,1));
  Zcg = aircraft.weight_balance.COG(6,3,1);
  if Xcg > 0.0
    fprintf(fid,'\n\t ### Warning: CAS25 prescribe 10 and 6 fps as landing sink velocities.');
    fprintf(fid,'\n\t     User defined values will not be considered.');
    VLAND = KLAND * VFLAND;
    MLAND = VLAND / A0;
    rot = FLAPLAND;
    ROTUP = zeros(nm,1);
    LHEAD = 4; SURF{1} = 'flap';
    for k=1:nm
      [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
      if (strfind(LABLIstr,SURF{1})==1)
        ROTUP(k) = rot;
      end
    end
    setm = 0;
    switch (ELEVP)
      case{0}
      if (ELEVCP==1)
        LHEAD = 5; SURF{1} = 'elevC'; setm = 1;
      end
      case{1}
        LHEAD = 5; SURF{1} = 'elev1'; setm = 1;
      case{2}
        LHEAD = 5; SURF{1} = 'elev2'; setm = 1;
    end
    if ~setm
      error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
    end
    trim_count = trim_count+1;
    DESCGRAD = atan(VSINK1/VLAND) * pi/180;
    fprintf(fid2,'\nTRIM=   %d', trim_count);
    fprintf(fid2,'\n$ Sec. 25.473 a2');
    fprintf(fid2,'\n$ Tail down landing at VFLAND, z=0m, VSINK = 10 fps');
    fprintf(fid2,'\n$ VFLAND: %g m/s', VFLAND);
    fprintf(fid2,'\n$ Landing configuration');
    fprintf(fid2,'\n$ 1) Determine pitch control and angle of attack');
    fprintf(fid2,'\n$ 2) Introduce ground reaction');
    fprintf(fid2,'\n$ MTOW percentage %g', pdcylin.trdata.clan*100);
    fprintf(fid2,'\n$ Descent gradient: %g degs', DESCGRAD);
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    fprintf(fid2, '%c', cnvt2_8chs(MLAND));
    fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(9.81 * pdcylin.trdata.clan * cos(DESCGRAD)));
    fprintf(fid2,'VSINK   ');  fprintf(fid2, '%c', cnvt2_8chs(VSINK1));
    fprintf(fid2,'\n        STROKE  '); fprintf(fid2, '%c', cnvt2_8chs(STROKE));
    fprintf(fid2,'LNDGEFF '); fprintf(fid2, '%c', cnvt2_8chs(LNDGEFF));
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 2); fprintf(fid2, '\n');
    LMPOINTS = [LMPOINTS; MLAND];
    LHPOINTS = [LHPOINTS; 0.0];
    IDL = [IDL; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MLAND, MAN_INDEX, MAN_H, MAN_M);
%
    trim_count = trim_count+1;
    DESCGRAD = atan(VSINK2/VLAND) * pi/180;
    fprintf(fid2,'\n\nTRIM=   %d', trim_count);
    fprintf(fid2,'\n$ Sec. 25.473 a3');
    fprintf(fid2,'\n$ Tail down landing at VFLAND, z=0m, VSINK = 6 fps, MTOW');
    fprintf(fid2,'\n$ VFLAND: %g m/s', VFLAND);
    fprintf(fid2,'\n$ Landing configuration');
    fprintf(fid2,'\n$ 1) Determine pitch control and angle of attack');
    fprintf(fid2,'\n$ 2) Introduce ground reaction');
    fprintf(fid2,'\n$ MTOW percentage 100');
    fprintf(fid2,'\n$ Descent gradient: %g degs', DESCGRAD);
    fprintf(fid2,'\nTRIM    ');
    fprintf(fid2, '%c', cnvt2_8chs(trim_count));
    fprintf(fid2, '1       ');
    fprintf(fid2, '%c', cnvt2_8chs(MLAND));
    fprintf(fid2, '0.0     SIDES   0.0     ROLL    0.0     ');
    fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   0.0     THRUST  0.0     ');
    fprintf(fid2,'\n        URDD4   0.0     URDD5   0.0     URDD6   0.0     CLIMB   0.0     ');
    fprintf(fid2,'\n        BANK    0.0     HEAD    0.0     URDD3   ');
    fprintf(fid2, '%c', cnvt2_8chs(9.81 * cos(DESCGRAD)));
    fprintf(fid2,'VSINK   ');  fprintf(fid2, '%c', cnvt2_8chs(VSINK2));
    fprintf(fid2,'\n        STROKE  '); fprintf(fid2, '%c', cnvt2_8chs(STROKE));
    fprintf(fid2,'LNDGEFF '); fprintf(fid2, '%c', cnvt2_8chs(LNDGEFF));
    print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 2); fprintf(fid2, '\n');
% Export LG (done in CONTROLGEO)
%    IDclc = 9000; BULKdataGRID(fid2, IDclc, 0, Xcg, Ycg, Zcg, 0, 0, 0);
%    fprintf(fid2, 'PARAM   LANDG   %d\n', IDclc);
%    if (Ycg~=0)
%      IDclc = 9001; BULKdataGRID(fid2, IDclc, 0, Xcg, -Ycg, Zcg, 0, 0, 0);
%      fprintf(fid2, 'PARAM   LANDG   %d\n', IDclc);
%    end
    LMPOINTS = [LMPOINTS; MLAND];
    LHPOINTS = [LHPOINTS; 0.0];
    IDL = [IDL; trim_count];
    [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MLAND, MAN_INDEX, MAN_H, MAN_M);
  end
end
%-------------------------------------------------------------------------------------------------------------
% ENGINE OUT
%-------------------------------------------------------------------------------------------------------------
if (isequal(ManCheck(6),1) && (VFTO>0)) 
  if aircraft.Vertical_tail.present
    if (aircraft.wing1.aileron.present==1 && aircraft.Vertical_tail.Rudder.present==1)
  %
      Ycg1 = 0;
      Ycg2 = 0;
      nengtot = 0;
      if (aircraft.engines1.Number_of_engines ~= 0)
        Xcg1 = aircraft.engines1.Location_engines_nacelles_on_X + aircraft.engines1.nacelle_length/2;
        Ycg1 = aircraft.engines1.Location_engines_nacelles_on_Y;
        Zcg1 = aircraft.engines1.Location_engines_nacelles_on_Z;
        nengtot = nengtot + aircraft.engines1.Number_of_engines;
      end
  %
      if (aircraft.engines2.Number_of_engines ~= 0)
        Xcg2 = aircraft.engines2.Location_engines_nacelles_on_X + aircraft.engines2.nacelle_length/2;
        Ycg2 = aircraft.engines2.Location_engines_nacelles_on_Y;
        Zcg2 = aircraft.engines2.Location_engines_nacelles_on_Z;
        nengtot = nengtot + aircraft.engines2.Number_of_engines;
      end
%
      if (nengtot>0 && FLAPP==1)
%
        export_eo = 0;
        if(Ycg1>0)
          thrust1 = (1000*aircraft.engines1.Max_thrust) .* [-1 0 0]';
          mthrust1 = crossm([Xcg1 Ycg1 Zcg1] - all_nodes(:, indCG)') * thrust1;
          export_eo = 1;
          thrust = thrust1;
          mthrust = mthrust1;  
        elseif(Ycg2>0)
          thrust2 = (1000*aircraft.engines2.Max_thrust) .* [-1 0 0]';
          mthrust2 = crossm([Xcg2 Ycg2 Zcg2] - all_nodes(:, indCG)') * thrust2;
          export_eo = 1;
          thrust = thrust2;
          mthrust = mthrust2;  
        end
        if (export_eo==1)
          trim_count = trim_count+1;
          fprintf(fid2, '\nFORCE   '); 
          fprintf(fid2, '%c', cnvt2_8chs(trim_count));
          fprintf(fid2, '%c', cnvt2_8chs(all_ID(indCG)));
          fprintf(fid2, '%c', cnvt2_8chs(0));
          fprintf(fid2, '%c', cnvt2_8chs(norm(thrust)));
          for k=1:3
            fprintf(fid2, '%c', cnvt2_8chs(thrust(k)/norm(thrust)));
          end
          fprintf(fid2, '\nMOMENT  '); 
          fprintf(fid2, '%c', cnvt2_8chs(trim_count));
          fprintf(fid2, '%c', cnvt2_8chs(all_ID(indCG)) );
          fprintf(fid2, '%c', cnvt2_8chs(0));
          fprintf(fid2, '%c', cnvt2_8chs(norm(mthrust)));
          for k=1:3
            fprintf(fid2, '%c', cnvt2_8chs(mthrust(k)/norm(mthrust)));
          end
%
          ROTUP = zeros(nm,1);
          LHEAD = 4; SURF{1} = 'flap';
          for k=1:nm
            [LABLIstr] = strtok(cnvt2_8chs(MASTER_LABEL{k}));
            if (strfind(LABLIstr,SURF{1})==1)
              ROTUP(k) = max(FLAPTO,0);
            end
          end
          VCLIMB = VFTO;
          BANK_MAX = 0;
          MCLIMB = MFTO;
          switch (nengtot)
            case 2
              CGRAD = 0.012;
            case 3
              CGRAD = 0.015;
            case 4
              CGRAD = 0.017;
          end
%
          GAMMA = atan(CGRAD);
          ACCMAN = 9.81 * cos(GAMMA);
          fprintf(fid2,'\nTRIM=   %d', trim_count);
          fprintf(fid2,'\n$ Sec. 25.121 c');
          fprintf(fid2,'\n$ One engine inoperative climb gradient at VCLIMB = VFTO, %g degs max bank angle', BANK_MAX);
          fprintf(fid2,'\n$ Take-off configuration if flap deflection provided');
          fprintf(fid2,'\n$ VCLIMB: %g m/s', VFTO);
          fprintf(fid2,'\n$ Determine lateral controls and sideslip angle');
          fprintf(fid2,'\nTRIM    ');
          fprintf(fid2, '%c', cnvt2_8chs(trim_count));
          fprintf(fid2, '0       ');
          fprintf(fid2, '%c', cnvt2_8chs(MCLIMB));
          fprintf(fid2, '%c', cnvt2_8chs(0.0 * FT2M));
          EOMPOINTS = [EOMPOINTS; MCLIMB];
          EOHPOINTS = [EOHPOINTS; 0.0 * FT2M];
          IDEO = [IDEO; trim_count];
          [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MCLIMB, MAN_INDEX, MAN_H, MAN_M);
          fprintf(fid2, 'ROLL    0.0     THRUST  0.0     ');
          fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   ');
          fprintf(fid2, '%c', cnvt2_8chs(-9.81 * sin(BANK_MAX*pi/180)));
          fprintf(fid2,'CLIMB   0.0     \n        URDD4   0.0     URDD5   0.0     URDD6   0.0     BANK    0.0     ');
          fprintf(fid2,'\n        HEAD    0.0     URDD3   ');
          fprintf(fid2, '%c', cnvt2_8chs(9.81 * cos(BANK_MAX*pi/180)));
          LHEAD(1) = 6; SURF{1} = 'rudder';
          LHEAD(2) = 7; SURF{2} = 'aileron';
          nhead = length(LHEAD)+1;
          setm = 0;
          switch (ELEVP)
            case{0}
            if (ELEVCP==1)
              LHEAD(nhead) = 5; SURF{nhead} = 'elevC'; setm = 1;
            end
            case{1}
              LHEAD(nhead) = 5; SURF{nhead} = 'elev1'; setm = 1;
            case{2}
              LHEAD(nhead) = 5; SURF{nhead} = 'elev2'; setm = 1;
          end
          if ~setm
            error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
          end
          print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 2);
%
          if (Ycg1>0 && Ycg2 >0)
            trim_count = trim_count+1;
            fprintf(fid2, '\nFORCE   '); 
            fprintf(fid2, '%c', cnvt2_8chs(trim_count));
            fprintf(fid2, '%c', cnvt2_8chs(all_ID(indCG)));
            fprintf(fid2, '%c', cnvt2_8chs(0));
            fe_mag = norm(thrust1)+norm(thrust2);
            fprintf(fid2, '%c', cnvt2_8chs(fe_mag));
            for k=1:3
              fprintf(fid2, '%c', cnvt2_8chs( (thrust1(k)+thrust2(k))/fe_mag ));
            end
            fprintf(fid2, '\nMOMENT  '); 
            fprintf(fid2, '%c', cnvt2_8chs(trim_count));
            fprintf(fid2, '%c', cnvt2_8chs(all_ID(indCG)) );
            fprintf(fid2, '%c', cnvt2_8chs(0));
            me_mag = norm(mthrust1)+norm(mthrust2);
            fprintf(fid2, '%c', cnvt2_8chs(me_mag));
            for k=1:3
              fprintf(fid2, '%c', cnvt2_8chs( (mthrust1(k)+mthrust2(k))/(norm(mthrust1)+norm(mthrust2)) ));
            end
            CGRAD = 0.012;
            GAMMA = atan(CGRAD);
            ACCMAN = 9.81 * cos(GAMMA);
            fprintf(fid2,'\nTRIM=   %d', trim_count);
            fprintf(fid2,'\n$ Sec. 25.123 c');
            fprintf(fid2,'\n$ One engine inoperative climb gradient at VCLIMB = VFTO, %g degs max bank angle', BANK_MAX);
            fprintf(fid2,'\n$ Take-off configuration');
            fprintf(fid2,'\n$ VCLIMB: %g m/s', VFTO);
            fprintf(fid2,'\n$ Determine lateral controls and sideslip angle');
            fprintf(fid2,'\nTRIM    ');
            fprintf(fid2, '%c', cnvt2_8chs(trim_count));
            fprintf(fid2, '0       ');
            fprintf(fid2, '%c', cnvt2_8chs(MCLIMB));
            fprintf(fid2, '%c', cnvt2_8chs(0.0 * FT2M));
            EOMPOINTS = [EOMPOINTS; MCLIMB];
            EOHPOINTS = [EOHPOINTS; 0.0 * FT2M];
            IDEO = [IDEO; trim_count];
            [MAN_INDEX, MAN_H, MAN_M] = set_man_index(0.0, MCLIMB, MAN_INDEX, MAN_H, MAN_M);
            fprintf(fid2, 'ROLL    0.0     THRUST  0.0     ');
            fprintf(fid2,'\n        PITCH   0.0     YAW     0.0     URDD2   ');
            fprintf(fid2, '%c', cnvt2_8chs(-9.81 * sin(BANK_MAX*pi/180)));
            fprintf(fid2,'CLIMB   0.0     \n        URDD4   0.0     URDD5   0.0     URDD6   0.0     BANK    0.0     ');
            fprintf(fid2,'\n        HEAD    0.0     URDD3   ');
            fprintf(fid2, '%c', cnvt2_8chs(9.81 * cos(BANK_MAX*pi/180)));
            LHEAD(1) = 6; SURF{1} = 'rudder';
            LHEAD(2) = 7; SURF{2} = 'aileron';
            nhead = length(LHEAD)+1;
            setm = 0;
            switch (ELEVP)
              case{0}
              if (ELEVCP==1)
                LHEAD(nhead) = 5; SURF{nhead} = 'elevC'; setm = 1;
              end
              case{1}
                LHEAD(nhead) = 5; SURF{nhead} = 'elev1'; setm = 1;
              case{2}
                LHEAD(nhead) = 5; SURF{nhead} = 'elev2'; setm = 1;
            end
            if ~setm
              error('No control surface detected for a consistent longitudinal trim, i.e. canard, htail. Contact DIAPM.')
            end
            print_master_controls(fid2, MASTER_LABEL, LHEAD, SURF, ROTUP, 2);
          end % 2nd engine inoperative
        end
      end
    end
  end
end
%
% Export flight points
%
% Plot flight points
figure(101); close; figure(101); hold on;
label = {};
title('Flight points assessed and maneuver ID');
cont = 0;
if (~isempty(AMPOINTS))
  MAN.VA.M =  AMPOINTS;
  MAN.VA.H =  AHPOINTS;
  MAN.VA.ID = IDA;
  plot_zm_points(MAN.VA.H, MAN.VA.M, MAN.VA.ID, 'co');
  cont = cont+1;
  label{cont} = 'VA';
end
%
if (~isempty(BMPOINTS))
  MAN.VB.M =  BMPOINTS;
  MAN.VB.H =  BHPOINTS;
  MAN.VB.ID = IDB;
  plot_zm_points(MAN.VB.H, MAN.VB.M, MAN.VB.ID, 'bo');
  cont = cont+1;
  label{cont} = 'VB';
end
%
MAN.VC.M =  CMPOINTS;
MAN.VC.H =  CHPOINTS;
MAN.VC.ID = IDC;
plot_zm_points(MAN.VC.H, MAN.VC.M, MAN.VC.ID, 'ko');
cont = cont+1;
label{cont} = 'VC';
%
MAN.VD.M =  DMPOINTS;
MAN.VD.H =  DHPOINTS;
MAN.VD.ID = IDD;
plot_zm_points(MAN.VD.H, MAN.VD.M, MAN.VD.ID, 'ro');
cont = cont+1;
label{cont} = 'VD';
%
if (~isempty(FMPOINTS))
  MAN.VF.M =  FMPOINTS;
  MAN.VF.H =  FHPOINTS;
  MAN.VF.ID = IDF;
  plot_zm_points(MAN.VF.H, MAN.VF.M, MAN.VF.ID, 'rs');
  cont = cont+1;
  label{cont} = 'VF';
end
%
if (~isempty(SMPOINTS))
  MAN.VS.M =  SMPOINTS;
  MAN.VS.H =  SHPOINTS;
  MAN.VS.ID = IDS;
  plot_zm_points(MAN.VS.H, MAN.VS.M, MAN.VS.ID, 'ys');
  cont = cont+1;
  label{cont} = 'VS';
end
%
if (~isempty(LMPOINTS))
  MAN.VL.M =  LMPOINTS;
  MAN.VL.H =  LHPOINTS;
  MAN.VL.ID = IDL;
  plot_zm_points(MAN.VL.H, MAN.VL.M, MAN.VL.ID, 'go');
  cont = cont+1;
  label{cont} = 'VLAND';
end
%
if (~isempty(EOMPOINTS))
  MAN.VEO.M =  EOMPOINTS;
  MAN.VEO.H =  EOHPOINTS;
  MAN.VEO.ID = IDEO;
  plot_zm_points(MAN.VEO.H, MAN.VEO.M, MAN.VEO.ID, 'mo');
  cont = cont+1;
  label{cont} = 'VEO';
end
%
MAN.Index = MAN_INDEX;
%
xlabel('Mach number'); ylabel('Altitude [m]'); grid on;
legend(label, 'Location','NorthEastOutside');
%
for i=1:length(MAN_INDEX)
  fprintf(fid2,'\nITRIM= ');
  fprintf(fid2, '%c', cnvt2_8chs(MAN_INDEX(i)));
end
fclose(fid2);
%
plot_nv(102, Nmax, Nmin, M*9.81, VA, VB, VC, VD, VS, CLMAX, USERSREF, filename_trim);
%
end
%-------------------------------------------------------------------------------
function print_master_controls(fid, NAME, LHEAD, SURF, ROT, offset)
  ncsm = length(NAME); 
  count = 0;
  nc = length(LHEAD); index = [];
  for (i=1:nc)
    for (j=1:ncsm)
      [LABLIstr] = strtok(cnvt2_8chs(NAME{j}));
      if (strfind(LABLIstr,SURF{i})==1)
        index = [index, j];
      end
    end
  end
  index = setdiff([1:ncsm], index);
  nc = length(index);
  for (n=1:nc)
    [LABLIstr] = cnvt2_8chs(NAME{index(n)});
    fprintf(fid, '%c', LABLIstr); 
    [defl] = cnvt2_8chs(ROT(index(n)));
    fprintf(fid, '%c', defl);
    count = count+1;
    if ( (mod(count-offset,4)==0) && (n~=nc))
      fprintf(fid,'\n        ');
    end
  end
end
%-------------------------------------------------------------------------------
function plot_zm_points(H, M, ID, linestyle)
%
  plot(M, H, linestyle);
  nn = length(H);
  acc = zeros(nn,1);
  for i=1:nn
    if (acc(i) == 0)
      index = [];
      IDsel = [];
      for j=1:nn
        if (acc(j) ==0)
          if (H(i) == H(j) && M(i) == M(j))
            index = [index, j];
            acc(j) = 1;
            IDsel = [IDsel, ID(j)];
          end
        end
      end
      text(M(i)*1.05,H(i)*1.05,num2str(IDsel),'Color', linestyle(1));
    end
  end
%
end
%-------------------------------------------------------------------------------
function [INDEX, H, M] = set_man_index(Hman, Mman, INDEX, H, M)
  nm = length(H);
  if (isequal(nm,0))
    INDEX = 1;
    H = Hman;
    M = Mman;
  else
    found = 0;
    for i=1:nm
      if (H(i) == Hman && M(i) == Mman)
        INDEX = [INDEX, i];
        found = 1;
        break;
      end
    end
    if (isequal(found,0))
      H = [H, Hman]; M = [M, Mman];
      INDEX = [INDEX, max(INDEX)+1];
    end
  end
end
%