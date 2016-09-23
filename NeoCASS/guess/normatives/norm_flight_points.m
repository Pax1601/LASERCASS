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
% Cruise altitude (m)
% Minimum cruise mach number MCRU
% CLMAX: clean maximum lift coefficient
% CLMAXTO: TO maximum lift coefficient
% CLMAXLAND: landing maximum lift coefficient
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
function MAN = norm_flight_points(fid, HCRU, MCRU, HMAX, ...
                          CLMAX, CLMAXTO, CLMAXLAND, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                          VSINK, STROKE, LNDGEFF, jwflag, swflag, ...
                          filename_aircraft, filename_trim, ManCheck, FarIndex)
%
if (HMAX<HCRU)
  fprintf(fid,'\n\t ### Warning: cruise altitude HCRU higher than max ceiling altitude HMAX. ');
  fprintf(fid,'\n\t     HMAX will be set to HCRU.');
  HMAX = HCRU;
end
%
fprintf(fid,'\n\t User defined data:');
fprintf(fid,'\n\n\t - cruise altitude HCRU:          %g.', HCRU);
fprintf(fid,'\n\t - mach cruise MCRU:              %g.', MCRU);
fprintf(fid,'\n\t - max ceiling altitude HMAX:     %g.', HMAX);
fprintf(fid,'\n\t - max CL at clean conf CLMAX:    %g.', CLMAX);
fprintf(fid,'\n\t - max CL at TO conf CLMAXTO:     %g.', CLMAXTO);
fprintf(fid,'\n\t - max CL at LAND conf CLMAXLAND: %g.', CLMAXLAND);
fprintf(fid,'\n\t - CL/alpha slope CLALPHAD:      %g.', CLALPHAD);
fprintf(fid,'\n\t - reference surface USERSREF:    %g.', USERSREF);
fprintf(fid,'\n\t - flap at TO FLAPTO:             %g degs.', FLAPTO);
fprintf(fid,'\n\t - flap at LAND FLAPLAND:         %g degs.', FLAPLAND);
fprintf(fid,'\n\t - landing sink speed VSINK:      %g m/s.', VSINK);
fprintf(fid,'\n\t - landing stroke length VSTROKE: %g m/s.', STROKE);
fprintf(fid,'\n\t - strut efficiency LNDGEFF:      %g.', LNDGEFF);
%
MAN = [];
%
LBS2KG  = 0.45;
NORM = FarIndex;
CHECK_NORM = 1;
% Let GUESS select the normative to be used
if (FarIndex ==0)
%
  CHECK_NORM = 0;
  aircraft = neocass_xmlwrapper(filename_aircraft);
  aircraft = setup_geofile_conversion(aircraft);
  M = sum(aircraft.weight_balance.COG(aircraft.weight_balance.COG(:,1,1)~=0,4,1));
  MLB = M/LBS2KG;
  npax = aircraft.cabin.Passenger_accomodation;
%
  fprintf(fid,'\n\t Passengers: %d [].', npax);
  fprintf(fid,'\n\t Total mass: %g [kg].', M);
%
% Propulsion_type
% 0 = turbofan
% 1 = turboprop tractor
% 2 = turboprop pusher
% 3 = propfan
%
  switch (aircraft.engines1.Propulsion_type==0)
    case 0
      fprintf(fid,'\n\t Engine type: jet/fan.');
%
      if (MLB > 12500)
        if (npax>=10)
          NORM = 2; % CAS25
          fprintf(fid,'\n\t CAS25 will be applied');     
        end
      else
        if (npax<10)
          NORM = 1; % CAS23
          fprintf(fid,'\n\t CAS23 will be applied');     
        else
         fprintf(fid,'\n\t Warning: CAS23 should be applied (MTOW<12500lbs) but number of passenger is too high (>9).');     
        end
      end
%
    case {1,2,3}
%
      fprintf(fid,'\n\t Engine type: propeller driven.');

      if (MLB < 12500)
        if (npax<10)
          NORM = 1; % CAS23
         fprintf(fid,'\n\t CAS23 will be applied');     
        else
         fprintf(fid,'\n\t ### Warning: CAS23 should be applied (MTOW<12500lbs) but number of passenger is too high (>9).');     
        end
      end
      if (MLB > 12500 && MLB < 19000)
        if (npax<19)
          NORM = 1; % CAS23
          fprintf(fid,'\n\t CAS23 will be applied');     
        else
         fprintf(fid,'\n\t ### Warning: CAS23 should be applied (12500<MTOW<19000lbs) but number of passenger is too high (>18).');     
        end
      end
      if (MLB > 19000)
        if (npax>19)
          NORM = 2; % CAS25
          fprintf(fid,'\n\t CAS25 will be applied');     
        else
         fprintf(fid,'\n\t ### Warning: CAS25 should be applied (MTOW>19000lbs) for propeller driven aircraft but number of passenger is too low (<18).');     
        end
      end
% 
    otherwise
      fprintf(fid,'\n\t Unknown propulsion type (check aircraft.Engines1.Propulsion_type is between 0 and 3)');     
      return;
  end
%
end
%
switch (NORM)

  case 1
    MAN = EASA_CAS_23(fid, HCRU, MCRU, HMAX,...
                     CLMAX, CLMAXTO, CLMAXLAND, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                     VSINK, STROKE, LNDGEFF, jwflag, swflag, ...
                     filename_aircraft, filename_trim, ManCheck, CHECK_NORM);
  case 2
    MAN = EASA_CAS_25(fid, HCRU, MCRU, HMAX,...
                     CLMAX, CLMAXTO, CLMAXLAND, CLALPHAD, USERSREF, FLAPTO, FLAPLAND, ...
                     VSINK, STROKE, LNDGEFF, jwflag, swflag, ...
                     filename_aircraft, filename_trim, ManCheck, CHECK_NORM);
  otherwise
    fprintf('### Warning: unable to select a normative consistent with propulsion type, number of passengers and MTOW.');
end


end