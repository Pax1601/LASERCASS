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
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Regresion analysis modulus applied to tail_booms.
% 
% Called by:    AFaWWE.m
%
% Calls:        
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%
%*******************************************************************************
function str = Regr_Tbooms(fid, pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure: GEO.TBOOMS
%--------------------------------------------------------------------------------------------------
str.tbooms.WS   = [];        % Structural weight        [kg], vector
str.tbooms.WP   = [];        % Primary structure        [kg], vector
str.tbooms.WTOT = [];        % Total weight             [kg], vector
str.tbooms.CG   = [];        % Tail boom cg             [m], scalar
kg2lbs = 2.2046226218487757;

%--------------------------------------------------------------------------------------------------
% TOTAL TAIL BOOMS WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.tboom_regr)

fprintf(fid, 'active option: ');

  switch pdcylin.fact.analf

      case 'linear'

          str.tbooms.WS   = 1.3503 .*str.tbooms.WI;
          % str.tbooms.WP   = 1.8872 .*str.tbooms.WI;
          % str.tbooms.WTOT = 2.5686 .*str.tbooms.WI;
          %  str.tbooms.WTOT = 1.8872 .*str.tbooms.WI;

          % Since the regression was originally intended for the fuselage, keep it
          % disabled
          str.tbooms.WTOT = str.tbooms.WI;
          fprintf(fid, 'linear');

      otherwise

          str.tbooms.WS   = (1.1304/kg2lbs) .*(str.tbooms.WI*kg2lbs) .^1.0179;
  %        str.tbooms.WP   = 1.6399 .*str.tbooms.WI .^1.0141;
  %        str.tbooms.WTOT = 3.9089 .*str.tbooms.WI .^0.9578;
          str.tbooms.TOT   = (1.6399/kg2lbs) .*(str.tbooms.WI*kg2lbs) .^1.0141;
          fprintf(fid, 'quadratic');

  end
else
  str.tbooms.WTOT   = str.tbooms.WI;
end
% Save in the total mass (if symmetry flag is activated double the weight,
% since it is referred to a single tail boom)
wf = 0;
if aircraft.Tailbooms.symmetry
  wf = 2*sum(str.tbooms.WTOT);
else
  wf = sum(str.tbooms.WTOT);
end
str.tbooms.CG = sum(meancouple(geo.tbooms.x).*str.tbooms.WTOT)/(sum(str.tbooms.WTOT));
str.M = str.M + wf;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', wf);
