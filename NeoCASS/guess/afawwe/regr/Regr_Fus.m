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
%   Author: <andreadr@kth.se>
%
% Regresion analysis modulus applied to fuselage.
% 
% Called by:    AFaWWE.m
% 
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080722      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str] = Regr_Fus(fid, pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure: GEO.FUS
%--------------------------------------------------------------------------------------------------
str.fus.WS   = [];        % Structural weight        [kg], vector
str.fus.WP   = [];        % Primary structure        [kg], vector
str.fus.WTOT = [];        % Total weight             [kg], vector
str.fus.CG   = []; % fusulage cg                                         [m], scalar
kg2lbs = 2.2046226218487757;
%--------------------------------------------------------------------------------------------------
% TOTAL FUSELAGE WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.fuse_regr)

  fprintf(fid, 'active option: ');

  switch pdcylin.fact.analf

      case 'linear'

          str.fus.WS   = 1.3503 .*str.fus.WI;
  %        str.fus.WP   = 1.8872 .*str.fus.WI;
  %        str.fus.WTOT = 2.5686 .*str.fus.WI;

          str.fus.WTOT   = 1.8872 .*str.fus.WI;

          fprintf(fid, 'linear');

      otherwise

          str.fus.WS   = (1.1304/kg2lbs) .* (str.fus.WI*kg2lbs) .^1.0179;
  %        str.fus.WP   = 1.6399 .*str.fus.WI .^1.0141;
  %        str.fus.WTOT = 3.9089 .*str.fus.WI .^0.9578;

          str.fus.WTOT   = (1.6399/kg2lbs) .* (str.fus.WI*kg2lbs) .^1.0141;

          fprintf(fid, 'quadratic');

  end
else
  str.fus.WTOT = str.fus.WI;
end
% Save in the total mass and compute CG along x-axis
wf = sum(str.fus.WTOT); 
str.fus.CG = sum(meancouple(geo.fus.x).*str.fus.WTOT)/(sum(str.fus.WTOT));
str.M = str.M + wf;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', wf);
