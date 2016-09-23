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
% Regression analysis modulus applied to canard.
% 
% Called by:    AFaWWE.m
% 
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080722      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str] = Regr_Canard(fid, pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure:
%--------------------------------------------------------------------------------------------------
str.canard.WSBOX = [];       % Structural weight for structural canard box       [kg], vector
str.canard.WPBOX = [];       % Primary structure for structural canard box       [kg], vector
str.canard.WTBOX = [];       % Total weight for structural canard box            [kg], vector
str.canard.WSC   = [];       % Structural weight for carrythrough structure     [kg], vector
str.canard.WPC   = [];       % Primary structure for carrythrough structure     [kg], vector
str.canard.WTC   = [];       % Total weight for carrythrough structure          [kg], vector
str.canard.CG    = [];       % canard cg [m] scalar
kg2lbs = 2.2046226218487757;
%--------------------------------------------------------------------------------------------------
% TOTAL HT WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.canard_regr)

  fprintf(fid, 'active option: ');

  switch pdcylin.fact.analh

      case 'linear'

          str.canard.WSBOX = 0.9843 .*(str.canard.WBOX);
          str.canard.WPBOX = 1.3442 .*(str.canard.WBOX);
          str.canard.WTBOX = 1.7372 .*(str.canard.WBOX);
  %        str.canard.WSC   = 0.9843 .*(str.canard.WC);
  %        str.canard.WPC   = 1.3442 .*(str.canard.WC);
  %        str.canard.WTC   = 1.7372 .*(str.canard.WC);
          str.canard.WSC   = 0.9843 .*(str.canard.WC);
          str.canard.WPC   = 1.3442 .*(str.canard.WC);
          str.canard.WTC   = str.canard.WPC;

          fprintf(fid, 'linear');

      otherwise

          str.canard.WSBOX = (1.3342/kg2lbs) .*(str.canard.WBOX*kg2lbs).^0.9701;
          str.canard.WPBOX = (2.1926/kg2lbs) .*(str.canard.WBOX*kg2lbs).^0.9534;
          str.canard.WTBOX = (3.7464/kg2lbs) .*(str.canard.WBOX*kg2lbs).^0.9268;
  %        str.canard.WSC   = 1.3342 .*(str.canard.WC)   .^0.9701;
  %        str.canard.WPC   = 2.1926 .*(str.canard.WC)   .^0.9534;
  %        str.canard.WTC   = 3.7464 .*(str.canard.WC)   .^0.9268;    
          str.canard.WSC   = (1.3342/kg2lbs) .*(str.canard.WC*kg2lbs).^0.9701;
          str.canard.WPC   = (2.1926/kg2lbs) .*(str.canard.WC*kg2lbs).^0.9534;
          str.canard.WTC   = str.canard.WPC;    

          fprintf(fid, 'quadratic');

  end
%
else
  str.canard.WTBOX  = str.canard.WBOX;
  str.canard.WTC    = str.canard.WC;
end
% Save in the total mass
wh = 2*sum(str.canard.WTBOX) + str.canard.WTC;
str.canard.CG = (sum(meancouple(geo.canard.x).*str.canard.WTBOX) + geo.canard.x(1)*str.canard.WTC*0.5)/(sum(str.canard.WTBOX) + str.canard.WTC*0.5);
str.M = str.M + wh;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', wh);
