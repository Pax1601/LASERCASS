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
% Regresion analysis modulus applied to wing.
% 
% Called by:    AFaWWE.m
% 
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080722      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str] = Regr_Wing(fid, pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure:
%--------------------------------------------------------------------------------------------------
str.wing.WSBOX = [];       % Structural weight for structural wing box       [kg], vector
str.wing.WPBOX = [];       % Primary structure for structural wing box       [kg], vector
str.wing.WTBOX = [];       % Total weight for structural wing box            [kg], vector
str.wing.WSC   = [];       % Structural weight for carrythrough structure    [kg], vector
str.wing.WPC   = [];       % Primary structure for carrythrough structure    [kg], vector
str.wing.WTC   = [];       % Total weight for carrythrough structure         [kg], vector
kg2lbs = 2.2046226218487757;
%--------------------------------------------------------------------------------------------------
% TOTAL WING WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.wing_regr)

  fprintf(fid, 'active option: ');

  switch pdcylin.fact.analw

      case 'linear'

          str.wing.WSBOX = 0.9843 .*(str.wing.WBOX);
          str.wing.WPBOX = 1.3442 .*(str.wing.WBOX);
          str.wing.WTBOX = 1.7372 .*(str.wing.WBOX);
  %        str.wing.WSC   = 0.9843 .*(str.wing.WC);
  %        str.wing.WPC   = 1.3442 .*(str.wing.WC);
  %        str.wing.WTC   = 1.7372 .*(str.wing.WC);
          str.wing.WSC   = 0.9843 .*(str.wing.WC);
          str.wing.WPC   = 1.3442 .*(str.wing.WC);
          str.wing.WTC   = str.wing.WPC;

          fprintf(fid, 'linear');

      otherwise

          str.wing.WSBOX = (1.3342/kg2lbs) .*(str.wing.WBOX*kg2lbs).^0.9701;
          str.wing.WPBOX = (2.1926/kg2lbs) .*(str.wing.WBOX*kg2lbs).^0.9534;
          str.wing.WTBOX = (3.7464/kg2lbs) .*(str.wing.WBOX*kg2lbs).^0.9268;
  %        str.wing.WSC   = 1.3342 .*(str.wing.WC)   .^0.9701;
  %        str.wing.WPC   = 2.1926 .*(str.wing.WC)   .^0.9534;
  %        str.wing.WTC   = 3.7464 .*(str.wing.WC)   .^0.9268;    
          str.wing.WSC   = (1.3342/kg2lbs) .*(str.wing.WC*kg2lbs).^0.9701;
          str.wing.WPC   = (2.1926/kg2lbs) .*(str.wing.WC*kg2lbs).^0.9534;
          str.wing.WTC   = str.wing.WPC;    

          fprintf(fid, 'quadratic');

  end
else
  str.wing.WTBOX = str.wing.WBOX;
  str.wing.WTC   = str.wing.WC;
end
% Save in the total mass and compute CG along x-axis
ww = 2*sum(str.wing.WTBOX) + str.wing.WTC;
%str.wing.CG = (sum(meancouple(geo.wing.x).*str.wing.WTBOX) + geo.wing.x(1)*str.wing.WTC*0.5)/(sum(str.wing.WTBOX) + str.wing.WTC*0.5);
% consider only structural weight to CG
% regression contribution to CG is added in guess.m from Add_NSM_conc results
str.wing.CG = (sum(meancouple(geo.wing.x).*str.wing.WBOX) + geo.wing.x(1)*str.wing.WC*0.5)/(sum(str.wing.WBOX) + str.wing.WC*0.5);
str.M = str.M + ww;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', ww);
