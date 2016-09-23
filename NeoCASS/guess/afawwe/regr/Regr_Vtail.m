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
% Regresion analysis modulus applied to vertical tail.
% 
% Called by:    AFaWWE.m
% 
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080722      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str] = Regr_Vtail(fid, pdcylin, aircraft, geo, loads, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------------------------------
str.vtail.WSBOX = [];       % Structural weight for structural wing box       [kg], vector
str.vtail.WPBOX = [];       % Primary structure for structural wing box       [kg], vector
str.vtail.WTBOX = [];       % Total weight for structural wing box            [kg], vector
str.vtail.WSC   = [];       % Structural weight for carrythrough structure    [kg], vector
str.vtail.WPC   = [];       % Primary structure for carrythrough structure    [kg], vector
str.vtail.WTC   = [];       % Total weight for carrythrough structure         [kg], vector
kg2lbs = 2.2046226218487757;

%--------------------------------------------------------------------------------------------------
% TOTAL VT WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.vt_regr)

  fprintf(fid, 'active option: ');

  switch pdcylin.fact.analv

      case 'linear'

          str.vtail.WSBOX = 0.9843 .*(str.vtail.WBOX);
          str.vtail.WPBOX = 1.3442 .*(str.vtail.WBOX);
          str.vtail.WTBOX = 1.7372 .*(str.vtail.WBOX);
  %        str.vtail.WSC   = 0.9843 .*(str.vtail.WC);
  %        str.vtail.WPC   = 1.3442 .*(str.vtail.WC);
  %        str.vtail.WTC   = 1.7372 .*(str.vtail.WC);
          str.vtail.WSC   = 0.9843 .*(str.vtail.WC);
          str.vtail.WPC   = 1.3442 .*(str.vtail.WC);
          str.vtail.WTC   = str.vtail.WPC;

          fprintf(fid, 'linear');

      otherwise

          str.vtail.WSBOX = (1.3342/kg2lbs) .*(str.vtail.WBOX*kg2lbs) .^0.9701;
          str.vtail.WPBOX = (2.1926/kg2lbs) .*(str.vtail.WBOX*kg2lbs) .^0.9534;
          str.vtail.WTBOX = (3.7464/kg2lbs) .*(str.vtail.WBOX*kg2lbs) .^0.9268;
  %        str.vtail.WSC   = 1.3342 .*(str.vtail.WC) .^0.9701;
  %        str.vtail.WPC   = 2.1926 .*(str.vtail.WC) .^0.9534;
  %        str.vtail.WTC   = 3.7464 .*(str.vtail.WC) .^0.9268;    
          str.vtail.WSC   = (1.3342/kg2lbs) .*(str.vtail.WC*kg2lbs) .^0.9701;
          str.vtail.WPC   = (2.1926/kg2lbs) .*(str.vtail.WC*kg2lbs) .^0.9534;
          str.vtail.WTC   = str.vtail.WPC;    

          fprintf(fid, 'quadratic');

  end
else
  str.vtail.WTBOX = str.vtail.WBOX;
  str.vtail.WTC   = str.vtail.WC;
end
% Save in the total mass and compute CG along x-axis
wv = sum(str.vtail.WTBOX);
str.vtail.CG = sum(meancouple(geo.vtail.x).*str.vtail.WTBOX)/sum(str.vtail.WTBOX);
str.M = str.M + wv;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', wv);
