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
% Regresion analysis modulus applied to wing.
% 
% Called by:    AFaWWE.m
% 
% Calls:        
% 
% MODIFIED 2008-07-22
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [str] = Regr_Wing2(fid, pdcylin, aircraft, geo, loads, str)


%--------------------------------------------------------------------------------------------------
% Initialize structure:
%--------------------------------------------------------------------------------------------------
str.wing2.WSBOX = [];       % Structural weight for structural wing2 box       [kg], vector
str.wing2.WPBOX = [];       % Primary structure for structural wing2 box       [kg], vector
str.wing2.WTBOX = [];       % Total weight for structural wing2 box            [kg], vector
str.wing2.WSC   = [];       % Structural weight for carrythrough structure    [kg], vector
str.wing2.WPC   = [];       % Primary structure for carrythrough structure    [kg], vector
str.wing2.WTC   = [];       % Total weight for carrythrough structure         [kg], vector
kg2lbs = 2.2046226218487757;
%--------------------------------------------------------------------------------------------------
% TOTAL wing2 WEIGHT
%--------------------------------------------------------------------------------------------------
if (pdcylin.smartcad.wing2_regr)
  fprintf(fid, 'active option: ');

  switch pdcylin.fact.analw

      case 'linear'

          str.wing2.WSBOX = 0.9843 .*(str.wing2.WBOX);
          str.wing2.WPBOX = 1.3442 .*(str.wing2.WBOX);
          str.wing2.WTBOX = 1.7372 .*(str.wing2.WBOX);
  %        str.wing2.WSC   = 0.9843 .*(str.wing2.WC);
  %        str.wing2.WPC   = 1.3442 .*(str.wing2.WC);
  %        str.wing2.WTC   = 1.7372 .*(str.wing2.WC);
          str.wing2.WSC   = 0.9843 .*(str.wing2.WC);
          str.wing2.WPC   = 1.3442 .*(str.wing2.WC);
          str.wing2.WTC   = str.wing2.WPC;

          fprintf(fid, 'linear');

      otherwise

          str.wing2.WSBOX = (1.3342/kg2lbs) .*(str.wing2.WBOX*kg2lbs).^0.9701;
          str.wing2.WPBOX = (2.1926/kg2lbs) .*(str.wing2.WBOX*kg2lbs).^0.9534;
          str.wing2.WTBOX = (3.7464/kg2lbs) .*(str.wing2.WBOX*kg2lbs).^0.9268;
  %        str.wing2.WSC   = 1.3342 .*(str.wing2.WC)   .^0.9701;
  %        str.wing2.WPC   = 2.1926 .*(str.wing2.WC)   .^0.9534;
  %        str.wing2.WTC   = 3.7464 .*(str.wing2.WC)   .^0.9268;    
          str.wing2.WSC   = (1.3342/kg2lbs) .*(str.wing2.WC*kg2lbs).^0.9701;
          str.wing2.WPC   = (2.1926/kg2lbs) .*(str.wing2.WC*kg2lbs).^0.9534;
          str.wing2.WTC   = str.wing2.WPC;    

          fprintf(fid, 'quadratic');

  end
else
  str.wing2.WTBOX = str.wing2.WBOX;
  str.wing2.WTC   = str.wing2.WC;
end
% Save in the total mass
ww = 2*sum(str.wing2.WTBOX) + str.wing2.WTC;
str.wing2.CG = (geo.wing2.x'*str.wing2.WTBOX + geo.wing2.x(1)*str.wing2.WTC*0.5)/(sum(str.wing2.WTBOX) + str.wing2.WTC*0.5);
str.M = str.M + ww;
fprintf(fid, '\n\tTotal weight: %7.0f Kg.', ww);
