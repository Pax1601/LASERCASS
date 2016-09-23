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
function mach = dp_over_p2mach(dp_over_p)
% Return the mach number for a given delta p over p
  mach = sqrt(5. * ((dp_over_p + 1.) ^ (2. / 7.) - 1.));
  if (mach>1)
    dp_over_p_seek = dp_over_p;
    low = 1;
    high = 10;
    dp_over_p_low = mach2dp_over_p(low);
    dp_over_p_high = mach2dp_over_p(high); 
    guess = (low + high) / 2. ;
    dp_over_p_guess = mach2dp_over_p(guess);
    while( (abs(dp_over_p_guess - dp_over_p_seek) / dp_over_p_seek) > 1e-5)
      if dp_over_p_guess > dp_over_p_seek
        high = guess;
      else 
        low = guess;
      end
      guess = (low + high) / 2.;
      dp_over_p_guess = mach2dp_over_p(guess); 
    end
    mach = guess;
  end
end