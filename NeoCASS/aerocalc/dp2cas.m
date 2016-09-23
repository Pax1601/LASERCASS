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
% Return CAS given differential pressure
function cas = dp2cas(dp)
  [rho0, p0, T0, A0, mu0] = ISA_h(0);
  err = 1;
  while (err==1)
    [cas, err] = dp2speed(dp, p0, rho0);
    if (err==1)
      % supersonic case
      dp_seek = dp;
      low = 340.0;
    % This function works up to approximately 6,600 kt CAS. 
    % Increase high if error persists.
      high = 3400.0;
      dp_low = super_cas2dp(low);
      if (dp_low > dp_seek)
       error('Initial lower cas guess is too high');
      end  
      dp_high = super_cas2dp(high);
      if (dp_high < dp_seek)
       error('Initial upper cas guess is too low');
      end  
      guess = (low + high) /2;
      dp_guess = super_cas2dp(guess);
      while(abs(dp_guess - dp_seek) / dp_seek > 1.0e-5)
        if (dp_guess > dp_seek)
          high = guess;
        else
          low = guess;
        end
        guess = (low + high) /2;
        dp_guess = super_cas2dp(guess);
      end
    end
  end
end
%----------------------------------------------------------------------------
function dp = super_cas2dp(mcas)
  [rho0, p0, T0, A0, mu0] = ISA_h(0);
  F = (1.25 ^ 2.5 * (2.4 ^ 2.) ^ 2.5) * 1.2;
  dp_over_P0 = (F * (mcas / A0) ^ 7.) / (7. * (mcas / A0) ^ 2. - 1.)^2.5 -1;
  dp = dp_over_P0 * p0; 
end