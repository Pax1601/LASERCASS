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
% Return the CAS for a given TAS and altitude
function cas = tas2cas(tas, alt)
  [rho, p, T, A, mu] = ISA_h(alt);
  [dp, err] = tas2dp(tas, alt);
  if (err==1)
    error('The speed found exceeds 661.48 knots.');
    return;
  end
  cas = dp2cas(dp);
end
% Return the differential pressure (difference between pitot and static
% pressures) for a given TAS
function [dp, err] = tas2dp(tas, alt)
  err = 0;
  dp = 0;
  if (tas * 1.94 > 661.48)
    err = 1;
    return;
  end
  [rho0, p0, T0, A0, mu0] = ISA_h(0);
  [rho, p, T, A, mu] = ISA_h(alt);
  press_ratio = p/p0;
  temp_ratio = T/T0;
  density_ratio = press_ratio / temp_ratio;
  rho = rho0 * density_ratio;
  dp = speed2dp(tas, p, rho);
end