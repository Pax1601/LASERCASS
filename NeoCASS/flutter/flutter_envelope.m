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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************

function [vf_eas, hf] = flutter_evelope(vf, rho, Mach)

  R = 287;
  a = -6.5;
  p0 = 101325;
  h1 = 11;
  %h2 = 20000;
  TRef = 288.16;
  RhoRef = 1.225;
  Th1 = TRef + a * h1;
  ph1 = p0 * (Th1/TRef)^5.25588;

  vf_eas = vf * sqrt(rho / RhoRef);
  q = 0.5 * RhoRef * vf_eas * vf_eas;
  p = 2*q / (1.4 * Mach^2);

  if (p < ph1) % use approximate formula (h > 11 km)
    
    sigma = p/ph1;
    hf = -(15*sigma - 37.0)/(sigma+1);
    if (hf > 25)
      hf = 25;
    end

  else % h < 11 km
    
    pRef1 = RhoRef * R * TRef;
    hf = -44.3 * ((p/pRef1).^(1/5.25)-1);

  end

end
