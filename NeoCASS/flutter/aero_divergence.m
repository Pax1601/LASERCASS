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
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
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
%   Author: Alessandro Scotti
%
%***********************************************************************************************************************
%
% Solves divergence problem when given Qhh and Khh Matrices
%
% Input: Khh - Stiffness Matrix
%        AER - QHHL Matrix
%        Kfreq - Reduced Frequencies Vector
%
% Output: Divergpressure - Dynamic Divergence Pressure
%         Divergvelocity - Divergence Velocity
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified by L.Cavagna
function [Div_P, Div_V] = aero_divergence(rho, AER, Khh, Kfreq, MID)
%
Div_P = -1;
Div_V = -1;
index = find(MID > 6);
if (~isempty(index))
  DIVERG = eig(Khh(index,index), real(AER(index,index,1) - Kfreq(1).*AER(index,index,2)));
  %
  for i=1:length(DIVERG)
    if (~isreal(DIVERG(i)))
      DIVERG(i) = 0;
    end
  end
  DIVERG = DIVERG(find(DIVERG>0));
  if ~isempty(DIVERG)
    Div_P = min(DIVERG);
    Div_V = sqrt(2*Div_P/rho);
  end
end
