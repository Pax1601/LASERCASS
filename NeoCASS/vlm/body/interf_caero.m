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
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
% Determines CAERO within aero bodies
%
function INDEX = interf_caero(CAERO)

nbody = length(CAERO.body.ID);
% limit intersection to CT patched only
CT_AERO = [200 250 300 350 400 450 500 550 600 650];
[id, ia, ib] = intersect(CT_AERO, CAERO.ID);
ncaero = length(id);
P = zeros(1,3);
TOLL = 1.0e-3;
for i=1:nbody
  Rmat = CAERO.body.Rmat(:,:,i);
  IGN = [];
  for n=1:ncaero
    k = ib(n);
    P(1,1) = CAERO.geo.startx(k);
    P(1,2) = CAERO.geo.starty(k);
    P(1,3) = CAERO.geo.startz(k);
    P = (Rmat'*(P - CAERO.body.geo.ref_point(i,:))')';
    R = norm(P(2:3)) + TOLL;
    if (P(1,1)<CAERO.body.geo.x{i}(end))
      Rint = interp1(CAERO.body.geo.x{i}, CAERO.body.geo.R{i}, P(1,1), 'linear', 'extrap');
      if (R < Rint)
        IGN = [IGN, k];
      end
    end
  end
  INDEX{i} = IGN;
end