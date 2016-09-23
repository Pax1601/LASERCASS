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
% Compute non structural mass linear density
%
%   Author: <andreadr@kth.se>
%
% Called by:    
%
% Calls:        
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080723      1.0      A. Da Ronch      Creation
%     120502      2.1.237  Riccobene        Modification
%
%*******************************************************************************
function mass_distr = massNSM_distr(nodes, domain, mass, dq, qtot, l_beam)
% Init vector
mass_distr = zeros(length(l_beam), 1);
if (mass>0)
  % discard last node
  nodes = nodes(1:end-1);
  % Find nodes inside domain
  indx = find( nodes>=domain(1) & nodes<=domain(2) );
  % Compute total over domain (q can be either a wetted area or a volume)
  qt = sum(dq(indx));
  % Compute non structural mass linear density [kg/m]
  mass_distr(indx, 1) = mass.*(dq(indx)./qt)./l_beam(indx);
end