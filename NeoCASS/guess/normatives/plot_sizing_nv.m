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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Luca Cavagna
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%*******************************************************************************
% This function loads the fig picture with original NV diagram and superpose
% the results from NeoCASS
% Input:
% filename: .fig file with the initial nv diagram saved by GUESS
% mconf: mass configuration to plot (only 1 for GUESS standard)
%
function plot_sizing_nv(filename, loads, mconf)
%
nconf = loads.nconf;
if (mconf>nconf)
  fprintf(1,'\n### Warning: mass configuration %d not available.', mconf);
  return
end
%
KX = 1.01;
KY = 1.03;
open(filename);
hold on
nm = length(loads.nv.V);
ID = [1:nm];
k = mconf;
nmax = max(loads.nv.N(:,k));
plot(loads.nv.V, loads.nv.N(:,k),'o');
for kk=1:nm
  text(loads.nv.V(kk)*KX, loads.nv.N(kk,k)*KY, num2str(kk));
end
grid on
title(['Load diagram, configuration', num2str(mconf)]);
set(gca,'Ylim',[0,ceil(nmax)]'); set(gca,'yTick',[0:ceil(nmax)]')

