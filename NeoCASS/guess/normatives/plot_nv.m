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
%
function plot_nv(ifig, nmax, nmin, W, VA, VB, VC, VD, VS, CLMAX, SREF, filename)
figure(ifig); close; H = figure(ifig); hold on;
indE = find(filename=='.'); 
if ~isempty(indE)
  indE = indE(end);
  name = [filename(1:indE-1),'_nv','.fig'];
else
  name = [filename,'_nv','.fig'];
end
[RHO0, p0, T0, A0, mu0] = ISA_h(0);
NP = 20;
V = [VS:(VA-VS)/NP:VA];
n = (0.5 * RHO0 * CLMAX * SREF * V.^2)./W;

plot(VA,0,'rs');
plot(VB,0,'ks');
plot(VC,0,'bs');
plot(VD,0,'gs');
plot(VS,0,'ys'); 
label={'VA','VB','VC','VD','VS'};
legend(label,'Location','EastOutside');
plot(VA, nmax,'rs');
plot(VS, 1,'ys');
plot(V,n,'-k');

plot([VA VC], [nmax nmax],'-k');
plot([VC VD], [nmax nmax],'-k');
plot(VC, nmax,'bs');
plot(VD, nmax,'gs');

xlabel('V_{EAS} [m/s]');
ylabel('Load factor n');
title('Load diagram');
set(gca,'Ylim',[0,ceil(nmax)]'); set(gca,'yTick',[0:ceil(nmax)]')

% name = [filename,'_nv'];
if (exist(name, 'file'))
  delete(name);
end
saveas(H,name,'fig');