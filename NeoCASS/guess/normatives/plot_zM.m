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
% figure index
% Inputs VA, VC and VD (TAS) 
% Inputs HA, HC and HD altitudes at which VA, VC and VD are given 
% 
function plot_zM(nfig, VA, HA, VC, HC, VD, HD)

NP = 50;
HMAX = 20000;
NV = 4;
%
VCAS(1) = tas2cas(VA, HA);
VCAS(2) = tas2cas(VC, HC);
VCAS(3) = tas2cas(VD, HD);
VEAS(1) = cas2eas(VCAS(1), HA);
VEAS(2) = cas2eas(VCAS(2), HC);
VEAS(3) = cas2eas(VCAS(3), HD);
%
h = zeros(NP+1);
rho = zeros(NP+1,1);
p = zeros(NP+1,1);
T = zeros(NP+1,1);
AC = zeros(NP+1,1);
mu = zeros(NP+1,1);
[rho0, p0, T0, AC0, mu0] = ISA_h(0);
[rhoC, pC, TC, ACC, muC] = ISA_h(HC);
hstep = HMAX / NP;
h = [0:hstep:HMAX];
nh = NP+1;
for i=1:NP+1
  [rho(i), p(i), T(i), AC(i), mu(i)] = ISA_h(h(i));
end
%
nv = length(VCAS);
MEAS = zeros(nh,nv);
MCAS = zeros(nh,nv);
figure(nfig); close; figure(nfig);
hold on;
for i=1:nv
  MEAS(:,i) = (VEAS(i) ./ sqrt(rho/rho0))./ AC;
end

for i=1:nv
  for k=1:nh
    MCAS(k,i) = cas_alt2mach(VCAS(i), h(k));
  end 
end
%
HMAX = 10000;
%
plot(MCAS(:,1),h,'--g', 'LineWidth', 1);
plot(MCAS(:,2),h,'--b', 'LineWidth', 1);
plot(MCAS(:,3),h,'--r', 'LineWidth', 1);
plot(MEAS(:,1),h,'g', 'LineWidth', 1);
plot(MEAS(:,2),h,'b', 'LineWidth', 1);
plot(MEAS(:,3),h,'r', 'LineWidth', 1);
%
ylim([0 HMAX]);
grid on;
labels = {'VA_{CAS}', 'VC_{CAS}','VD_{CAS}','VA_{EAS}', 'VC_{EAS}', 'VD_{EAS}'};
legend(labels)
xlabel('Mach');
ylabel('Altitude [m]');
end

