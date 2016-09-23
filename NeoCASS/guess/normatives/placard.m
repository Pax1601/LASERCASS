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
% Plot Placard diagram given:
% figure index nfig
% Maximum Cruise speed VC (CAS)
% Cruise altitude HC
% Dive speed VD (CAS)
% Dive Mach number MD
%

function [VELC, HCRU, VELD, HDIVE] = placard(nfig, HC, MC, HD, MD)
  figure(nfig); close; figure(nfig); hold on;
  KFAC = 1.2;
  vstep = 20;
  hstep = 1;
  HMAX = 16000;
%
%
  VCCAS = mach_alt2cas(MC, HC);
  VCEAS = cas2eas(VCCAS, HC);
  VCTAS = cas2tas(VCCAS, HC);
%
  VDCAS = mach_alt2cas(MD, HD);
  VDEAS = cas2eas(VDCAS, HD);
  VDTAS = cas2tas(VDCAS, HD);
%
  [EASC, CASC, HCRU]  = placard_knee(VCTAS, MC, HC, HMAX);
  [EASD, CASD, HDIVE] = placard_knee(VDTAS, MD, HD, HMAX);
  TASC = CASC;
  TASD = CASD;
  nc = length(CASC);
  nd = length(CASD);
  for i=1:nc
    TASC(i) = cas2tas(CASC(i), HCRU(i));
  end 
  for i=1:nd
    TASD(i) = cas2tas(CASD(i), HDIVE(i));
  end 
%
  title('Placard diagram');
  xlabel('V [m/s]');
  ylabel('Altitude [m]');
%  set(gca,'XTick',[0.0:vstep:max(V)*KFAC]');
  set(gca,'YTick',[0.0:1000:HMAX]')
%
  plot(EASC, HCRU,'-k','LineWidth',2);
  plot(CASC, HCRU,'--k','LineWidth',2);
  plot(TASC, HCRU,'-.k','LineWidth',2);
  plot(EASD, HDIVE,'-r','LineWidth',2);
  plot(CASD, HDIVE,'--r','LineWidth',2);
  plot(TASD, HDIVE,'-.r','LineWidth',2);
  grid on;
%
  label={'VC_{EAS}','VC_{CAS}','VC_{TAS}','VD_{EAS}','VD_{CAS}','VD_{TAS}'};
  legend(label,'Location','NorthEastOutside');
  VELC = zeros(nc, 3);
  VELD = zeros(nd, 3);
  VELC(:,1) = EASC;
  VELC(:,2) = CASC;
  VELC(:,3) = TASC;
  VELD(:,1) = EASD;
  VELD(:,2) = CASD;
  VELD(:,3) = TASD;
%
end
%
function [EASC, CASC, H] = placard_knee(VCTAS, MC, HC, HMAX)
%
  NP = 100;
%
  [RHOC, p, T, AC, mu] = ISA_h(HC);
  [RHO0, p, T, A0, mu] = ISA_h(0);
  H1 = [0 : HC/NP : HC]';
  H2 = [HC : (HMAX-HC)/NP : HMAX]'; 
%
  n1 = length(H1); n2 = length(H2);
  CASC = zeros(n1+n2,1);
  EASC = zeros(n1+n2,1);
%
  for i=1:n1
    EASC(i) = VCTAS * sqrt(RHOC/RHO0);
    CASC(i) = eas2cas(EASC(i), H1(i));
  end
%
  for i=1:n2
    [RHOH, p, T, AH, mu] = ISA_h(H2(i));
    EASC(i+n1) = MC * AH * sqrt(RHOH/RHO0);
    CASC(i+n1) = mach_alt2cas(MC, H2(i));
  end
  H = [H1; H2];
end
