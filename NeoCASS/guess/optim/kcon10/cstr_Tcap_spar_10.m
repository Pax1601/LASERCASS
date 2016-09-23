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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     251111      1.0     L. Cavagna       Creation
%                         
%*******************************************************************************
%
% function guess
%
%   DESCRIPTION: Run GUESS Module from structural sizing to stick model creation
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                X0             array 3x1  current solution
%                data           struct     internal parameters
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                CIN,CEQ        array      constraint value for < and =
%
%    REFERENCES:
%
%*******************************************************************************

function [CIN, CEQ] = cstr_ZTcap_spar_10(X, data)
%
CIN = [];
CEQ = [];
PRINT = 0;
% coefficients for plate crippling to be used in Gerard formula
% OEF_K = 0.6424
% OEF_EXP = -0.788;
OEF_K = 0.546;
OEF_EXP = -0.8;
%
% Design variables
%
tw = X(1); % web thickness
%
% SPAR CAP
%
%   B1
% ___________________
% |                  T1
% ------------------- 
%         |  |        
%         |  |         
%         |  |        B2
%         |  |
%         |  |        
%         |  |
%         |  |        
%          T2
%
B1 = X(2); % T width 
T1 = X(3); % thickness
B2 = X(4); % T height
T2 = X(5); % % thickness
%
% STIFFENER
%  ___
% |   |       
% |   | 
% |   | 
% |   | 
% |   | 
% |   | 
% |   | 
% |------------------
% |                  tu
% ------------------- 
%         BU

BU = X(6); % L widht and height
tu = X(7); % L thickness
RP = X(8); % stiffener pitch
%
H = data.geo.h;   % section height
Astr = data.geo.Astr;
Nstr = data.geo.Nstr;
CHORD = data.geo.C;
tskin = data.geo.tskin;
E = data.param.E; % Young modulus
%
smax = data.param.smax; % yeld stress
sult = data.param.sult; % ultimate stress
Ty = data.load.Ty;      % shear force 
Mx = data.load.Mx;      % bend moment
%
%
CINDEX = data.param.Cindex;
%-------------------------------------------------------------------------------
% T CAP
%
Acap = B1*T1 + B2*T2;
Y = (B1*T1^2+T2*B2*(2*T1+B2))/2/Acap;
Ix = 1/12*B1*T1^3 +B1*T1*(Y-T1/2)^2+ 1/12*T2*B2^3 + B2*T2*(Y-(B2/2+T1))^2;
% 
He = H - 2*Y;
Hu = He - (B2+T1-Y);
Hc = Hu - (B2+T1-Y)/2;

%-------------------------------------------------------------------------------
% spar web failure
%
tau = abs(Ty) / He / tw;
%tau = abs(Ty) * Acap* He/2 / (Ix + Acap*He^2/4)/tw
tb = tw/min(RP, Hc);
ab = max(RP,Hc)/min(RP,Hc);
Kss = -0.138*(ab^3)+1.4076*(ab^2)-4.6814*ab+10.467;
Fscr = (pi^2)*Kss*E*(tb^2)/12/(1-0.33^2);
%
%ab = 1/ab;
%Kss = 2.865 * ab^2 + 0.787 * ab + 4.807;
% web fixity coefficients
% single upright
tuw = tu/tw;
if (tuw<=0.6)
  Ru = 0.3299*tuw^2+0.2904*tuw+0.0015;
elseif (tuw<=1.25)
  Ru = 0.9977*tuw - 0.3011;
else
  if (tuw>2.5)
    Ru = 0.04*tuw + 1.1917;
  else
    Ru = 0.2667*tuw^3 - 1.7829*tuw^2+4.0405*tuw-1.8343;
  end
end
% web fixity coefficients
% single flange
t2w = T2/tw;
%if (t2w<=0.6)
%  Rf = 0.3299*t2w^2+0.2904*t2w+0.0015;
%elseif (t2w<=1.25)
%  Rf = 0.9977*t2w - 0.3011;
%else
%  if (t2w>2.5)
%    Rf = 0.04*t2w + 1.1917;
%  else
%    Rf = 0.2667*t2w^3 - 1.7829*t2w^2+4.0405*t2w-1.8343;
%  end
%end

if (t2w<=1)
  Rf = t2w;
elseif (t2w<=2.5)
  Rf = 0.0533*t2w^3 - 0.52*t2w^2+1.6867*t2w-0.22;
else
  Rf = 0.08*t2w + 1.38;
end


if (RP/Hc<=1)
  ab = RP / Hc;
  Kss = 2.865 * ab^2 + 0.787 * ab + 4.807;
  Rlim = Ru;
  Fscr_e = Kss * E * (tw/RP)^2*(Ru + 0.5*(Rf-Ru)*(RP/Hc)^3);
else
  ab = Hc / RP;
  Kss = 2.865 * ab^2 + 0.787 * ab + 4.807;
  Rlim = Rf;
  Fscr_e = Kss * E * (tw/Hc)^2*(Rf + 0.5*(Ru-Rf)*(Hc/RP)^3);
end
Fscr_e = Fscr_e / 1000;
% plasticity correction
if (Fscr_e<5) % kpsi
  Fscr = Fscr_e;
elseif (Fscr_e<41)  % kpsi
  Fscr = (0.8056*Fscr_e + 0.9722); 
else
  Fscr = (0.2111*Fscr_e + 24.937); 
end
Fscr = Fscr * 1000;
% 
K = tanh(0.5*log10(tau/Fscr));
if (tau<2*Fscr)
  rho = (tau-Fscr)/(tau+Fscr);
  K = 0.434*(rho + 1/3*rho^3);
  if (tau<Fscr)
    K = 0;
  end
end
%
% effective cross sectional area of upright
%
B1u = BU;
B2u = BU;
T1u = tu;
T2u = tu;
%
Au = B1u*T1u + B2u*T2u;
%Yu = (B2u^2+(B1u-T2u)*T1u ) / (2*(B1u+B2u-T1u));
%Ixu = ( T1u*(B2u-Yu)^2 + B1u*Yu^3 - (B1u-T2u)*(Yu-T1u)^3)/3
Yu = (B1u*T1u*T1u/2 + B2u*T2u*(T1u+B2u/2))/Au;
Ixu = 1/12*B1u*T1u^3 + B1u*T1u*(T1u/2-Yu)^2 + 1/12*T2u*B2u^3 + B2u*T2u*(T1u+B2u/2-Yu)^2;
eu = tw/2 + Yu;
rhou = sqrt(Ixu/Au);
Aue = Au /(1+(eu/rhou)^2);
% buckling angle in pure diag tensions
num = 1 + (He*tw/(2*Acap));
den = 1 + (RP*tw/Aue);
ALPHADT = atan((num/den)^0.25);
ALPHA = pi/4 - K*(pi/4-ALPHADT);

C1 = 1/sin(2*ALPHA)-1;
% flexibility cap
wd = RP*sin(ALPHA)*(tw*(2/Ix)/(4*He))^0.25;

%if (wd>4)
%  C2 = 1;
%  C3 = 0.6;
%else
%  C2 = 0.0013*wd^6-0.0211*wd^5+0.1156*wd^4-0.2434*wd^3+0.2267*wd^2-0.0713*wd;
%  C3 = 5e-5*wd^6 -0.001*wd^5+0.0084*wd^4-0.0332*wd^3+0.0341*wd^2-0.0092*wd+1;
%end
%
if wd<=1
  C2 = 0;
elseif (wd<=3)
  C2 = 0.0573*wd^3 - 0.2074*wd^2+0.2994*wd-0.15;
else
  C2 = 0.54*wd-1.19;
end
if wd<=1
  C3 = 1;
elseif (wd<=5)
  C3 = 0.0053*wd^3 - 0.0648*wd^2+0.1204*wd+0.9391;
else
  C3 = 0.58;
end
%
fsmax = tau * (1+C1*K^2)*(1+K*C2);
% @40 deg
%Fs_all = -52.612*K^6+186.3*K^5-243.12*K^4+124.62*K^3+4.2935*K^2-27.272*K+34.85;
% @45 deg
%Fs_all = -1.1689*K^6+41.756*K^5-92.897*K^4+53.707*K^3+19.277*K^2-28.05*K+34.924;
%Fs_all = 1000 * Fs_all
%
% general
Fs_all = 0.9*smax*(1+0.5*(sult/smax-1)^2)*(0.5 -((1-K)^3)*(1/sqrt(3)-0.5));
%
Msw = fsmax/Fs_all -1;
%-------------------------------------------------------------------------------
% UPRIGHT
% effective length
if (RP<1.5*H)
  Lu = Hu / sqrt(1+K^2*(3-2*RP/Hu));
else
  Lu = Hu;
end
% stress
sigmau = K * tau * tan(ALPHA) / (Aue/(RP*tw)+0.5*(1-K)); 
% buckling
Fcu = pi^2*E*(0.5*Lu/rhou)^(-2);
Fccu = sigmau*(1-K)*(0.7729-0.6366*RP/Hu) + sigmau;
Fccu_all = 32500 * (K^2*tu/tw)^(1/3);
Msu_cc = Fccu/Fccu_all -1;
if K==0
  Msu_cc = -1;
end
%-------------------------------------------------------------------------------
% CAP
%
% compression due to main bending
%sigma_b = abs(Mx)/He/Acap;
Jtot = 2* ( (2*Acap + Nstr*Astr)*(H/2)^2 + 1/12*tw*He^3 + (CHORD*tskin) * (H/2-tskin/2)^2 + 1/12*CHORD*tskin^3);
sigma_b = abs(Mx)*He/Jtot;
% compression due to shear
%sigma_f = K*abs(Ty)/(2*Acap*tan(ALPHA))
sigma_f = K * tau * cot(ALPHA) / (2*Acap/(H*tw) + 0.5*(1-K));
% crippling
B2c = B2 + T1/2;
%
Fcc1 = OEF_K*smax*(max(B1/2,T1)/min(B1/2,T1)*sqrt(smax/E))^(OEF_EXP) * B1 * T1;
Fcc2 = OEF_K*smax*(max(B2c,T2)/min(B2c,T2)*sqrt(smax/E))^(OEF_EXP) * B2c * T2;
% cut off values
if (B1/T1<8)
  Fcc1 = smax;
end
if (B2/T2<8)
  Fcc2 = smax;
end
Fcc = (Fcc1 + Fcc2) / (B2c*T2 + B1*T1);
% bending
Mmax = K*C3*tau*tw*RP^2*tan(ALPHA)/12;
fsb = Mmax*(B2+T1-Y)/Ix;
Mscap = ((sigma_b+sigma_f)/Fcc + (fsb/sult)) - 1;
%-------------------------------------------------------------------------------
% Store constraints
%
CIN = -1.*ones(20,1);
% WEB
CIN(1) = Msw;        % web diagonal tension
CIN(2) = tau / Fscr -5; % limit to fatigue
%CIN(XX) = K - sqrt(tw-0.012);  % avoid excessive wrinkling
% CAP
CIN(3) = Mscap;           % cap crippling
% B1 > 8T1
CIN(4) = 1 - (B1/8/T1);   % proportions of T
% B2 > 8T2
CIN(5) = 1 - (B2/8/T2);   % proportions of T
% B2 < B1 < 2.5 B2
%CIN(6) = B1/(2.5*B2) - 1;
%CIN(7) = B2/B1 - 1;
% yelding
CIN(6) = sigma_b/smax - 1;
% T2 > 0.6 tw
CIN(7) = 1 - (T2/0.6/tw); % dimensions
CIN(8) = T2/(3*tw) - 1; % dimensions
%
% stiffener constraints
% 0.2 < RP/Hc < 1
%CIN(XX) = RP/H - 1;     % dimensions
%CIN(XX) = 0.2*H/RP - 1; % dimensions
% TU > 0.6 tw
CIN(9) = 1 - (tu/0.6/tw); % dimensions
CIN(10) = tu/(3*tw) - 1; % dimensions
% BU > 8Tu
CIN(11) = 1 - (BU/8/tu);   % section
CIN(12) = sigmau/Fcu -1;   % buckling
CIN(13) = Msu_cc; % crippling
CIN(14) = wd/2.5 -1; % crippling
CIN(15) = sqrt(3)*tau/smax -1;
% web and section constraints
% 115 < hc/tw < 1500
%CIN(18) = (Hc / tw)/1500 - 1;
%CIN(19) = 115*(tw/Hc) - 1; 
%CIN(23) = 1 - tau / (2*Fscr);
CIN(16) = 1 - Aue/(0.05*RP*tw);
CIN(17) = Aue/(1.4*RP*tw) -1;
CIN(18) = 1- 2*Acap/(He*tw);
%CIN(XX) = 2*Acap/(8*He*tw) -1; this value can go to infinite NACA TN 2661, pag 111
%
CIN(19) = (B2+T1-Y)/(0.4*H) -1;
%
CIN(20) = 1 - tau / (1.1*Fscr);
%
%
if (~isempty(CINDEX))
  CIN = CIN(CINDEX);
end
if (PRINT)
  fid = 1;

  fprintf(fid, '\nShear index: %g', sqrt(abs(Ty)) / H);
  fprintf(fid, '\nEfficiency: %g',  abs(Ty) / (H*(tw + Au/RP))/1000);
  fprintf(fid, '\nH: %g', H);
  fprintf(fid, '\nHe: %g', He);
  fprintf(fid, '\nHu: %g', Hu);
  fprintf(fid, '\nHc: %g', Hc);

  fprintf(fid, '\nY cap: %g', Y);
  fprintf(fid, '\nAcap: %g', Acap);
  fprintf(fid, '\ncap flexibility: %g', wd);
  fprintf(fid, '\nt2/tw: %g', t2w);
  fprintf(fid, '\n2Acap/He/t: %g', 2*Acap/He/tw);

  fprintf(fid, '\nDiagonal tension factor: %g',  K);
  fprintf(fid, '\nAngle: %g',  ALPHA * 180/pi);
  fprintf(fid, '\nC1, C2, C3: %g, %g, %g', C1, C2, C3);

  fprintf(fid, '\ntau: %g',  tau);
  fprintf(fid, '\ntau_cr: %g', Fscr);
  fprintf(fid, '\ntau/tau_cr: %g', tau/Fscr);

  fprintf(fid, '\nKss: %g', Kss);
  fprintf(fid, '\nRu, Rf, Rlim: %g, %g, %g', Ru, Rf, Rlim);

  fprintf(fid, '\nMaximum shear: %g', fsmax);
  fprintf(fid, '\nMaximum shear allowed: %g', Fs_all);
  fprintf(fid, '\npitch: %g', RP);
  fprintf(fid, '\nAu/(d*t): %g', Au/RP/tw);
  fprintf(fid, '\nAue/(d*t): %g', Aue/RP/tw);
  fprintf(fid, '\nd/h: %g', RP/H);
  fprintf(fid, '\nIxu: %g', Ixu);
  fprintf(fid, '\neu: %g', eu);
  fprintf(fid, '\nrhou: %g', rhou);
  fprintf(fid, '\nAu, Aue, Aue/Au: %g, %g, %g', Au, Aue,Aue/Au);
  fprintf(fid, '\ntu/tw: %g', tuw);

end


