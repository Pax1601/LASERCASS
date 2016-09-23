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

function [CIN, CEQ] = cstr_Zst_sk_panel_10(X, data)
%
PRINT = 0;
CIN = [];
CEQ = [];
% skin, stringer area
ts = X(1);
As = X(2);
% rib
tr = X(3);
D1 = X(4);
D2 = X(5);
RP = X(6);
%
C = data.geo.c;
H = data.geo.h;
E = data.param.E;
Acap = data.geo.Acap;
tw = data.geo.tw;
%
NS = data.param.NS;
NSPAR = data.param.Nspar;
%
smax = data.param.smax;
Kas = data.param.KAs;
ds = data.param.ds;
RAPP1_MAX = data.param.tws_ts_ratio_max;
RAPP1_MIN = data.param.tws_ts_ratio_min;
N  = data.load.Mx/H;
Tx = data.load.Tx;
Ty = data.load.Ty;
Mx = data.load.Mx;
Mt = data.load.Mz;
CINDEX = data.param.Cindex;
%-------------------------------------------------------------------------------
% RIBS
% find shear fluxes
%
save('rib.mat')
% solve for fluxes along section
NU = 0.3;
G = E / (2 * (1 + NU));
lskin = C / (NS-1); 
% spar
Ncells = NSPAR -1;
% stringers + spar cap position
NODE = [[lskin.*[0:NS-1]', repmat(H, NS,1)]; [lskin.*[NS-1:-1:0]', repmat(0, NS,1)]];
% stringers + spar cap area
AREA = As .* ones(NS*2,1);
AREA(1) = AREA(1) + Acap; AREA(NS) = AREA(NS) + Acap; 
AREA(NS+1) = AREA(NS+1) + Acap;  AREA(end) = AREA(end) + Acap;
% total number of panels
np = (NS-1) * 2 + NSPAR;
% thickness
thickn = ts .* ones(np,1); thickn(1:NSPAR) = tw;
% connectivity
BETA = zeros(np, 2);
BETA(1,1) = 2*NS; BETA(1,2) = 1;
BETA(2,1) = NS+1; BETA(2,2) = NS;
% if multicell add internal webs and cap areas
if (NSPAR>2)
  offset = floor((NS-1) / Ncells) +1;
  AREA(offset.*[1:NSPAR-2]) = AREA(offset.*[1:NSPAR-2]) + Acap;
  AREA(offset.*[1:NSPAR-2]+NS) = AREA(offset.*[1:NSPAR-2]+NS) + Acap;
  BETA(3:3+NSPAR-3,1) = offset.*[1:NSPAR-2]+NS; BETA(3:3+NSPAR-3,2) = offset.*[1:NSPAR-2];
end
offset = NSPAR;
for n=1:NS-1
  BETA(offset+n,1) = n;
  BETA(offset+n,2) = n+1;
end
offset = offset+NS-1;
for n=1:NS-1
  BETA(offset+n,1) = n+NS;
  BETA(offset+n,2) = n+1+NS;
end
%
fluxes = solve_mono(NODE, AREA, BETA, thickn, G .* ones(np,1), Tx, Ty, Mt);
% rib stresses
qmax = max(abs(fluxes));
tau_rib = qmax / tr;
% web effective height
Hr = 2/3 * H;
%if (RP/Hr<=1)
%  ab = RP / Hr;
%  Kss = 2.865 * ab^2 + 0.787 * ab + 4.807;
%  Fscr_r = (Kss * pi^2/12/(1-NU^2)) * E * (tr/RP)^2;
%else
%  ab = Hr / RP;
%  Kss = 2.865 * ab^2 + 0.787 * ab + 4.807
%  Fscr_r = (Kss * pi^2/12/(1-NU^2)) * E * (tr/Hr)^2;
%end
%
% use diagrams reported in Niu (pag. 536) for collapse stress and correction Kr
%ab = Hr / tr;
%F0_r = -0.002917*ab^3 + 1.95 * ab^2 -451.3 * ab +  43120; % psi
%KSI2PA = 6894.757;
%F0_r = KSI2PA * F0_r % Pa
%[X,Y] = meshgrid([0.3:0.1:0.7]);
%Z = [0.78 0.76 0.75 0.68 0.53; 0.72 0.7 0.67 0.58 0.47; 0.64 0.62 0.56 0.5 0.4; ...
%     0.54 0.52 0.47 0.36 0.32; 0.44 0.42 0.36 0.3 0.22]';
%Kr = interp2(X,Y,Z, D1, D2)
%tau_all = Kr * F0_r
%
KSI2PA = 6894.757;
ab = Hr / tr;
if (ab<60)
  tau_h = -2.784036*ab + 3.9136144e+4;
else
  tau_h = -0.002917*ab^3 + 1.95 * ab^2 -451.3 * ab +  43120; % psi
end
tau_h = tau_h * KSI2PA;
br = D1 * Hr /D2; % hole spacing
ab = br / tr;
if (ab<60)
  tau_c = -2.784036*ab + 3.9136144e+4; % psi
else
  tau_c = -0.002917*ab^3 + 1.95 * ab^2 -451.3 * ab +  43120; % psi
end
tau_c = tau_c * KSI2PA;
% correction
% use average data
kav = (1-3.5*(Hr/1000/tr)^2);
% use conservative data
kall = (0.85 - 0.0006 * Hr/tr);
% collapse stress
tau_all = kall * ((tau_h*(1-D1^2) + tau_c*sqrt(D1)) * (1-D2));
%
% shear at vertical net web at holes
fs1 = tau_rib * (1/(1 - D1));
% shear at net web between holes
fs2 = tau_rib * (1/(1 - D2));
%
Msr1 = fs1 *sqrt(3)/ smax -1;
Msr2 = fs2 *sqrt(3)/ smax -1;
%
Msrall1 = tau_rib/tau_all-1;
%-------------------------------------------------------------------------------
% stresses
%
%
d  = sqrt(As/Kas);
% stringer moment of inertia
Js = 1/12 * 0.12 * d^4 + 2 * ( 0.12*d^2*0.56^2*d^2 + 1/12*0.12^3*d^4 ) + As *(H/2-ts-0.62*d)^2;
% overall moment of inertia
J = 2* ( Js*NS + (C*ts) * (H/2-ts/2)^2 + 1/12*C*ts^3) + 2*NSPAR* ( Acap*(H/2)^2 ) + NSPAR/12*tw*H^3;
% skin stress
sigma_sk = (H/2-ts/2)*abs(Mx)/J;
% stringer stress
sigma_st = (H/2-ts-0.62*d)*abs(Mx)/J;
%
%if (sigma_st < 0)
%  error('Center or gravity of skin-stringer panel is negative. Please reduce rpitch and spitch.');
%end
if (sigma_sk<eps) 
  sigma_sk = eps;
  sigma_st = eps;
end
%
Smp = sigma_sk/smax - 1;
%-------------------------------------------------------------------------------
% skin
%
% compression
a     = C / (NS-1);
t     = ds*d;
tb    = ts/a;
bt    = a/ts;
rapp1 = t/ts;
rapp2 = d/a;
% skin-stringer constraints for a feasible solution
FEAS1 = rapp1/RAPP1_MAX -1;
FEAS2 = 1 - rapp1/RAPP1_MIN;
%
sigma_mean = abs(Mx)/(C*(H-ts)*(ts*a+As)/a);

kc = ( 23.43*rapp1^2 -24.55*rapp1 +2.22)*rapp2^3 + ...
     (-17.92*rapp1^2 +16.54*rapp1 -1.48)*rapp2^2 + ...
     ( 3.11 *rapp1^2 -2.74 *rapp1 +0.24)*rapp2   + ...
     (-6.69 *rapp1^2 +13.17*rapp1 -1.24);

Rlp = (sigma_sk * 12 * (1-0.33^2))/(kc*(pi^2)*E*(tb^2));
%
% shear
%
ab=RP/a;
ks = -0.138*(ab^3)+1.4076*(ab^2)-4.6814*ab+10.467;
tau = max(abs(fluxes(NSPAR+1:end)./thickn(NSPAR+1:end)));
if (tau<eps) 
  tau = eps;
end

Rsp = (abs(tau) * 12 * (1-0.33^2))/((pi^2)*ks*E*(tb^2));
%
% combined 
%
Msp = 1-2/( Rlp + sqrt(Rlp^2+4*(Rsp^2)) + 0.001);
%-------------------------------------------------------------------------------
% spar
%
% web compression
%
%tb = tw/min(RP, H);
%ab = max(RP,H)/min(RP,H);
%ks = -0.138*(ab^3)+1.4076*(ab^2)-4.6814*ab+10.467;
%tau = abs(Mt)/(2*Omega*tw) + abs(Ty)/(2*H*tw) + 0.25*abs(Tx)/(2*C*tw);
%Rsw = (abs(tau) * 12 * (1-0.33^2))/((pi^2)*ks*E*(tb^2));
%
% web bending
%
%ab=RP/a;
%tb = tw/H;
%kb = 0.0012*(ab^4)-0.0646*(ab^3)+1.2644*(ab^2)-10.08*ab+61.983;
%sol = (sigma_sk * 12 * (1-0.33^2))/((pi^2)*kb*E*(tb^2));
%check = (tanh(ab-3)+1)/2;
%Rbw = check*sol;
%
% combined 
%
%Msw = 1-1/(sqrt(Rsw^2+Rbw^2) + 0.001);
%-------------------------------------------------------------------------------
% stringer
SAF_MARG = 1.5; % impose instability for ultimate loads
sigma_st = sigma_st * SAF_MARG;
%
% compression
%
W = 1.7*ts*sqrt(E/sigma_st);
CG=(-0.5*(ts^2)*W+0.2232*d^3)/(W*ts+0.36*d^2);
%CG = (-0.5*W*(ts^2)+(3/2+3*ds)*ds*(d^3))/(W*ts + 3 * ds * d^2);
%I = (1/12)*(W*(ts^3)+2*d*(ds*d^3)+ds*d^4) + ...
%    ts*W*((CG+0.5*ts)^2)+ds*(d^2)*(CG-ds/2*d)^2+ds*(d^2)*(CG-(0.5+ds)*d)^2 + ...
%    ds*d^2*(CG-(1+1.5*ds)*d)^2;
I = (1./12.)*(W*ts^3+2.*d*(0.12*d^3)+0.12*d^4)+ts*W*((CG+0.5*ts)^2)+...
    0.12*(d^2)*(CG-0.06*d)^2+0.12*(d^2)*(CG-0.62*d)^2 +0.12*d^2*(CG-1.18*d)^2;
%
radius = sqrt(I/As);
L = RP*0.7/radius;
instc = 1.e+03*(0.0053*L^4-0.399*L^3-42.83*L^2+235.84*L+353660)/sigma_st;
instc = 1.0-instc;
%
% skin stringer buckling
%
I0 = 0.0855*(d^4);
Z = 0.5*ts+0.62*d;
bs = a;
x = RP/bs;
k1 = 0.0041*(x^3)-0.0774*(x^2)+0.494*x-0.0153;
I = I0+((As*(Z^2))/(1+(As/(k1*bs*ts))));
D = E*(ts^3)/(12*(1-0.33^2));
y = E*I/(bs*D);
c1 = 4.4396-1.8146*x+0.0895*y + ...
     0.1708*x^2-0.0104*x*y+3.2538e-6*y^2;
c2 = 4.;
k2 = min(c1,c2);
Scr = k2*(pi^2)*D/((bs^2)*ts);
instglob = 1.0-Scr/sigma_sk;

%-------------------------------------------------------------------------------
%
% Store constraints
%
CIN = -1 .*ones(9,1);
CIN(1) = instglob;
CIN(2) = instc;
CIN(3) = Smp;
CIN(4) = Msp;
CIN(5) = FEAS1;
%CIN(6) = FEAS2;
% rib
CIN(7) = Msr1;
CIN(8) = Msr2;
CIN(9) = Msrall1;
%
CIN(10) = 1 - (H/2-ts-0.62*d)/(0.5*H/2);
if (~isempty(CINDEX))
  CIN = CIN(CINDEX);
end

if (PRINT)
  fid = 1;

  fprintf(fid, '\n');
  fprintf(fid, '\nSKIN:');
  fprintf(fid, '\n\nsigma: %g', sigma_sk);
  fprintf(fid, '\ntau: %g', tau);
  fprintf(fid, '\ntskin: %g', ts);
  fprintf(fid, '\nbay bs: %g', a);
  fprintf(fid, '\neffective width be: %g', W);
  fprintf(fid, '\nwidth ratio bs/be: %g', bs/W);


  fprintf(fid, '\n\nSTRINGER:');
  fprintf(fid, '\n\nsigma: %g', sigma_st);
  fprintf(fid, '\ntw/tskin: %g', rapp1);
  fprintf(fid, '\ntw: %g', 0.12*d);
  fprintf(fid, '\nhw: %g', d);
  fprintf(fid, '\nStiffening ratio Astr/Askin: %g', As/(ts*a));
  fprintf(fid, '\nWing box half height: %g', H/2);
  fprintf(fid, '\nSkin stringer CG: %g', H/2-ts-0.62*d);
  fprintf(fid, '\nSkin stringer CG/half height: %g', (H/2-ts-0.62*d)/(H/2));

  fprintf(fid, '\n\nRIB:');
  fprintf(fid, '\n\nPitch: %g', RP);
  fprintf(fid, '\ntr: %g', tr);
  fprintf(fid, '\nD/h: %g', D1);
  fprintf(fid, '\nD/b: %g', D2);
  fprintf(fid, '\nWing box chord: %g', C);
  fprintf(fid, '\nHole pitch: %g', br);
  fprintf(fid, '\nHole Diameter: %g', D1*Hr);
  fprintf(fid, '\nCorrection factor: %g', tau_all / tau_h);
  fprintf(fid, '\nCollapse stress: %g',  tau_all);
  fprintf(fid, '\nShear nominal: %g',  tau_rib);
  fprintf(fid, '\nShear at holes: %g',  fs1);
  fprintf(fid, '\nShear between holes: %g',  fs2);
  fprintf(fid, '\nShear yeld: %g',  smax/sqrt(3));

end
