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

function [CIN, CEQ] = cstr_Zst_sk_panel_9(X, data)
%
CIN = [];
CEQ = [];

ts = X(1);
tw = X(2);
As = X(3);

C = data.geo.c;
H = data.geo.h;
E = data.param.E;
NS = data.param.NS;
RP = data.param.RP;
%
smax = data.param.smax;
Kas = data.param.KAs;
ds = data.param.ds;
ts_t_max = data.param.ts_t_max;
ts_t_min = data.param.ts_t_min;
%N  = data.load.Mx/H;
N  = data.load.N;
Tx = data.load.Tx;
Ty = data.load.Ty;
Mx = data.load.Mx;
Mt = data.load.Mz;
CINDEX = data.param.Cindex;
d  = sqrt(As/Kas);
% stringer moment of inertia
Js = 1/12 * 0.12 * d^4 + 2 * ( 0.12*d^2*0.56^2*d^2 + 1/12*0.12^3*d^4 ) + As *(H/2-ts-0.62*d)^2;
% overall moment of inertia
J = 2* ( Js*NS + (C*ts) * (H/2-ts/2)^2 + 1/12*C*ts^3 + 1/12*tw*H^3 );
% overall area
AREA = (C*ts + H*tw + As*NS)*2;
% skin stress due to bending and axis load
sigma_sk = (H/2-ts/2)*abs(Mx)/J;% - N/AREA;
% stringer stress due to bending and axis load
sigma_st = (H/2-ts-0.62*d)*abs(Mx)/J;% - N/AREA;
%
%if (sigma_st < 0)
%  fprintf('\n### Warning: center or gravity of skin-stringer not feasible. Please reduce rpitch and spitch.');
%  fprintf('\n             Skin thickness: %g.', ts);
%  fprintf('\n             Stringer CG:    %g.', 0.62*d);
%  fprintf('\n             Box half depth: %g.', H/2);
%  fprintf('\n             Skin-stringer CG: %g.', H/2-ts-0.62*d);
%end
if (sigma_sk<eps) 
  sigma_sk = eps;
  sigma_st = eps;
end
%
Smp = sigma_sk/smax - 1;
Omega = C*H;
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
tau = abs(Mt)/(2*Omega*ts) + abs(Tx)/(2*C*ts) + 0.5*abs(Ty)/(2*H*ts);
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
tb = tw/min(RP, H);
ab = max(RP,H)/min(RP,H);
ks = -0.138*(ab^3)+1.4076*(ab^2)-4.6814*ab+10.467;
tau = abs(Mt)/(2*Omega*tw) + abs(Ty)/(2*H*tw) + 0.25*abs(Tx)/(2*C*tw);
Rsw = (abs(tau) * 12 * (1-0.33^2))/((pi^2)*ks*E*(tb^2));
%
% web bending
%
ab=RP/a;
tb = tw/H;
kb = 0.0012*(ab^4)-0.0646*(ab^3)+1.2644*(ab^2)-10.08*ab+61.983;
sol = (sigma_sk * 12 * (1-0.33^2))/((pi^2)*kb*E*(tb^2));
check = (tanh(ab-3)+1)/2;
Rbw = check*sol;
%
% combined 
%
Msw = 1-1/(sqrt(Rsw^2+Rbw^2) + 0.001);
%-------------------------------------------------------------------------------
% stringer
SAF_MARG = 1.5; % impose instability for ultimate loads
sigma_st = sigma_st * SAF_MARG;
%
% compression
%
W = 1.7*ts*sqrt(E/sigma_st);
CG=(-0.5*ts^2*W+0.2232*d^3)/(W*ts+0.36*d^2);
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
instglob = 1.0-Scr/sigma_st;
%-------------------------------------------------------------------------------
% skin-stringer constraints for a feasbile solution
t     = ds * d;
FEAS1 = ts - ts_t_max * t;
FEAS2 = ts_t_min * t - ts;
%-------------------------------------------------------------------------------
%
% Store constraints
%
CIN(1) = instglob;
CIN(2) = instc;
CIN(3) = Smp;
CIN(4) = Msp;
CIN(5) = Msw;
CIN(6) = FEAS1;
CIN(7) = FEAS2;
CIN(8) = 1-(H/2-ts-0.62*d)/(0.2*H/2);
%
if (~isempty(CINDEX))
  CIN = CIN(CINDEX);
end
