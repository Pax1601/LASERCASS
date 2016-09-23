function [M, G] = Zst_sk_mass_10(X, data)
% skin and stringers
ts = X(1);
As = X(2);
% rib
tr = X(3);
D1 = X(4);
D2 = X(5);
RP = X(6);
%
Ns = data.param.NS;
C = data.geo.c;
H = data.geo.h;
%
Arib = C*H;
c1 = C * pi/4;
Hr = 2/3 * H;
% smeared rib weight
Mrib_sme = (Arib * tr - c1 * D1 * D2 * Hr * tr)/RP;
%
M = 2*(Ns * As + ts*C) + Mrib_sme; 
G =[2*C; 2*Ns
   (C*H - (pi*C*D1*D2*H)/6)/RP
    -(pi*C*D2*H*tr)/(6*RP)
    -(pi*C*D1*H*tr)/(6*RP)
    -(C*H*tr - (pi*C*D1*D2*H*tr)/6)/RP^2];
