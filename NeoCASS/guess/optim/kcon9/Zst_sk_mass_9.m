function [M, G] = Zst_sk_mass_9(X, data)

ts = X(1);
tw = X(2);
As = X(3);
Ns = data.param.NS;
C = data.geo.c;
H = data.geo.h;
%
M = 2*(Ns * As + ts*C + tw*H);
G = [2*C; 2*H; 2*Ns];
