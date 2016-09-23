function [M, G] = strut_mass_8(X, data)
%
ts = X(1);
R = data.geo.R;
r = R - ts;
M = pi * (R^2 - r^2);
G = pi * (2*R - 2*ts);
