function [xc, yc, R] = ComputeFuselageXSecRadius(a0, a1, b1, N, xi, d)
%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     120508      2.237   L.Riccobene      Creation
%
%**************************************************************************
%
% function       [xc, yc, R] = ComputeFuselageXSecRadius(a0, a1, b1, N, xi, d)
%
%
%   DESCRIPTION:   Starting from Fourier coefficient used to approximate
%                  fuselage section compute radius over 360 degrees.
%                  Function called by ComputeCabinDimension.
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%
%
%    REFERENCES:
%
%**************************************************************************
%

% Compute radius from Fourier expansion
radius = @(x,a,b,c) a + b*cos(2*x) + c*sin(x);
phi    = 0:2*pi/N:2*pi;

% Radius and centre coordinates
R  = radius(phi, a0, a1, b1);
xc = 0;
yc = xi*d - d/2;