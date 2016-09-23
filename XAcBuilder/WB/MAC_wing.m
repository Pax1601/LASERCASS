function [MAC, YMAC, XLEMAC, XACMAC] = MAC_wing(W, FUSEL)
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci          <ricci@aero.polimi.it>
%                      Luca Cavagna          <cavagna@aero.polimi.it>
%                      Luca Riccobene        <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS      PROGRAMMER       DESCRIPTION
%     171912      2.1.416   L.Riccobene      Creation
%
%**************************************************************************
%
% function     [MAC, YMAC, XLEMAC, XACMAC] = MAC_wing(W, FUSEL)
%
%
%   DESCRIPTION:  Given wing1 geo struct and fuselage length, returns mean
%                 aerodynamic chord, its spanwise position with respect
%                 symmetry plane x-z, its leading edge x-coordinate with
%                 respect aircraft nose and subsonic aerodynamic centre
%                 x-coordinate w.r.t. aircraft nose.
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                W              struct     stores wing geometrical data
%                                          coming from AcBuilder xml model
%
%                FUSEL          double     total fuselage length
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                MAC            double     mean aerodynamic chord [m]
%
%                YMAC           double     MAC spanwise position w.r.t.
%                                          aircraft centerline [m] 
%
%                XLEMAC         double     MAC leading edge x-coordinate
%                                          w.r.t. aircraft nose [m] 
%
%                XACMAC         double     MAC aerodynamic centre
%                                          x-coordinate w.r.t aircraft nose [m]
%
%    REFERENCES:
%
%**************************************************************************

% Recover geometric definition
tp = [W.taper_kink1; W.taper_kink2; W.taper_tip];  % taper vector [-]
Sw = W.area;                                       % wing area [m2]
h  = W.Span_matrix_partition_in_mid_outboard;      % trapezoid height vector [m]
L  = [W.LE_sweep_inboard; W.LE_sweep_midboard; W.LE_sweep_outboard]; % leading edge sweep vector [deg]

% Compute root chord (on simmetry plane, fuselage centerline)
Nh  = numel(h);
tpv = ones(Nh, 1).*(1+tp(1));
for k = 2:Nh,
   tpv(k) = tp(k-1) + tp(k); 
end
cr  = Sw/sum(h.*tpv);

% Chord, area, mac, y_mac vectors
%
% Remember that chord_kink1 = taper1*chord_root, chord_kink2 =
% taper2*chord_root etc...
crv   = cr.*[1; tp];
crv   = crv(1:end-1);
Swv   = cr.*(0.5.*h).*tpv;
macv  = (2/3).*crv.*((1 + tp + tp.^2)./(1 + tp));
ymacv = (h./6).*((1 + 2*tp)./(1 + tp));

% Compute wing MAC (weighted average)
MAC  = sum(macv.*Swv)/(Sw/2);
YMAC = sum(ymacv.*Swv)/(Sw/2); 

% Compute MAC leading edge and CA (subsonic case) x-coordinate w.r.t.
% aircraft nose
%
xler  = W.apex_locale.*FUSEL;
xbar  = ymacv.*tan(L.*pi/180);
xcbar = [0; cumsum(h.*tan(L.*pi/180))];
xmacv = xler + xcbar(1:end-1) + xbar;
%
XLEMAC = sum(xmacv.*Swv)/(Sw/2);
XACMAC = XLEMAC + .25*MAC; 