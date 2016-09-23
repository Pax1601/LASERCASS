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
%     121019      2.1.473   L.Riccobene      Creation
%     121116      2.2.497   L.Cavagna        Rewritten
%
%**************************************************************************
%
% function     [MAC, XLEMAC, XACMAC] = RecoverMACInfoFromBeamModel(beam_model)
%
%
%   DESCRIPTION: Given the beam model returns mean aerodynamic chord,
%                its leading edge x-coordinate with respect aircraft nose and
%                aerodynamic centre x-coordinate w.r.t. aircraft nose. 
%
%         2UT: NAME           TYPE       DESCRIPTION
%
%                beam_model     struct     beam model database
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                MAC            double     mean aerodynamic chord [m]
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
function [MAC, MAC_LE, MAC_AC] = RecoverMACInfoFromBeamModel(BEAM)
%
  MAC = 0;
  MAC_LE = 0;
  MAC_AC = 0;
  % wing CAERO
  AERO = BEAM.Aero;
  index = find(AERO.ID<250);
  np = length(index);
  %
  if (~isempty(index))
    geo = AERO.geo;
    MACP = zeros(np,1);
    AREAP = zeros(np,1);
    XLE = zeros(np,1);
    XAC = zeros(np,1);
%   modify carrythrough patch
    STARTP = [geo.startx(index(2)),geo.starty(index(2)),geo.startz(index(2))];
    [P2, C] = prolong_patch(STARTP, geo.c(index(2)), geo.b(index(2)), geo.T(index(2)), ...
              geo.SW(index(2)), geo.TW(index(2),1,1), geo.dihed(index(2)), geo.b(index(1)));
    geo.c(index(1)) = C;
    geo.SW(index(1)) = geo.SW(index(2));
    geo.T(index(1)) = geo.c(index(2)) / C;
    geo.startx(index(1)) = P2(1);
    for i=1:np
      p = index(i);
      [MACP(i), AREAP(i), XLE(i)] = MAC_patch(geo.c(p), geo.T(p), geo.SW(p), ...
                                               geo.b(p), geo.dihed(p));
      XAC(i) = 1.25*XLE(i) + geo.startx(p);
      XLE(i) = XLE(i) + geo.startx(p);
    end
  % area weighted results
    AREATOT = sum(AREAP);
    MAC     = dot(MACP, AREAP)/ AREATOT; 
    MAC_LE  = dot(XLE, AREAP) / AREATOT; 
    MAC_AC  = dot(XAC, AREAP) / AREATOT; 
  %
  end
end
%-------------------------------------------------------
function [MAC, A, xLE] = MAC_patch(CROOT, TP, SW, SPAN, DIHED)
  % patch MAC
  MAC = (2/3)*(1+TP+TP^2)*CROOT/(1+TP);
  % patch projected AREA
  A = CROOT*(1+TP)*SPAN*cos(DIHED)/2;
  % patch AR
  AR = 2*SPAN/(CROOT*(1+TP));
  % AR * tan(SW) along c/4
  ARTAN4 = AR * tan(SW);
  % AR * tan(SW) along leading edge
  ARTAN0 = ARTAN4 + (1-TP)/(1+TP);
  % MAC LE
  xLE = MAC * ((1+TP)*(1+2*TP)/(8*(1+TP+TP^2)))*ARTAN0;
end