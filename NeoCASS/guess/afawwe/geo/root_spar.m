%
% Determine spar chord % at fuselage-wing intersection
%
function [FSPAN, RSPAN] = root_spar(CR, CK, C, etaFR, etaRR, etaFK, etaRK, DIST, SPAN, LAMBDA)

  FSPAN = (etaFR * CR + DIST*(tan(LAMBDA)*SPAN + etaFK*CK - etaFR*CR)/SPAN - tan(LAMBDA)*DIST)/C;
  RSPAN = (etaRR * CR + DIST*(tan(LAMBDA)*SPAN + etaRK*CK - etaRR*CR)/SPAN - tan(LAMBDA)*DIST)/C;


 