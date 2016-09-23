function [As_end, I1, I2, J, K1, K2] = Prop_Sec_8(nct, tbs, skin)

  Nsec = length(tbs);
  As_end = zeros(Nsec-nct,1);
  I1 = zeros(Nsec,1);
  I2 = zeros(Nsec,1);
  J = zeros(Nsec,1);
  K1 = zeros(Nsec,1);
  K2 = zeros(Nsec,1);
  r = tbs - skin.tskin; 
  % Compute structural areas at BAR ends (BAR number + 1, only cantilever part)
  As_end = pi .* (tbs(nct+1:end).^2 - r(nct+1:end).^2);
  % hor. plane bend
  I1 = 0.25 * pi * (tbs.^4 - r.^4);
  % vert. plane bend
  I2 = I1;
  % torsion
  J  = 2 * I1;
end
