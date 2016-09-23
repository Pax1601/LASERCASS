function [As_end, I1, I2, J, K1, K2] = Prop_Sec_9(nct, Zs, tbs, skin, web)

  Nsec = length(Zs);
  As_end = zeros(Nsec-nct,1);
  I1 = zeros(Nsec,1);
  I2 = zeros(Nsec,1);
  J = zeros(Nsec,1);
  K1 = zeros(Nsec,1);
  K2 = zeros(Nsec,1);
  % Compute structural areas at BAR ends (BAR number + 1, only cantilever part)
  As_end = 2.*(Zs(nct+1:end)).*skin.tskin(nct+1:end) +...
      2*tbs(nct+1:end).*web.tw(nct+1:end)+...
      2*skin.Astr(nct+1:end).*skin.Nstr(nct+1:end);
  % hor. plane bend
  I1 = 2*Zs.*(skin.tskin).*(tbs./2).^2 + 2*1/12.*web.tw.*tbs.^3 + ...
                2.*(skin.Astr.*skin.Nstr).*(tbs./2).^2;
  % vert. plane bend
  I2 = 2*1/12.*(skin.tskin + (skin.Astr.*skin.Nstr)./Zs).*Zs.^3 + 2*tbs.*web.tw.*(Zs./2).^2;
  % torsion
  J  = 2 .*(Zs.*tbs).^2. ./ (tbs./web.tw + Zs./skin.tskin);
end
