function [As_end, I1, I2, J, K1, K2] = Prop_Sec_kcon10(nct, Zs, tbs, web, skin, NSPAR)

  Nsec = length(Zs);
  As_end = zeros(Nsec-nct,1);
  I1 = zeros(Nsec,1);
  I2 = zeros(Nsec,1);
  J = zeros(Nsec,1);
  K1 = zeros(Nsec,1);
  K2 = zeros(Nsec,1);
  % Compute structural areas at BAR ends (BAR number + 1, only cantilever part)
  As_end = 2.*(Zs(nct+1:end)).*skin.tskin(nct+1:end) +...
      2*skin.Astr(nct+1:end).*skin.Nstr(nct+1:end) + 2*NSPAR*web.Acap(nct+1:end) + NSPAR*tbs(nct+1:end).*web.tw(nct+1:end);

  % hor. plane bend
  I1 = 2*Zs.*(skin.tskin).*(tbs./2).^2 + ...
                2.*NSPAR.*(skin.Astr.*skin.Nstr + 2.*web.Acap).*(tbs./2).^2 + NSPAR*1/12.*web.tw.*tbs.^3;
  % vert. plane bend
  I2 = 2*1/12.*(skin.tskin + (skin.Astr.*skin.Nstr+NSPAR*web.Acap)./Zs).*Zs.^3 + NSPAR*tbs.*web.tw.*(Zs./2).^2;
  % torsional stiffness
  J = jstiff_10(Zs, tbs, web, skin, NSPAR);  
end

function J = jstiff_10(Zs, tbs, web, skin, NSPAR)

  Nsec = length(Zs);
  for k=1:Nsec
    NS = skin.Nstr(k);
    As = skin.Astr(k);
    ts = skin.tskin(k);
    Acap = web.Acap(k);
    tw = web.tw(k);
    lskin = Zs(k) / (NS-1); 
    H = tbs(k);
    % spar
    Ncells = NSPAR -1;
    % stringers + spar cap position
    NODE = [[lskin.*[0:NS-1]', repmat(H, NS,1)]; [lskin.*[NS-1:-1:0]', repmat(0, NS,1)]];
    % stringers + spar cap area
    AREA = As .* ones(NS*2,1);
    AREA(1) = AREA(1) + Acap; AREA(NS) = AREA(NS) + Acap; 
    AREA(NS+1) = AREA(NS+1) + Acap;  AREA(end) = AREA(end) + Acap;
    % total number of panels
    np = (NS-1) * 2 + NSPAR;
    % thickness
    thickn = ts .* ones(np,1); thickn(1:NSPAR) = tw;
    % connectivity
    BETA = zeros(np, 2);
    BETA(1,1) = 2*NS; BETA(1,2) = 1;
    BETA(2,1) = NS+1; BETA(2,2) = NS;
    % if multicell add internal webs and cap areas
    if (NSPAR>2)
      offset = floor((NS-1) / Ncells) +1;
      AREA(offset.*[1:NSPAR-2]) = AREA(offset.*[1:NSPAR-2]) + Acap;
      AREA(offset.*[1:NSPAR-2]+NS) = AREA(offset.*[1:NSPAR-2]+NS) + Acap;
      BETA(3:3+NSPAR-3,1) = offset.*[1:NSPAR-2]+NS; BETA(3:3+NSPAR-3,2) = offset.*[1:NSPAR-2];
    end
    offset = NSPAR;
    for n=1:NS-1
      BETA(offset+n,1) = n;
      BETA(offset+n,2) = n+1;
    end
    offset = offset+NS-1;
    for n=1:NS-1
      BETA(offset+n,1) = n+NS;
      BETA(offset+n,2) = n+1+NS;
    end
  %
    [~, ~, ~, ~, ~, ~, ~, ~, J(k)] = smonoq_beam_prop(NODE, AREA, BETA, thickn, ones(np,1), 1);
  end
end