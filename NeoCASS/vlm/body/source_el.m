function A = source_el(COORDS, COORDC, NORMC, PG)
  % source points
  ns = size(COORDS,1);
  % collocation points
  nc = size(COORDC,1);
  A = zeros(nc, ns);
%
  COORDS(:,2:3) = COORDS(:,2:3).*PG;
  COORDC(:,2:3) = COORDC(:,2:3).*PG;
  for i=1:nc % loop on collocation points
    for j=1:ns % source loop
      dij = COORDC(i,:) - COORDS(j,:);
      dij3 = norm(dij)^3;
      dij(2:3) = dij(2:3).*PG;
      A(i,j) = dot(NORMC(i,:), dij./dij3);
    end
  end 
%
end
