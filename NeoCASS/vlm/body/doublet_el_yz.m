
function [Y,Z] = doublet_el_yz(COORDS, COORDC, NORMC, PG)
  % source points
  ns = size(COORDS,1);
  % collocation points
  nc = size(COORDC,1);
%
  PG2 = PG^2;
  V = zeros(1,3);
  dij = zeros(1,3);
%
  Y = zeros(nc, ns);
  Z = zeros(nc, ns);
%
  for i=1:nc % loop on collocation points
    for j=1:ns  % source loop
      dij(1) = COORDC(i,1) - COORDS(j,1);
      dij(2) = PG*(COORDC(i,2) - COORDS(j,2));
      dij(3) = PG*(COORDC(i,3) - COORDS(j,3));
      nn = norm(dij);
      dij3 = nn^3;
      dij5 = nn^5;
% Y
      c1 = 3*PG2*(COORDC(i,2) - COORDS(j,2));
      V(1) = c1 * (COORDC(i,1) - COORDS(j,1))/dij5;
      V(2) = c1 * PG2*(COORDC(i,2) - COORDS(j,2))/dij5 - PG2/dij3;
      V(3) = c1 * PG2*(COORDC(i,3) - COORDS(j,3))/dij5;
      Y(i,j) = dot(NORMC(i,:), V);
% Z
      c1 = 3*PG2*(COORDC(i,3) - COORDS(j,3));
      V(1) = c1 * (COORDC(i,1) - COORDS(j,1))/dij5;
      V(2) = c1 * PG2*(COORDC(i,2) - COORDS(j,2))/dij5;
      V(3) = c1 * PG2*(COORDC(i,3) - COORDS(j,3))/dij5 - PG2/dij3;
      Z(i,j) = dot(NORMC(i,:), V);
    end
  end  
end
%
