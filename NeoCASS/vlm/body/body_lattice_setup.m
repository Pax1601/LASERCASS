%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%**************************************************************************
%*****
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%*****
function BAERO = body_lattice_setup(BAERO, COORD)

nbaero = length(BAERO.ID);
% collocation point angular position
COLLOCT = D2R([0, 120, -120]); ST = sin(COLLOCT); CT = cos(COLLOCT);
NP = 20;
THETA = [0:2*pi/NP:2*pi];
nt = length(THETA);
%
for i=1:nbaero
%
  if (BAERO.CP(i))
    Rmat = COORD.R(:,:,BAERO.CP(i));
    ROT = 1;
  else
    ROT = 0;
    Rmat = eye(3);
  end
  ne = BAERO.geo.Nelem(i);
  dL = BAERO.geo.L(i)/ne;
% mesh element
  X = zeros(ne+1,1);
  X(end) = BAERO.geo.L(i);
  X(2:end) = [dL:dL:BAERO.geo.L(i)];
  Xnode = (X(1:end-1) + X(2:end))/2;
% input geometry data
  Xgeo = BAERO.geo.fs{i} .* BAERO.geo.L(i);
% interpolate radius
  R = interp1(Xgeo, BAERO.geo.Rs{i}, Xnode, 'linear', 'extrap');
% radius derivative along x axis
  PP = interp1([0; Xnode], [0; R],'linear','pp');
%
  BAERO.geo.x{i}  = Xnode; % node coordinates
  BAERO.geo.R{i}  = R; % radius 
  BAERO.geo.Rx{i} = PP.coefs(:,1); % radius derivative
  nrad = length(R);
  ns = nrad*3;
  BCOLLOC = zeros(ns ,3); BNORM = ones(ns,3);
  BCOLLOC(:,1) = repmat(BAERO.geo.x{i},3,1);
  BCOLLOC(:,2) = [BAERO.geo.R{i}.*ST(1); BAERO.geo.R{i}.*ST(2); BAERO.geo.R{i}.*ST(3)];
  BCOLLOC(:,3) = [BAERO.geo.R{i}.*CT(1); BAERO.geo.R{i}.*CT(2); BAERO.geo.R{i}.*CT(3)];
%
  BNORM(:,1) = repmat(-BAERO.geo.Rx{i},3,1);
  BNORM(:,2) = [repmat(ST(1),nrad,1); repmat(ST(2),nrad,1); repmat(ST(3),nrad,1)];
  BNORM(:,3) = [repmat(CT(1),nrad,1); repmat(CT(2),nrad,1); repmat(CT(3),nrad,1)];
  BNORM = BNORM./repmat(sqrt(dot(BNORM,BNORM,2)),1,3);
% close body points for bc
%  BNORM(1:nrad:end,1) = -1.0;
%  BNORM(1:nrad:end,2) = 0.0;
%  BNORM(1:nrad:end,3) = 0.0;
%  BNORM(nrad:nrad:end,1) = 1.0;
%  BNORM(nrad:nrad:end,2) = 0.0;
%  BNORM(nrad:nrad:end,3) = 0.0;
%
  for k=1:ns
    BCOLLOC(k,:) = (Rmat*BCOLLOC(k,:)')' + BAERO.geo.ref_point(i,:);
    BNORM(k,:) = (Rmat*BNORM(k,:)')';
  end
% panels
  xgeo = BAERO.geo.x{i};
  ns = length(xgeo)-2;
  R = BAERO.geo.R{i}(2:end-1);
  NODE = []; CONN = [];
  NODE(:,1) = repmat(xgeo(2:end-1), nt,1);
  offset = 0; p=0;
  for k=1:nt-1
    NODE([(k-1)*ns+1:ns*k],2) = R .* sin(THETA(k));
    NODE([(k-1)*ns+1:ns*k],3) = R .* cos(THETA(k));
    offset = (k-1)*ns; 
  end

  for j=1:ns-1
    for k=1:nt-2
      offset = (k-1)*ns; 
      p = p+1;
      CONN(p,1) = j    + offset;
      CONN(p,4) = j+1  + offset;
      CONN(p,3) = j+1  + k*ns;
      CONN(p,2) = j    + k*ns;
    end
    k = nt-1;
    offset = (k-1)*ns; 
    p = p+1;
    CONN(p,1) = j    + offset;
    CONN(p,4) = j+1  + offset;
    CONN(p,3) = j+1;
    CONN(p,2) = j;
  end
%
%  k = nt-1;
%  offset = (k-1)*ns; 
%  for j=1:ns-1
%    p = p+1;
%    CONN(p,1) = j    + offset;
%    CONN(p,4) = j+1  + offset;
%    CONN(p,3) = j+1;
%    CONN(p,2) = j;
%  end
  NODE(end+1,1) = xgeo(1);
  NODE(end+1,1) = xgeo(end);
  nn = size(NODE,1);
% add radome
  pr = 0;
  for k=1:nt-2
    offset = (k-1)*ns; 
    pr = pr+1;
    CONNR(pr,1) = nn-1;
    CONNR(pr,4) = 1  + offset;
    CONNR(pr,3) = 1  + k*ns;
    CONNR(pr,2) = nn-1;
  end
  k=nt-1;
  offset = (k-1)*ns; 
  pr = pr+1;
  CONNR(pr,1) = nn-1;
  CONNR(pr,4) = 1  + offset;
  CONNR(pr,3) = 1;
  CONNR(pr,2) = nn-1;

% add tailcone
  j = ns-1;
  for k=1:nt-2
    offset = (k-1)*ns; 
    p = p+1;
    CONN(p,1) = nn;
    CONN(p,4) = j+1  + offset;
    CONN(p,3) = j+1+ k*ns;
    CONN(p,2) = nn;
  end
  k=nt-1;
  offset = (k-1)*ns; 
  p = p+1;
  CONN(p,1) = nn;
  CONN(p,4) = j+1  + offset;
  CONN(p,3) = j+1;
  CONN(p,2) = nn;
%
  CONN = [CONNR; CONN];
%
  NODE_LOC = NODE;
  if ROT==1
    for k=1:nn
      NODE(k,:) = (Rmat * NODE(k,:)')';
    end
  end
  NODE = NODE + repmat(BAERO.geo.ref_point(i,:), nn, 1);
%
% element centroid
  ne = size(CONN,1);
  ELCENTR = zeros(ne, 3);
  ELNORM = zeros(ne, 3);
  ELAREA = zeros(ne, 1);
  ELRAD = zeros(ne, 1);
  THETA = zeros(ne, 1);
%
  for k=1:pr
    ELCENTR(k,:) = sum(NODE_LOC(CONN(k,2:4),:),1)./3;
    P1 = NODE_LOC(CONN(k,3),:) - NODE_LOC(CONN(k,1),:);
    P2 = NODE_LOC(CONN(k,4),:) - NODE_LOC(CONN(k,1),:);
    ELNORM(k,:) = cross(P1, P2);
    AREA(k) = norm(ELNORM(k,:));
  end
  for k=(ne-pr)+1:ne
    ELCENTR(k,:) = sum(NODE_LOC(CONN(k,2:4),:),1)./3;
    P1 = NODE_LOC(CONN(k,3),:) - NODE_LOC(CONN(k,1),:);
    P2 = NODE_LOC(CONN(k,4),:) - NODE_LOC(CONN(k,1),:);
    ELNORM(k,:) = cross(P1, P2);
    AREA(k) = norm(ELNORM(k,:));
  end
  for k=pr+1:(ne-pr)
    ELCENTR(k,:) = mean(NODE_LOC(CONN(k,:),:),1);
    P1 = NODE_LOC(CONN(k,3),:) - NODE_LOC(CONN(k,1),:);
    P2 = NODE_LOC(CONN(k,4),:) - NODE_LOC(CONN(k,2),:);
    ELNORM(k,:) = cross(P1, P2);
    AREA(k) = norm(ELNORM(k,:));
  end
% normalize normals
  ELNORM = ELNORM./ repmat(sqrt(dot(ELNORM,ELNORM,2)),1,3);
% panel area
  AREA = AREA./2;
  ELRAD = sqrt(dot(ELCENTR(:,2:3),ELCENTR(:,2:3),2));
  index_left = find(ELCENTR(:,2)>=0);
  index_right = find(ELCENTR(:,2)<0);
  THETA(index_left) = acos(ELCENTR(index_left,3)./ELRAD(index_left));
  THETA(index_right) = -acos(ELCENTR(index_right,3)./ELRAD(index_right))+2*pi;
% save data
  BAERO.Rmat(:,:,i) = Rmat;       % body frame
  BAERO.lattice.Elem.Node{i} = NODE;   % body node coords
  BAERO.lattice.Elem.Node_loc{i} = NODE_LOC;   % body node coords in body frame
  BAERO.lattice.Elem.Midpoint_loc{i} = ELCENTR;   % panel midopint in body frame
  BAERO.lattice.Elem.Conn{i} = CONN;   % element connectivity
  BAERO.lattice.Elem.Area{i} = AREA(1:NP:end);   % element area
  BAERO.lattice.Elem.Norm{i} = ELNORM; % element normals pointing into body
%  BAERO.lattice.Elem.R{i} = ELRAD(1:NP:end);     % element centers radius
  BAERO.lattice.Elem.R{i}= interp1(Xgeo, BAERO.geo.Rs{i}, BAERO.lattice.Elem.Midpoint_loc{i}(1:NP:end,1), 'linear', 'extrap');
  BAERO.lattice.Elem.Theta{i} = THETA(1:NP); % element centers azimuth
% collocation points and normals in main reference frame
  BAERO.lattice.COLLOC{i} = BCOLLOC;
  BAERO.lattice.N{i} = BNORM;
% collocation point angular position
  BAERO.COLLOCT = COLLOCT;
%
end


