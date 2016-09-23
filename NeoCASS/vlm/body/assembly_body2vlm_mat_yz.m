function [Sij, DijY, DijZ, IGN] = assembly_body2vlm_mat_yz(nbody, lattice, BAERO, state, PG)
%
COLLOCVLM = [];
NORMVLM =[];
%
Rmat = BAERO.Rmat(:,:,nbody);
%
nvlm = size(lattice.COLLOC,1);
COLLOCVLM = zeros(nvlm, 3);
NORMVLM = zeros(nvlm, 3);
% rotate VLM mesh to body reference frame
for k=1:nvlm
  COLLOCVLM(k,:) = (Rmat'*(lattice.COLLOC(k,:) -  BAERO.geo.ref_point(nbody,:))')';
  NORMVLM(k,:) = (Rmat'* lattice.N(k,:)')';
end
ns = length(BAERO.geo.x{nbody});
ORIGIN = [BAERO.geo.x{nbody}, zeros(ns,2)];
COLLOC = COLLOCVLM;
NORM = NORMVLM;
[DijY, DijZ] = doublet_el_yz(ORIGIN, COLLOC, NORM, PG);
Sij = source_el(ORIGIN, COLLOC, NORM, PG);
% find ignored panels
IGN = [];
for k=1:nvlm
  R = norm(COLLOCVLM(k,2:3));
  Rint = interp1(BAERO.geo.x{nbody}, BAERO.geo.R{nbody}, COLLOCVLM(k,1), 'linear', 'extrap');
  if (R < Rint)
    IGN = [IGN, k];
  end
end
