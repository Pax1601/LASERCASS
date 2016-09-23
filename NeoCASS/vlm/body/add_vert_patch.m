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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function [DijY, DijZ, Sij] = get_body_matrix(nbody, BAERO, state, PG)
%
%   DESCRIPTION: Assemblly influence matrix for aero body
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                nbody          int        body index             
%                BAERO          struct     body struct             
%                state          struct     VLM state struct
%                PG             real       Prandtl Glauert             
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                DijY,DijZ,Sij  matrix     influence matrix for Y doublets, 
%                                          Z doublets and sources
%    REFERENCES:
%
%*******************************************************************************
% Add dummy CAERO for vertical tail to guarantee tail body intersection
%
% fid: output pointer
% CAERO: AERO struct
function [CAERO, nadd] = add_vert_patch(fid, CAERO)

ID = 300;
nadd = 0;
BAERO = CAERO.body;
% find vertical plane
i1 = find(CAERO.ID>300);
i2 = find(CAERO.ID<400);
index = intersect(i1,i2);
%
if(~isempty(index))
  rs = index(1); % access to first CAERO for VT (300)
  fuse = find(BAERO.ID == 1);
  if (fuse)
    Rmat = BAERO.Rmat(:,:,fuse);
    AP1(1,1) = CAERO.geo.startx(rs) - BAERO.geo.ref_point(fuse,1);
    AP2(1,1) = CAERO.geo.startx(rs) + CAERO.geo.c(rs)- BAERO.geo.ref_point(fuse,1);
    AP1(2,1) = CAERO.geo.starty(rs) - BAERO.geo.ref_point(fuse,2);
    AP2(2,1) = AP1(2,1);
    AP1(3,1) = CAERO.geo.startz(rs) - BAERO.geo.ref_point(fuse,3);
    AP2(3,1) = AP1(3,1);
    AP1R = (Rmat' * AP1)';
    AP2R = (Rmat' * AP2)';
    R1 = interp1(BAERO.geo.x{fuse}, BAERO.geo.R{fuse}, AP1R(1),'linear','extrap');
    R2 = interp1(BAERO.geo.x{fuse}, BAERO.geo.R{fuse}, AP2R(1),'linear','extrap');
   % check if CAERO is within body and cut vt CAERO
    if (AP1R(1)<BAERO.geo.x{fuse}(end)) % exclude VT2 and tailbooms
      if (AP1R(3)<R1) % if vert patch is within body, cut
        sec2 = find(BAERO.geo.x{fuse}>AP1R(1));
        sec1 = find(BAERO.geo.x{fuse}<AP1R(1));
        sec1 = max(sec1);
        sec2 = min(sec2);
        AP3(1,1) = CAERO.geo.startx(rs);
        AP3(2,1) = 0.0;
        AP3(3,1) = CAERO.geo.startz(rs) + CAERO.geo.b(rs,1);
        AP3(1,1) = CAERO.geo.startx(rs) + tan(CAERO.geo.SW(rs,1))*CAERO.geo.b(rs,1)+CAERO.geo.c(rs)/4-CAERO.geo.c(rs)*CAERO.geo.T(rs)/4;
        AP3(3,1) = AP3(3,1) - BAERO.geo.ref_point(fuse,3);
        CTIP = CAERO.geo.c(rs)*CAERO.geo.T(rs);
        AP3R = (Rmat' * AP3)';
        TANSW = (AP3R(3)-AP1R(3))/(AP3R(1)-AP1R(1));
        r1t = interp1(BAERO.geo.x{fuse}, BAERO.geo.R{fuse}, BAERO.geo.x{fuse}(sec1),'linear','extrap');
        r2t = interp1(BAERO.geo.x{fuse}, BAERO.geo.R{fuse}, BAERO.geo.x{fuse}(sec2),'linear','extrap');
        DELTAR = r2t - r1t;
        DX = (BAERO.geo.x{fuse}(sec2)-BAERO.geo.x{fuse}(sec1));
        DRX = DELTAR / DX;
        XINT = (r1t - DRX * BAERO.geo.x{fuse}(sec1) - AP1R(3) + TANSW*AP1R(1)) / (TANSW - DRX);
        RINT = interp1(BAERO.geo.x{fuse}, BAERO.geo.R{fuse}, XINT,'linear','extrap');
        PINT = Rmat * [XINT;0;RINT] + BAERO.geo.ref_point(fuse,:)';
        DELTAC = PINT(1) - CAERO.geo.startx(rs);
        DELTAB = PINT(3) - CAERO.geo.startz(rs);
        CROOT = CAERO.geo.c(rs);
        CAERO.geo.c(rs) =  CAERO.geo.c(rs) - DELTAC;
        CAERO.geo.b(rs,1) = CAERO.geo.b(rs,1) - DELTAB;
        CAERO.geo.c(rs) =  4*(CTIP/4-PINT(1)+AP3(1,1)-CAERO.geo.b(rs,1)*tan(CAERO.geo.SW(rs,1)));
        CAERO.geo.startz(rs) = PINT(3);
        CAERO.geo.startx(rs) = PINT(1);
        CAERO.geo.T(rs) =   CTIP/CAERO.geo.c(rs);
        APmin = Rmat * [XINT;0;RINT/4] + BAERO.geo.ref_point(fuse,:)';
        fprintf(fid,'\n\t\t### Warning: CAERO1 %d geometry modified to account for fuselage intersection.', CAERO.ID(rs));
      else  % if vert patch is outside body, set internal patch
        Rmin = min(R1,R2)/4;
        APmin = (Rmat * [AP1R(1,1); 0.0; Rmin])' + BAERO.geo.ref_point(fuse,:);
      end
      ncaero = length(CAERO.ID)+1;
      CAERO.geo.nelem(ncaero) = 1;
      CAERO.ID(ncaero) = 300;
      CAERO.geo.dihed(ncaero, 1) = pi/2;
      CAERO.CP(ncaero) =0 ;
      CAERO.geo.nx(ncaero,1) = (CAERO.geo.nx(rs,1) + CAERO.geo.fnx(rs,1));
      CAERO.geo.foil(ncaero, 1, 1) = CAERO.geo.foil(rs, 1, 1);
      CAERO.geo.foil(ncaero, 1, 2) = CAERO.geo.foil(rs, 1, 2);
      CAERO.geo.meshtype(ncaero,1) = CAERO.geo.meshtype(rs,1);
      CAERO.INT(ncaero) = CAERO.INT(rs);
      CAERO.geo.flapped(ncaero,1) = 0;
      CAERO.geo.fc(ncaero,1,1) = 0;
      CAERO.geo.fc(ncaero,1,2) = 0;
      CAERO.geo.fnx(ncaero,1) = 0;
      CAERO.geo.fsym(ncaero,1) = 0;
      CAERO.geo.b(ncaero,1)     = abs(APmin(3) - CAERO.geo.startz(rs));   
      STARTP = [CAERO.geo.startx(rs),CAERO.geo.starty(rs),CAERO.geo.startz(rs)];
      [P2, C] = prolong_patch(STARTP, CAERO.geo.c(rs), CAERO.geo.b(rs), CAERO.geo.T(rs), CAERO.geo.SW(rs), ...
                CAERO.geo.TW(rs,1,1), CAERO.geo.dihed(rs), CAERO.geo.b(ncaero));
      CAERO.geo.startx(ncaero)  = CAERO.geo.startx(rs);
      CAERO.geo.ny(ncaero,1) = ceil( CAERO.geo.ny(rs,1) * CAERO.geo.b(ncaero,1) / CAERO.geo.b(rs,1));
      CAERO.geo.SW(ncaero,1)    = CAERO.geo.SW(rs,1);
      CAERO.geo.TW(ncaero,1,1)  = 0.0;
      CAERO.geo.TW(ncaero,1,2)  =  0.0;
%
      CAERO.geo.startx(ncaero) = P2(1);
      CAERO.geo.starty(ncaero) = P2(2);
      CAERO.geo.startz(ncaero) = P2(3);
      CAERO.geo.c(ncaero)  = C;
      CAERO.geo.T(ncaero,1) = CAERO.geo.c(rs) /C;
%
      fprintf(fid,'\n\t\tAdded CAERO1 300 for vertical tail carrythrough.');
      nadd = nadd +1;

    end
  end % fuse
end