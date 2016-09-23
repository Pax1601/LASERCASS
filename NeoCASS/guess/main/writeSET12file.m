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
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% SET1 card
%
% Called by:    MainCode.m
%
% Calls:        BULKdataSET1.m
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%     091119      1.3.7   L. Travaglini    Modification
%
% Modification by Travaglini: only cleaned
%*******************************************************************************
function writeSET12file(outf, fid, stick, aircraft, pdcylin, LEVEL)
% LEVEL is 0 for GUESS fine mesh, 1 for SMA coarse mesh
ST = pdcylin.smartcad.spline_type;
%
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Structural interpolation set');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
%
fprintf(outf, '\n\tExporting Structural interpolation sets...');
% fuselage
if (pdcylin.smartcad.caerob)
  if isequal(stick.model.fuse, 1)
    if (LEVEL==0)
      BULKdataSET1(fid, stick.IDSET.fuse, stick.ID.fuse_thick, []);
    else
      BULKdataSET1(fid, stick.IDSET.fuse, stick.ID.fuse, []);
    end
  end
end
%--------------------------------------------------------------------------
% wing
% 
if isequal(stick.model.winr, 1)
  switch ST
    case 1
      m = [];
      if (LEVEL==0)
        m(1) = pdcylin.stick.nwing_carryth;
        m(2) = pdcylin.stick.nwing_inboard;
        m(3) = pdcylin.stick.nwing_midboard;
        m(4) = pdcylin.stick.nwing_outboard;  
        m(5) = pdcylin.stick.nwing_winglet;
      else
        m(1) = pdcylin.stick.nwing_carryth_coarse;
        m(2) = pdcylin.stick.nwing_inboard_coarse;
        m(3) = pdcylin.stick.nwing_midboard_coarse;
        m(4) = pdcylin.stick.nwing_outboard_coarse;
        m(5) = pdcylin.stick.nwing_winglet_coarse;
      end
      PSET = zeros(5,2);
%     CT
      PSET(1,1) = 1;
      PSET(1,2) = m(1)+1;
%     inb
      PSET(2,1) = PSET(1,2); 
      PSET(2,2) = sum(m(1:2))+1; 
%     midb
      PSET(3,1) = PSET(2,2); 
      PSET(3,2) = sum(m(1:3))+1; 
%     outb
      PSET(4,1) = PSET(3,2); 
      PSET(4,2) = sum(m(1:4))+1; 
%     winglet
      PSET(5,1) = PSET(4,2); 
      PSET(5,2) = sum(m(1:5))+1; 
      cont = 0;
      PART = unique(stick.part.winr(stick.part.winr>0));
      npart = length(PART);
      for i = 1:npart
        index = [PSET(PART(i),1):PSET(PART(i),2)];
        BULKdataSET1(fid, stick.IDSET.winr+cont, stick.ID.winr(index), []);
        [A, B, C] = define_beam_coord2R(stick.nodes.winrC2, index(1), index(end));
        BULKdataCORD2R(fid, stick.IDSET.winr+cont, 0, A, B, C);
        cont = cont+1;
      end
      if isequal(stick.model.winl, 1)
        cont = 0;
        for i = 1:npart
          index = [PSET(PART(i),1):PSET(PART(i),2)];
          BULKdataSET1(fid, stick.IDSET.winl+cont, stick.ID.winl(index), []);
          [A, B, C] = define_beam_coord2R(stick.nodes.winlC2, index(1), index(end));
          BULKdataCORD2R(fid, stick.IDSET.winl+cont, 0, A, B, C);
          cont = cont+1;
        end
      end
%
    case 2
      BULKdataSET1(fid, stick.IDSET.winr, stick.ID.winr, stick.slaves.ID.winr);
        if isequal(stick.model.winl, 1)
          BULKdataSET1(fid, stick.IDSET.winl, stick.ID.winl, stick.slaves.ID.winl);
        end
  end
end
%--------------------------------------------------------------------------
% vertical
%
if isequal(stick.model.vert, 1)
  switch ST
    case 1
      m = [];
      if  (LEVEL==0)
        m(1) = pdcylin.stick.nvtail_inboard;
        m(2) = pdcylin.stick.nvtail_outboard;
      else
        m(1) = pdcylin.stick.nvtail_inboard_coarse;
        m(2) = pdcylin.stick.nvtail_outboard_coarse;
      end
      PSET = zeros(2,2);
      PSET(1,1) = 1;
      PSET(1,2) = m(1)+1;
      PSET(2,1) = PSET(1,2); 
      PSET(2,2) = sum(m(1:2))+1; 
      cont = 0;
      PART = unique(stick.part.vert(stick.part.vert>0));
      npart = length(PART);
      for i = 1:npart
        index = [PSET(PART(i),1):PSET(PART(i),2)];
        BULKdataSET1(fid, stick.IDSET.vert+cont, stick.ID.vert(index), []);
        [A, B, C] = define_beam_coord2R(stick.nodes.vert, index(1), index(end));
        BULKdataCORD2R(fid, stick.IDSET.vert+cont, 0, A, B, C);
        cont = cont+1;
      end

    case 2
    %
    % if HT is defined, one node is in common between HT and VT and 4
    % slaves nodes from HT must be included in the current SET1
    if isequal(stick.model.horr, 1)
      IDslaves = [stick.slaves.ID.horr(end-3:end); stick.slaves.ID.vert];
    else
      IDslaves = stick.slaves.ID.vert;
    end
    % SET1 card
    BULKdataSET1(fid, stick.IDSET.vert, stick.ID.vert, IDslaves);
  end
end
%
if isequal(stick.model.vert2, 1)
  switch ST
    case 1
      cont = 0;
      offset = 1;
      for i = 1:length(stick.IDCAERO1.vert)
        if (m(i)>0)
          index = [offset : offset+m(i)];
          BULKdataSET1(fid, stick.IDSET.vert2+cont, stick.ID.vert2(index), []);
          [A, B, C] = define_beam_coord2R(stick.nodes.vert2, index(1), index(end));
          BULKdataCORD2R(fid, stick.IDSET.vert2+cont, 0, A, B, C);
          offset = offset+m(i);
          cont = cont+1;
        end
      end
    case 2
  %
  % if HT is defined, one node is in common between HT and VT and 4
  % slaves nodes from HT must be included in the current SET1
    if isequal(stick.model.horl, 1)
      IDslaves = [stick.slaves.ID.horl(end-3:end); stick.slaves.ID.vert2];
    else
      IDslaves = stick.slaves.ID.vert2;
    end
    % SET1 card
    BULKdataSET1(fid, stick.IDSET.vert2, stick.ID.vert2, IDslaves);
  end
end
%--------------------------------------------------------------------------
% horizontal
%
if isequal(stick.model.horr, 1)
%  
  switch ST
    case 1
      m = [];
      if  (LEVEL==0)
        m(1) = pdcylin.stick.nhtail_carryth;
        m(2) = pdcylin.stick.nhtail_inboard;
        m(3) = pdcylin.stick.nhtail_outboard;
      else
        m(1) = pdcylin.stick.nhtail_carryth_coarse;
        m(2) = pdcylin.stick.nhtail_inboard_coarse;
        m(3) = pdcylin.stick.nhtail_outboard_coarse;
      end
      PSET = zeros(3,2);
      PSET(1,1) = 1;
      PSET(1,2) = m(1);
      PSET(2,1) = PSET(1,2); 
%     check if CT present
      if (PSET(2,1)==0)
        PSET(2,1) = 1;
      end
      PSET(2,2) = sum(m(1:2))+1; 
      PSET(3,1) = PSET(2,2);
      PSET(3,2) = sum(m)+1;
      cont = 0;
      PART = unique(stick.part.horr(stick.part.horr>0));
      npart = length(PART);
      for i = 1:npart
        index = [PSET(PART(i),1):PSET(PART(i),2)];
        BULKdataSET1(fid, stick.IDSET.horr+cont, stick.ID.horr(index), []);
        [A, B, C] = define_beam_coord2R(stick.nodes.horrC2, index(1), index(end));
        BULKdataCORD2R(fid, stick.IDSET.horr+cont, 0, A, B, C);
        cont = cont+1;
      end
      if isequal(stick.model.horl, 1)
        cont = 0;
        for i = 1:npart
          index = [PSET(PART(i),1):PSET(PART(i),2)];
          BULKdataSET1(fid, stick.IDSET.horl+cont, stick.ID.horl(index), []);
          [A, B, C] = define_beam_coord2R(stick.nodes.horlC2, index(1), index(end));
          BULKdataCORD2R(fid, stick.IDSET.horl+cont, 0, A, B, C);
          cont = cont+1;
        end
      end

    case 2
      %
      % if HT and VT are connected, find VT slaves nodes to be included in
      % the current SET1 for HT
      if isequal(stick.model.vert, 1)
        for i = 1:length(stick.ptos.vert(1,:))
            IndVH = find(stick.ptos.vert(3,i) == stick.ptos.horr(3,1));
            IDslaves = [stick.slaves.ID.vert(1+(IndVH-1)*4 : 4+(IndVH-1)*4); stick.slaves.ID.horr];
        end
      else
        IDslaves = stick.slaves.ID.horr;
      end
      % SET1 card
      BULKdataSET1(fid, stick.IDSET.horr, stick.ID.horr, IDslaves);
      if isequal(stick.model.horl, 1)
        BULKdataSET1(fid, stick.IDSET.horl, stick.ID.horl, stick.slaves.ID.horl);
      end
  end
end
%--------------------------------------------------------------------------
% canard
%
if isequal(stick.model.canr, 1) && aircraft.Strut_wing.present~=1

  switch ST
    case 1
      m = [];
      if  (LEVEL==0)
        m(1) = pdcylin.stick.ncanard_carryth;
        m(2) = pdcylin.stick.ncanard_inboard;
        m(3) = pdcylin.stick.ncanard_outboard;
      else
        m(1) = pdcylin.stick.ncanard_carryth_coarse;
        m(2) = pdcylin.stick.ncanard_inboard_coarse;
        m(3) = pdcylin.stick.ncanard_outboard_coarse;
      end
      PSET = zeros(3,2);
      PSET(1,1) = 1;
      PSET(1,2) = m(1)+1;
      if (m(1)==0)
        PSET(2,1) = 1; 
        PSET(2,2) = m(2)+1;
      else
        PSET(2,1) = PSET(1,2); 
        PSET(2,2) = m(1)+m(2)+1; 
      end
      PSET(3,1) = PSET(2,2);
      PSET(3,2) = sum(m)+1;
      cont = 0;
      PART = unique(stick.part.canr(stick.part.canr>0));
      npart = length(PART);
      for i = 1:npart
        index = [PSET(PART(i),1):PSET(PART(i),2)];
        BULKdataSET1(fid, stick.IDSET.canr+cont, stick.ID.canr(index), []);
        [A, B, C] = define_beam_coord2R(stick.nodes.canrC2, index(1), index(end));
        BULKdataCORD2R(fid, stick.IDSET.canr+cont, 0, A, B, C);
        cont = cont+1;
      end
      if isequal(stick.model.canl, 1)
        cont = 0;
        for i = 1:npart
          index = [PSET(PART(i),1):PSET(PART(i),2)];
          BULKdataSET1(fid, stick.IDSET.canl+cont, stick.ID.canl(index), []);
          [A, B, C] = define_beam_coord2R(stick.nodes.canlC2, index(1), index(end));
          BULKdataCORD2R(fid, stick.IDSET.canl+cont, 0, A, B, C);
          cont = cont+1;
        end
      end

    case 2
      % SET1 card
      BULKdataSET1(fid, stick.IDSET.canr, stick.ID.canr, stick.slaves.ID.canr);
      if isequal(stick.model.canl, 1)
        BULKdataSET1(fid, stick.IDSET.canl, stick.ID.canl, stick.slaves.ID.canl);
      end
  end
end
%
fprintf(outf, 'done.');
end
% Given beam nodes, determines A, B, C points defining COORD2R
function [A, B, C] = define_beam_coord2R(coord, i1, i2)
  A = coord(:,i1);
  B = coord(:,i2);
  Y = B-A; Y = Y./norm(Y);
  X = [1 0 0]';
  Z = crossm(X) * Y; Z = Z./norm(Z);
  X = crossm(Z) * Y; X = X./norm(X);
  B = A + Z;
  C = A + X;
end

