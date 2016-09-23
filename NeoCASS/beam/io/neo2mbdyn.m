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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, DIAPM
%***********************************************************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     042711      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function neo2mbdyn(headname, beam_model)
%
%   DESCRIPTION: Export NeoCASS model into MBDyn format 
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                headname       FILE       Output headname
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                beam_model     struct     beam_model database with structural
%                                          and aerodynamic data
%    REFERENCES:
%
%*******************************************************************************
%
function neo2mbdyn(headname, beam_model)
%
NGRID  = beam_model.Info.ngrid;
NDIM   = size(beam_model.Node.Coord,2);
NBARS  = beam_model.Info.nbar;
NBEAMS = beam_model.Info.nbeam;
NCOMS  = beam_model.Info.nconm;
NRBE   = beam_model.Info.nrbe2;
DAMP_FACTOR = 0.0;
%-------------------------------------------------------------------------------
% Export NODES
%
% TAG AERONODES
FLAG = ones(NGRID,1);
%
filename = [headname, '.nod'];
fid = fopen(filename, 'w');
for j=1:beam_model.Info.ngrid
  FLAG(beam_model.Node.Aero.Index(j).data) = 0;
end 
%
index = find(FLAG);
NNODES = length(index);
%structural: <node_label>, dynamic,
%            reference, <ref_label>, <x>, <y>, <z>,
%            reference, <ref_label>, eye,
%            reference, <ref_label>, null,
%            reference, <ref_label>, null;
fprintf(fid,'# ----------------------------------------\n');
fprintf(fid,'# NODES: %d\n', NNODES);
fprintf(fid,'# ----------------------------------------\n');
for j=1:NNODES
  fprintf(fid, '\n\tstructural: %d, dynamic,\n\t ', beam_model.Node.ID(index(j)));
  for k=1:NDIM
    fprintf(fid, ' %f,', beam_model.Node.Coord(index(j),k));
  end
  fprintf(fid, '\n\t  eye,\n\t  null,\n\t  null;');
end
%
fclose(fid);
filename = [headname, '.int'];
fid = fopen(filename, 'w');
%
% Export aero nodes
%
count = 0;
for j=1:NGRID
  if (~isempty(beam_model.Node.Aero.Index(j).data))
    count = count + length(beam_model.Node.Aero.Index(j).data);
  end
end
fprintf(fid,'# ----------------------------------------\n');
fprintf(fid,'# MASTER NODES: %d\n', NGRID);
fprintf(fid,'# SLAVE AERONODES: %d\n', count);
fprintf(fid,'# ----------------------------------------\n');
%
scoord = zeros(3,1);
for j=1:NNODES
  mid = beam_model.Node.ID(index(j));
  fprintf(fid,'\n%d, label, %d, offset, %f, %f, %f,', mid, mid, scoord(1), scoord(2), scoord(3));
end
%
for j=1:NGRID
  if (~isempty(beam_model.Node.Aero.Index(j).data))
    lastmid = beam_model.Node.ID(j);
  end
end
%
for j=1:NGRID
  if (~isempty(beam_model.Node.Aero.Index(j).data))
    mid = beam_model.Node.ID(j);
    if (mid~=lastmid)
      for k=1:length(beam_model.Node.Aero.Index(j).data)
        sid = beam_model.Node.ID(beam_model.Node.Aero.Index(j).data(k));
        scoord = beam_model.Node.Aero.Coord(j).data(:,k);
        fprintf(fid,'\n%d, label, %d, offset, %f, %f, %f,', mid, sid, ...
              scoord(1), scoord(2), scoord(3));
      end
    else
      for k=1:length(beam_model.Node.Aero.Index(j).data)-1
        sid = beam_model.Node.ID(beam_model.Node.Aero.Index(j).data(k));
        scoord = beam_model.Node.Aero.Coord(j).data(:,k);
        fprintf(fid,'\n%d, label, %d, offset, %f, %f, %f,', mid, sid, ...
              scoord(1), scoord(2), scoord(3));
      end
        k = k+1;
        sid = beam_model.Node.ID(beam_model.Node.Aero.Index(j).data(k));
        scoord = beam_model.Node.Aero.Coord(j).data(:,k);
        fprintf(fid,'\n%d, label, %d, offset, %f, %f, %f;', mid, sid, ...
              scoord(1), scoord(2), scoord(3));
    end
  end
end
%
fclose(fid);
filename = [headname, '.elm'];
fid = fopen(filename, 'w');
%-------------------------------------------------------------------------------
% export reference frames
REF_COUNTER = 0;
for j=1:NBARS
  n = beam_model.Bar.Conn(j,1);
  REF_COUNTER = REF_COUNTER +1;
  SDR = ['BAR',num2str(beam_model.Bar.ID(j))];
  fprintf(fid, 'set: integer %s = %d;\n', SDR, REF_COUNTER);
  fprintf(fid, 'reference: %s,\n\t', SDR);
  fprintf(fid,'%f,', beam_model.Node.Coord(n,:));
  fprintf(fid,'\n\t1,');
  fprintf(fid,' %f,', beam_model.Bar.R(:,1,1,j)');
  fprintf(fid,'\n\t3,');
  fprintf(fid,' %f,', beam_model.Bar.R(:,3,1,j)');
  fprintf(fid,'\n\tnull,\n\tnull;\n');
end
for j=1:NBEAMS
  n = beam_model.Beam.Conn(j,1);
  REF_COUNTER = REF_COUNTER +1;
  SDR = ['BAR',num2str(beam_model.Bar.ID(j))];
  fprintf(fid, 'set: integer %s = %d;\n', SDR, REF_COUNTER);
  fprintf(fid, 'reference: %s,\n\t', SDR);
  fprintf(fid,' %f,', beam_model.Node.Coord(n,:));
  fprintf(fid,'\n\t1,');
  fprintf(fid,' %f,', beam_model.Beam.R(:,1,1,j)');
  fprintf(fid,'\n\t3,');
  fprintf(fid,' %f,', beam_model.Beam.R(:,3,1,j)');
  fprintf(fid,'\n\tnull,\n\tnull;\n');
end
% Export BARS
%  beam: <beam_label>,
%      <node_1_label>,
%          position, null,
%          orientation, eye,
%      <node_2_label>,
%          position, null,
%          orientation, eye,
%      <node_3_label>,
%          position, null,
%          orientation, eye,
%      from nodes,
%      linear viscoelastic generic,
%          diag, <EA>, <GAy>, <GAz>, <GJ>, <EJy>, <EJz>,
%          proportional, <damp_factor>
%      same,
%      same;
fprintf(fid,'# ----------------------------------------\n');
fprintf(fid,'# BEAM ELEMENTS: %d\n', NBARS);
fprintf(fid,'# ----------------------------------------');
%
MASS = zeros(6,6,beam_model.Info.ngrid);
for j=1:NBARS
  fprintf(fid, '\n\tbeam: %d, ', beam_model.Bar.ID(j));  
  SDR = ['BAR',num2str(beam_model.Bar.ID(j))];
  for k=1:size(beam_model.Bar.Conn,2)
    n = beam_model.Bar.Conn(j,k);
    id = beam_model.Node.ID(n);
    fprintf(fid, '\n\t  %d,',id);
    fprintf(fid, '\n\t  position, null,\n\t  orientation, reference, %s, eye,', SDR);
    MASS(:,:,n) = MASS(:,:,n) + beam_model.Bar.M(:,:,k,j);
  end
  fprintf(fid, '\n\t  from nodes,\n\t  linear viscoelastic generic,\n\t  diag,');
  P = diag(beam_model.Bar.D(:,:,1,j));
  for k=1:length(P)
    fprintf(fid, '%e, ', P(k));    
  end
  fprintf(fid, '\n\t  proportional, DAMP_FACTOR,\n\t  same, \n\t  same;');
end
for j=1:NBEAMS
  fprintf(fid, '\n\tbeam: %d, ', beam_model.Beam.ID(j));  
  SDR = ['BEAM',num2str(beam_model.Bar.ID(j))];
  for k=1:size(beam_model.Beam.Conn,2)
    n = beam_model.Beam.Conn(j,k);
    id = beam_mode.Node.ID(n);
    fprintf(fid, '\n\t  %d,',id);
    fprintf(fid, '\n\t  position, null,\n\t  orientation, reference, %s, eye,', SDR);
    MASS(:,:,n) = MASS(:,:,n) + beam_model.Beam.M(:,:,k,j);
  end
  fprintf(fid, '\n\t  from nodes,\n\t  linear viscoelastic generic,\n\t  diag,');
  P = diag(beam_model.Beam.D(:,:,1,j));
  for k=1:length(P)
    fprintf(fid, '%e, ', P(k));    
  end
  fprintf(fid, '\n\t  proportional, DAMP_FACTOR = 0.0;,\n\t  same, \n\t  same;');
end
%-------------------------------------------------------------------------------
% export LUMPED MASSES
%body: <body_label>, <node_label>,
%      <mass>,
%      reference, node, null,
%      diag, <jx>, <jy>, <jz>;
% SUM LUMPED MASSES contribution
for j=1:NCOMS
  n = beam_model.ConM.Node(j);
  MASS(:,:,n) = MASS(:,:,n) + beam_model.ConM.M(:,:,j);
end
%
for j=1:NNODES
%
  n = index(j);
  fprintf(fid,'\n\tbody: %d, %d,', j, beam_model.Node.ID(n)); 
  m = MASS(1,1,n); 
  fprintf(fid,'\n\t  %f,', m); 
  cg = zeros(NDIM,1);
  S = MASS(4:6,1:3,n);
  if (m>eps)
    cg(3) = S(2,1) / m;
    cg(2) = S(3,1) / m;
    cg(1) = S(3,2) / m;
  end
  J = MASS(4:6,4:6,n);
  fprintf(fid,'\n\t  reference, node, %f, %f, %f,', cg(1), cg(2), cg(3)); 
  fprintf(fid,'\n\t  matr,');
  fprintf(fid, ' %f,', J(1,:));
  fprintf(fid, ' %f,', J(2,:));
  fprintf(fid, ' %f,', J(3,1:2));
  fprintf(fid, ' %f;', J(3,3));
%
end
%-------------------------------------------------------------------------------
% export joints
%body: <body_label>, <node_label>,
%      <mass>,
%      reference, node, null,
%      diag, <jx>, <jy>, <jz>;
fprintf(fid,'\n# ----------------------------------------\n');
fprintf(fid,'# JOINTS %d\n', NRBE);
fprintf(fid,'# ----------------------------------------');
%
NJ = 0;
for j=1:NRBE
  for k=1:length(beam_model.RBE2.IDS(j).data)
%
    NJ = NJ +1;
    fprintf(fid,'\njoint: %d, total joint,\n\t  %d,', NJ, beam_model.RBE2.IDM(j));
    fprintf(fid,'\n\t    position, reference, node, null,');
    fprintf(fid,'\n\t    position orientation, reference, node, eye,');
    fprintf(fid,'\n\t    rotation orientation, reference, node, eye,');
    fprintf(fid,'\n\t  %d,', beam_model.RBE2.IDS(j).data(k));
    fprintf(fid,'\n\t    position, reference, other node, null,');
    fprintf(fid,'\n\t    position orientation, reference, other node, eye,');
    fprintf(fid,'\n\t    rotation orientation, reference, other node, eye,');
%
    fprintf(fid,'\n\t  position constraint,');
    for n=1:3
      in = find(n == beam_model.RBE2.GDL(j).data);
      if (isempty(in))
        fprintf(fid,'\n\t    inactive,');
      else
        fprintf(fid,'\n\t    active,');
      end
    end
    fprintf(fid,'\n\t  null,');
    fprintf(fid,'\n\t  orientation constraint,');
    for n=4:6
      in = find(n == beam_model.RBE2.GDL(j).data);
      if (isempty(in))
        fprintf(fid,'\n\t    inactive,');
      else
        fprintf(fid,'\n\t    active,');
      end
    end
    fprintf(fid,'\n\t  null;');
%
  end
end
fclose(fid);
%-------------------------------------------------------------------------------
% export counters
filename = [headname, '.set'];
fid = fopen(filename, 'w');
fprintf(fid,'set: integer N_NODES = %d;\n', NNODES);
fprintf(fid,'set: integer N_BEAMS = %d;\n', NBARS+NBEAMS);
fprintf(fid,'set: integer N_BODIES = %d;\n', NNODES);
fprintf(fid,'set: integer N_JOINTS = %d;\n', NJ);
fprintf(fid,'set: real DAMP_FACTOR = %f;', DAMP_FACTOR);
fclose(fid);
