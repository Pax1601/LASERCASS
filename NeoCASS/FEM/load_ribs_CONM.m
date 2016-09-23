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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Load smartcad file and ribs data (.mat file created by wing_fem.m). 
% Export masses along semiwing as CONM1 and GRIDs.
% Connects GRIDs to ribs through RBE3 elements
% Two files are created:
% ns_mass.dat
% RBE3.dat. RBE3 (which INCLUDES ns_mass.mat) is already included in 
% the main NASTRAN file
%
function load_ribs_CONM(sma_file, ribs_file)
%
OFFSET     = 1999999;
OFFSET_RBE = 2000000;
OFFSET_M   = 2000000;
fid = 1;
%
if ~exist(sma_file,'file');
  fprintf(fid, '\nUnable to find %s file.\n', sma_file);
  return;
end
if ~exist(ribs_file,'file');
  fprintf(fid, '\nUnable to find %s file.\n', ribs_file);
  return;
end
%
beam_model = load_nastran_model(sma_file);
load(ribs_file)
%
ConM = beam_model.ConM;
NODE = beam_model.Node;
%
rib_c = ribs_data.C;
vert1 = ribs_data.V1;
vert2 = ribs_data.V2;
vert3 = ribs_data.V3;
vert4 = ribs_data.V4;
%
%right wing
i1 = find(beam_model.Node.ID>2000);
i2 = find(beam_model.Node.ID<3000);
index = intersect(i1, i2);
n = length(index)/2;
i1 = find(beam_model.Node.ID==2000);
nindex = [i1, index(1:n)];
ID = beam_model.Node.ID(nindex);
flag = zeros(length(nindex),1);
% half mass at wing box centerline (node 2000)
mindex = find(ConM.Node == nindex(1));
for j=1:length(mindex)
  ncom = mindex(j);
  ConM.M(:,:,ncom) = 0.5 .* ConM.M(:,:,ncom);
end
%
fp=fopen('ns_mass.dat','w');
fprintf(fp,'$\n$ Lumped masses along wing box from SMARTCAD model %s\n$',sma_file);
%
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
mcont = 0;
for k=1:length(nindex)
  mindex = find(ConM.Node == nindex(k));
  if ~isempty(mindex)
    flag(k) = 1;
    for j=1:length(mindex)
      mcont = mcont+1;
      id = sprintf('%-8d', mcont+OFFSET_M);
      ncom = mindex(j);
      n = sprintf('%-8d', beam_model.Node.ID(beam_model.ConM.Node(ncom)));
      M11 = num2str8(cnvt2_8chs(ConM.M(1,1,ncom)));
      M21 = num2str8(cnvt2_8chs(ConM.M(2,1,ncom)));
      M22 = num2str8(cnvt2_8chs(ConM.M(2,2,ncom)));
%
      M31 = num2str8(cnvt2_8chs(ConM.M(3,1,ncom)));
      M32 = num2str8(cnvt2_8chs(ConM.M(3,2,ncom)));
      M33 = num2str8(cnvt2_8chs(ConM.M(3,3,ncom)));
%
      M41 = num2str8(cnvt2_8chs(ConM.M(4,1,ncom)));
      M42 = num2str8(cnvt2_8chs(ConM.M(4,2,ncom)));
      M43 = num2str8(cnvt2_8chs(ConM.M(4,3,ncom)));
      M44 = num2str8(cnvt2_8chs(ConM.M(4,4,ncom)));
%
      M51 = num2str8(cnvt2_8chs(ConM.M(5,1,ncom)));
      M52 = num2str8(cnvt2_8chs(ConM.M(5,2,ncom)));
      M53 = num2str8(cnvt2_8chs(ConM.M(5,3,ncom)));
      M54 = num2str8(cnvt2_8chs(ConM.M(5,4,ncom)));
      M55 = num2str8(cnvt2_8chs(ConM.M(5,5,ncom)));
%
      M61 = num2str8(cnvt2_8chs(ConM.M(6,1,ncom)));
      M62 = num2str8(cnvt2_8chs(ConM.M(6,2,ncom)));
      M63 = num2str8(cnvt2_8chs(ConM.M(6,3,ncom)));
      M64 = num2str8(cnvt2_8chs(ConM.M(6,4,ncom)));
      M65 = num2str8(cnvt2_8chs(ConM.M(6,5,ncom)));
      M66 = num2str8(cnvt2_8chs(ConM.M(6,6,ncom)));
%
      fprintf(fp,'\nCONM1   %s%s        %s%s%s%s%s', id, n, M11,M21,M22,M31,M32);
      fprintf(fp,'\n        %s%s%s%s%s%s%s%s', M33,M41,M42,M43,M44,M51,M52,M53);
      fprintf(fp,'\n        %s%s%s%s%s%s%s%s', M54,M55,M61,M62,M63,M64,M65,M66);
%
    end  
  end
end
%
fprintf(fp,'\n$\n$ Grids along SMARTCAD model\n$');
fprintf(fp,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
for k=1:length(nindex)
%  if (flag(k))
    ngrid = nindex(k);
    X1 = num2str8(cnvt2_8chs(ConM.M(6,4,ncom)));
    id = sprintf('%-8d', NODE.ID(ngrid));
    
    c1 = num2str8(cnvt2_8chs(NODE.Coord(ngrid,1)));
    c2 = num2str8(cnvt2_8chs(NODE.Coord(ngrid,2)));
    c3 = num2str8(cnvt2_8chs(NODE.Coord(ngrid,3)));
    fprintf(fp,'\nGRID    %s 0      %s%s%s', id, c1,c2,c3);
%  end
end
% add dummy node for RFORCE
id =  sprintf('%-8d', max(NODE.ID(nindex)+1));
sup = find(NODE.ID == beam_model.Param.SUPORT(1));
c1 = num2str8(cnvt2_8chs(NODE.Coord(sup,1)));
c2 = num2str8(cnvt2_8chs(NODE.Coord(sup,2)));
c3 = num2str8(cnvt2_8chs(NODE.Coord(sup,3)));
fprintf(fp,'\n$dummy node for RFORCE\nGRID    %s 0      %s%s%s        0123456', id, c1,c2,c3);
fclose(fp);
%
fp=fopen('RBE3.dat','w');
dist=zeros(length(rib_c),1);
fprintf(fp,'$\n$ Include non-structural masses\n$\n');
fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fp,'INCLUDE  ''ns_mass.dat''\n');
fprintf(fp,'$\n$ Connect masses to ribs through RBE3 element\n$\n');
fprintf(fp,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
for k=1:length(nindex)
   ngrid = nindex(k);
   for j=1:length(rib_c)
       dist(j)=norm(NODE.Coord(ngrid,:)-rib_c(j).coord);       
   end
   [~,rib] = min(dist);
   fprintf(fp,'%8s%8d%8s%8d%8s%8f%8s%8d%8d\n','RBE3    ',OFFSET_RBE+k,'',...
      NODE.ID(ngrid),'123',1.,'123',OFFSET+vert1(rib).grid,OFFSET+vert2(rib).grid);
   fprintf(fp,'%8s%8d%8d\n','',OFFSET+vert3(rib).grid,OFFSET+vert4(rib).grid);
end
fclose(fp);

