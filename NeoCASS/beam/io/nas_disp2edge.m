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
%     092211      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function nas_disp2edge(filename, outheadname)
%
%   DESCRIPTION: Export NASTRAN displacements to Edge FFA format for coupled 
%   aeroelastics (ISOOPT = 202)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename       FILE       Input filename with displacements 
%                                          (.pch or .f06 NASTRAN file)
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                outheadname    FILE       Output filename 
%                                          (.ainp as expected by Edge)
%
%    REFERENCES:
%
%*******************************************************************************
%
function [disp,data] = nas_disp2edge(filename, outheadname, IPRM, id_edge, coord, prev, DAMP)
%
[headname, ext] = strtok(outheadname, '.');
if (~isempty(ext))
  error('Output filename has extension.');
end
%
fp = fopen(filename, 'r');
[headname, ext] = strtok(filename, '.');
if (isempty(ext))
  error('Filename has no extension.');
end
%
j = 1;
%-------------------------------------------------------------------------------
switch (ext)
%
% PCH file
%
  case '.pch'
% skip head strings
    for k=1:7
      tline = fgets(fp);
    end
% count nodes
    while ~feof(fp)
      j = j+1;
      tline = fgets(fp);
      tline = fgets(fp);
    end
    fclose(fp);
    j = j-1;
%
    id = zeros(j,1);
    data = zeros(j,3);
%
    fp = fopen(filename, 'r');
    j = 1;
    for k=1:6
      tline = fgets(fp);
    end
% read data
    while ~feof(fp)
      A = fscanf(fp, '%10d%8s%18g%18g%18g%8d');
      id(j) = A(1);
      data(j, 1:3) = A(3:5);
      tline = fgets(fp);
      j = j+1;
    end
    fclose(fp);
%
% f06 file
%
  case '.f06'

    error('F06 extension not available yet.');

  othwerise
    error('Unknown file extension.');
end
%-------------------------------------------------------------------------------
% sort data
[ids, index] = sort(id);
data = data(index,:);
nn = length(id_edge);
expi = zeros(nn,1);
%
for k=1:nn
  i = find(ids==id_edge(k));
  if (isempty(i))
    error('Unable to find node %d in NASTRAN database.', id_edge(k));
  end
  expi(k) = i; 
end
%
id = id(expi);
disp = data(expi,:);
data = (1-DAMP).*disp + DAMP.*prev + coord;
%-------------------------------------------------------------------------------
%
% Export FFA file
%
p = ffa_create('ext_model');
ie = 0;
pdata = ffa_create('str_coordinates', 'R', data);
[p, ie] = ffa_putsub(p, pdata);
pdata = ffa_create('grid_idents', 'I', id);
[p, ie] = ffa_putsub(p, pdata);
% write FFA binary file
ffa_dump(p, [outheadname,'_DAE202.binp']);
command = ['ffab2a ', outheadname,'_DAE202.binp ', outheadname,'_DAE202.ainp'];
system(command);
if (IPRM==1)
  command = ['mv ', outheadname,'_DAE202.ainp ',outheadname,'_DAE202.ainp_in'];
  system(command);
end
%
%fp = fopen([outheadname,'_FAE202.ainp'],'w');
%fprintf(fp, 'UPDATE,N,0,0,1\nFLAG,I,1,1,0\n3');
%fclose(fp);
%
end
