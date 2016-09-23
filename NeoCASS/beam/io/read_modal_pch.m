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
% function [id, displ] = read_modal_pch(filename)
%
%   DESCRIPTION: Export NASTRAN displacements to Edge FFA format for coupled 
%   aeroelastics (ISOOPT = 202)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                filename       FILE       Input pch file with displacements 
%                                          
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                id             INT        Node ID 
%                displ          REAL       Node modal displacements 
%
%    REFERENCES:
%
%*******************************************************************************
%
function [id, displ] = read_modal_pch(filename)
%
fp = fopen(filename, 'r');
[headname, ext] = strtok(filename, '.');
if (isempty(ext))
  error('Filename has no extension.');
end
%
j = 0;
%-------------------------------------------------------------------------------
% skip head strings
% count nodes
  for k=1:7
    tline = fgets(fp)
  end
  while ~feof(fp)
    % number of nodes
    j = j+1
    tline = fgets(fp);
    if (strcmp(tline(1),'$'))
      break;
    end
  end
  j = j-1;
  nstr = j/2;
  fclose(fp);
% count modes
  nm = 0
  fp = fopen(filename, 'r');
  while ~feof(fp)
    % number of nodes
    tline = fgets(fp);
    if (strcmp(tline(1:6),'$TITLE'))
    nm = nm+1;
    end
  end
  fclose(fp);
% read data
  displ = zeros(nstr,6,nm);
  id = zeros(nstr,1);
  fp = fopen(filename, 'r');
  for i=1:nm
    for k=1:7
      tline = fgets(fp)
    end
    for j=1:nstr
      tline = fgets(fp)
      id(j) = str2num(tline(1:10));
      displ(j,1,i) = str2num(tline(19:36));
      displ(j,2,i) = str2num(tline(37:54));
      displ(j,3,i) = str2num(tline(55:72));
      tline = fgets(fp)
      displ(j,4,i) = str2num(tline(19:36));
      displ(j,5,i) = str2num(tline(37:54));
      displ(j,6,i) = str2num(tline(55:72));

    end

  end

