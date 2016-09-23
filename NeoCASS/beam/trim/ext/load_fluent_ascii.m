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
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
% Load FLUENT ASCII file with surface data 
%
function [C, F] = load_fluent_ascii(filename, extra, symm, Frame)
%
fid = 1;
%
fprintf(fid,'\n Reading file %s...', filename)
%
if exist(filename, 'file')
  val = importdata(filename, ' ', 1);
%
  IDi = find_field(val.textdata, 'nodenumber', 1);
  Xi  = find_field(val.textdata, 'x-coordinate', 1);
  Yi  = find_field(val.textdata, 'y-coordinate', 1);
  Zi  = find_field(val.textdata, 'z-coordinate', 1);
  Xfi  = find_field(val.textdata, 'x-face-area', 1);
  Yfi  = find_field(val.textdata, 'y-face-area', 1);
  Zfi  = find_field(val.textdata, 'z-face-area', 1);
  cpi  = find_field(val.textdata, 'pressure-coefficient', 0);
  pi  = find_field(val.textdata, 'pressure', 0);
% recover data
  C = [val.data(:,Xi),  val.data(:,Yi),  val.data(:,Zi)];
  A = [val.data(:,Xfi), val.data(:,Yfi), val.data(:,Zfi)];
%
  if ~isempty(cpi)
    F = repmat(val.data(:,cpi) .* extra.QREF, 1, 3) .* A;
  elseif ~isempty(pi)
    F = repmat((val.data(:,pi) - extra.PREF), 1, 3) .* A;
  else
    error('Unable to find either pressure-coefficient or pressure field.');
  end  
%
  np = size(F,1);
%
  if ~isempty(Frame)
    if Frame.Coord
      for i=1:np
        C(i,:) = (Frame.Rmat * C(i,:)')' + Frame.Origin;
        F(i,:) = Frame.Rmat * F(i,:)';
      end
    end
  end
%
  plot3(C(i,1), C(i,2), C(i,3),'ko'); axis equal;
%
  if (symm)
    F(:,2) = -F(:,2);
    C(:,2) = -C(:,2);
  end
% 
  fprintf(fid, '\nForces resultants along aero mesh: ');
  fprintf(fid, '\n\tFx: %g.', sum(F(:,1)));
  fprintf(fid, '\n\tFy: %g.', sum(F(:,2)));
  fprintf(fid, '\n\tFz: %g.', sum(F(:,3)));
  fprintf(fid, '\n');
% 
  F = reshape(F', np*3,1);
%
else
  fprintf(fid,'Unable to find file %s.', filename);
end
%
fprintf(fid,'\n done.')
end
%
function index = find_field(name, field, err_check)
  index = [];
  len = length(name);
  for i=1:len
    if (strcmp(name{i},field))
      index = i;
      break;
    end
  end
  if err_check
    if isempty(index)
      error(['Unable to find field', fiel,'.']);
    end
  end
end

