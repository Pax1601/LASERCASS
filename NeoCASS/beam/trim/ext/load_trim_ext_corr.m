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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci           <ricci@aero.polimi.it>
%                      Luca Cavagna           <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari  <degaspari@aero.polimi.it>
%                      Luca Riccobene         <riccobene@aero.polimi.it>
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
% Load corrections to apply to VLM derivatives
%
function DER = load_trim_ext_corr(filename, MASTER)

  fid = 1;
  fdata = importdata(filename);
  nd = size(fdata.data,1);
  nsurf = length(MASTER);
  nc = 5+nsurf;
  DER = ones(nc,1);
%
  for i=1:nd
    switch(char(fdata.textdata(i,1)))
      case {'ALPHA', 'alpha', 'ANGLEA', 'anglea'}
        DER(1) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for ALPHA: %g.', DER(1));
     case {'BETA', 'beta', 'SIDES', 'sides'}
        DER(2) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for BETA: %g.', DER(2));
      case {'p', 'P' 'ROLL', 'roll'}
        DER(3) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for P: %g.', DER(3));
      case {'q', 'Q', 'PITCH', 'pitch'}
        DER(4) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for Q: %g.', DER(4));
      case {'r', 'R', 'YAW', 'yaw'}
        DER(5) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for R: %g.', DER(5));
    end
  end
%
  for i=1:nd
    for n=1:nsurf
      if strcmp(MASTER(n),char(fdata.textdata(i,1)))
        DER(5+n) = fdata.data(i,1);
        fprintf(fid,'\n - Scale factor for %s: %g.', cell2mat(MASTER(n)), DER(5+n));
        break;
      end
    end
  end
%
end
