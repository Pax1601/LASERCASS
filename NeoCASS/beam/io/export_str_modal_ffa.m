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
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************
function export_str_modal_ffa(fid, outname, NODE_ID, NODE_COORD, MODES, ID, MASS, STIFF, DAMP)
%
ie = 0;
twopi = 2*pi;
dtpos = find('.' == outname);
%
if (isempty(dtpos))
  headname = outname;
else
  headname = outname(1:dtpos(end)-1);
end
%
fileout = strcat(headname, '.bmod');
fprintf(fid, '\n - Exporting structural modal model to %s file in FFA format...', fileout); 
m = diag(MASS);
k = diag(STIFF);
g = diag(DAMP);
%
Freq = (sqrt(k./m))./twopi;

p = ffa_create('modes_definition');

pdata = ffa_create('title', 'L', 'Elastic modes from NEOCASS');
[p, ie] = ffa_putsub(p, pdata);

pdata = ffa_create('grid_idents', 'I', NODE_ID);
[p, ie] = ffa_putsub(p, pdata);

pdata = ffa_create('str_coordinates', 'R', NODE_COORD);
[p, ie] = ffa_putsub(p, pdata);

pmodes = ffa_create('mode_set');
NMODES = size(MODES, 3);

for i = 1:NMODES

  pmode = ffa_create('mode');

  pdata = ffa_create('identifier', 'I', ID(i));
  [pmode, ie] = ffa_putsub(pmode, pdata);

  pdata = ffa_create('frequency_hz', 'R', Freq(i));
  [pmode, ie] = ffa_putsub(pmode, pdata);

  pdata = ffa_create('generalised_mass', 'R', m(i));
  [pmode, ie] = ffa_putsub(pmode, pdata);

  pdata = ffa_create('damping_ratio', 'R', g(i)/2);
  [pmode, ie] = ffa_putsub(pmode, pdata);

  % do not write nodal rotations
  pdata = ffa_create('str_displacement', 'R', MODES(:,1:3,i));
  [pmode, ie] = ffa_putsub(pmode, pdata);

  [pmodes, ie] = ffa_putsub(pmodes, pmode);

end

[p, ie] = ffa_putsub(p, pmodes);

ffa_dump(p, fileout);
fprintf('done.');      
