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
% Export fuselage differential pressure to SMARTCAD
%
function writeFUSE_DP2file(fid, fp, pdcylin, aircraft)

if (pdcylin.fact.stab ~= 0)
  fprintf(fid, '\n     Exporting fuselage differential pressure...');

  scale = 6.8927*1e3;
  pd = scale *aircraft.cabin.Max_pressure_differential;
  fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fp, '\n$ Fuselage internal differential pressure (Pa)');
  fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  [DP] = cnvt2_8chs(pd);
  fprintf(fp, 'PARAM   FUSE_DP '); fprintf(fp, '%c', DP);
  fprintf(fp, '\n');
  fprintf(fid, 'done.');
end    

end
