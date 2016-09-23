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
% Export aerodynamic tansfer matrix using MASSA ASCII format
%
function export_qhh_massa(fid, outname, freq, Mach, Qhh)
%
dtpos = find('.' == outname);
%
if (isempty(dtpos))
  headname = outname;
else
  headname = outname(1:dtpos(end)-1);
end
%
lf = length(freq);
nmodes = size(Qhh, 1);
%
for n = 1: length(Mach)
%
  fileout = strcat(headname, '_M');
  string = sprintf('%4.3f', Mach(n));
  fileout =[fileout, string];
  fileout = strcat(fileout, '.aer');
  fprintf(fid, '\n - Exporting aerodynamic matrix to %s file in plain format...', fileout); 
  fp = fopen(fileout,'w');
%
  for i=1:lf
    fprintf(fp, '%5.3f %5.3f   ', Mach(n), freq(i));
  end
%
  fprintf(fp,'\n-1.000\n');
%
  for i=1:lf % frequency loop
    for j=1:nmodes % row loop
      for k=1:nmodes % column loop
        fprintf(fp,'%.9e,%.9e  ',real(Qhh(j,k,i,n)),imag(Qhh(j,k,i,n)));
      end
      fprintf(fp,'\n');
    end
  end
%
  fclose(fp);
  fprintf(fid, 'done.');
%
end
