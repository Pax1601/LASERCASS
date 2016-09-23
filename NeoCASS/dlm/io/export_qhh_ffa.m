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
% Export aerodynamic tansfer matrix using FFA format
%
function export_qhh_ffa(fid, outname, freq, Mach, Qhh, lref)
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
  fileout = strcat(fileout, '.baer');
  fprintf(fid, '\n - Exporting aerodynamic matrix to %s file in FFA format...', fileout); 
  %
  p = ffa_create('aero_rom');
  ie = 0;
  %
  pdata = ffa_create('Mach', 'R', Mach(n));
  [p, ie] = ffa_putsub(p, pdata);
  %
  pdata = ffa_create('Nmodes_used', 'I', [1:nmodes]);
  [p, ie] = ffa_putsub(p, pdata);
  %
  pdata = ffa_create('ref_length', 'R', lref);
  [p, ie] = ffa_putsub(p, pdata);
  %
  pdata = ffa_create('red_freq', 'R', freq);
  [p, ie] = ffa_putsub(p, pdata);
%
  for i=1:lf
%
    pham = ffa_create('ham_mat');
%  pdata = ffa_create('k', 'R', freq(i));
%  [pham, ie] = ffa_putsub(pham, pdata);
%
    data = real(Qhh(:,:,i,n));
    pdata = ffa_create('real_part', 'R', data);
    [pham, ie] = ffa_putsub(pham, pdata);
%
    data = imag(Qhh(:,:,i,n));
    pdata = ffa_create('imag_part', 'R', data);
    [pham, ie] = ffa_putsub(pham, pdata);
%
    [p, ie] = ffa_putsub(p, pham);
%
  end

  if (ie ~= 0) 
    fprintf(fid, '\nError occoured in creating ROM FFA format dataset.');
    return;
  end
  % write FFA binary file
  ffa_dump(p, fileout);
  clear p;
  fprintf('done.');      

end
