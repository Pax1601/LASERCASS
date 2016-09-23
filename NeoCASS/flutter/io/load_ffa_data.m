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
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
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
%   Author: Luca Cavagna
%
%***********************************************************************************************************************
%
% Load structural and aerodynamic data from FFA format (FOI, Sweden)
% FFA matlab toolbox is required and is provided by FOI (www.foi.se/edge)
%
% Binary files are required
% Use Edge utility ffaa2b to convert ASCII files to binary format
%
function [Mhh, Chh, Khh, Freq, MODE_ID, Qhh, Klist, Mach, lref, err] = load_ffa_data(str_filename, aero_filename)
%

fid = 1;
%
err = 0;
Klist = [];
Mach = 0;
%
fprintf(fid, '\n - Loading aeroelastic model using FFA format...');
%
[ds, ps, err] = ffabload(str_filename, 'mute'); if (err ~= 0) return; end
%
pmode = ffa_find(ds,'name','mode_set');
dims = ffa_get(ds, pmode{1}, 'dims');
NMODES = dims(3);
%
[da, pa, err] = ffabload(aero_filename, 'mute'); if (err ~= 0) return; end
%***********************************************************************************************************************
% Structural data
%
Mhh = zeros(NMODES);
Bhh = zeros(NMODES);
Chh = zeros(NMODES);
Khh = zeros(NMODES);

MODE_ID = zeros(NMODES,1);
Omega = zeros(NMODES, 1);
Freq = zeros(NMODES, 1);

[ds_modes, err] = ffa_getsub(ds,pmode{1});
pmodes = ffa_find(ds_modes,'name','mode');

for n = 1:NMODES

  ds_mode = ffa_getsub(ds_modes,pmodes{n});    
  pdata = ffa_find(ds_mode,'name','identifier');
  MODE_ID(n) = ffa_get(ds_mode, pdata{1});
%
  pdata = ffa_find(ds_mode,'name','generalised_mass');
  Mhh(n,n) = ffa_get(ds_mode, pdata{1});
% 
  pdata = ffa_find(ds_mode,'name','damping_ratio');
  Chh(n,n) = 2*ffa_get(ds_mode, pdata{1});
%
  pdata = ffa_find(ds_mode,'name','frequency_hz');
  Freq(n) = ffa_get(ds_mode, pdata{1});
  Omega(n) = 2 * pi * Freq(n);
  Khh(n,n) = Mhh(n,n) * Omega(n) * Omega(n);
%
end
%
fprintf(fid, '\n   - Structural data correctly loaded from file %s.', str_filename);
fprintf(fid, '\n       Number of modes used: %g.', NMODES);
%
%***********************************************************************************************************************
% Aerodynamic data
%
pdata = ffa_find(da,'name','Mach');
Mach = ffa_get(da, pdata{1});
%
pdata = ffa_find(da,'name','red_freq');
Klist = ffa_get(da, pdata{1});
%
pdata = ffa_find(da,'name','Nmodes_used');
NMODES_QHH = ffa_get(da, pdata{1});
%
%if (length(NMODES_QHH) ~= NMODES) 
%  fprintf(fid, '\nWrong dimension between structural and aerodynamic model.');
%  err = 1;
%  return
%end
%
pdata = ffa_find(da,'name','ref_length');
lref = ffa_get(da, pdata{1});
%
pham = ffa_find(da,'name','ham_mat');
%
Qhh = complex(zeros(NMODES, NMODES, 2*length(Klist)));
%
[Klist, index] = sort(Klist);
for n = 1:length(Klist)
%
  [da_ham, err] = ffa_getsub(da, pham{n});
  pdata = ffa_find(da_ham,'name','real_part');
  ham_real = ffa_get(da_ham, pdata{1});
  pdata = ffa_find(da_ham,'name','imag_part');
  ham_imag = ffa_get(da_ham, pdata{1});
  page = 2 * index(n) -1;
  Qhh(:,:,page) = complex(ham_real, ham_imag);
%
end
% 
fprintf(fid, '\n   - Aerodynamic data correctly loaded from file %s.', aero_filename);
fprintf(fid, '\n       Mach number: %g.', Mach);
fprintf(fid, '\n       Matrix dimensions: %d,%d,%d.', size(Qhh,1), size(Qhh,2), size(Qhh,3));
%***********************************************************************************************************************
%
fprintf(fid, '\n   done.\n');
