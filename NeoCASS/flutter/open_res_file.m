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
function fid = open_res_file(filename)
%
global fl_model;
%
fid=fopen(filename, 'w');
nmodc = length(fl_model.struct.MID);
nmods = length(fl_model.struct.SID);
NKfreq = length(fl_model.aero.Klist);
%
fprintf(fid,'        ***************************************************************\n');
fprintf(fid,'        **                                                           **\n');
fprintf(fid,'        **                  FLUTTER SUMMARY                          **\n');
fprintf(fid,'        **                                                           **\n');
fprintf(fid,'        ***************************************************************\n\n\n\n\n');
fprintf(fid,'PARAMETERS USED FOR AEROELASTIC ANALYSIS\n\n');
fprintf(fid,'AEROELASTIC BASE SIZE:                        %i\n',nmods);
fprintf(fid,'REDUCED FREQUENCIES NUMBER:                   %i\n',NKfreq);
fprintf(fid,'\n\nREDUCED FREQUENCY LIST:\n\n');
for n = 1:NKfreq
  fprintf(fid,'%-8.5f \n',fl_model.aero.Klist(n));
end;
fprintf(fid,'\n\nREFERENCE CHORD:                          %i\n',fl_model.ref.lref);
fprintf(fid,'REFERENCE DENSITY:                             %d\n',fl_model.aero.rho);
if (fl_model.struct.Chh ~= zeros(nmodc))
  fprintf(fid,'\n\n                  STRUCTURAL DAMPING SUMMARY\n\n');
  fprintf(fid,' Damping        Frequency\n');
  for n=1:nmodc
    fprintf(fid,'%d   %g\n',fl_model.struct.Chh(n,n),fl_model.struct.MFreq(n));
  end;

else
    
  fprintf(fid,'STRUCTURAL DAMPING:                            0\n');  

end;

fprintf(fid,'\n\n                    FREQUENCY SWEEP\n\n');
fprintf(fid,'DELTA V:                                       %g\n',     fl_model.param.DVEL);
fprintf(fid,'MAXIMUM VELOCITY:                              %g\n',     fl_model.param.VMAX);
fprintf(fid,'MAXIMUM DELTA V:                               %g\n',     fl_model.param.DVMAX);
fprintf(fid,'MINIMUM DELTA V:                               %g\n',     fl_model.param.DVMIN);
fprintf(fid,'CONVERGENCE CRITERION (%%):                     %g \n',   fl_model.param.ERR);
fprintf(fid,'ERROR CRITERION FOR STEP INCREASING (%%):       %g \n',   fl_model.param.DOUBLING);
fprintf(fid,'MAXIMUM NUMBER OF ITERATIONS FOR FIRST STEP:   %g \n',    fl_model.param.MAXITE);
fprintf(fid,'MAXIMUM NUMBER OF ITERATIONS DURING SWEEP:     %g \n\n\n',fl_model.param.MAXITS);
