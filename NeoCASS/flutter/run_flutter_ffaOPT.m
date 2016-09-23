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

function [OUT] = run_flutter_ffaOPT(strf, aerof, mbase, msw, rho, vrange, plot_req, nfig, nres)
%
%***********************************************************************************************************************
%
% FUNCTION: Flutter tracking at constant Mach number
%           Load structural and aerodynamic data from FFA format (FOI, Sweden)
%           FFA matlab toolbox is required and is provided by FOI (www.foi.se/edge)
%
%           Binary files are required
%           Use Edge utility ffaa2b to convert ASCII files to binary format
%
%  INPUTS:    NAME       TYPE                   DESCRIPTION
%  1          strf       character              .bmod file with structural data
%  2          aerof      character              .brom file with aerodynamic data
%  3          mbase      integer array          identifiers of structural modes used
%  4          msw        integer array          identifiers of structural modes followed
%  5          rho        real                   flight density
%  5          vrange     real array             vel step and maximum vel
%  6          plot_req   boolean                enable Vg plots
%  7          nfig       integer                OPTIONAL if plot required -> figure index 
%
%  OUTPUTS:   NAME       TYPE                   DESCRIPTION
%  1          FILE       aerof.log              ASCII file with flutter results
%  2          vf         real                   flutter speed EAS m/s
%  3          hf         real                   flutter altitude km
%  CALLED FROM: 
%
%  CREATION BY: Luca Cavagna (luca.cavagna@polimi.it)
%               Dipartimento di Ingegneria Aerospaziale
%               Politecnico di Milano
%
%
%  CREATION DATE: 2008-01-16
%
%     MODIFICATIONS:
%     DATE        VERS   PROGRAMMER    DESCRIPTION
%
%
%  REFERENCES:
%  See flutter_model.m for further information
%
global fl_model;
%
% vf = [];
% hf = [];

if (nargin == 7)
  nfig = 100;
end

fid = 1;
nmodc = length(mbase);
%
fprintf(fid, '\nRunning flutter boundary solver for constant Mach assessment...\n');
%
fprintf(fid, '\n - Flight density: %g.',      rho);
fprintf(fid, '\n - Incremental velocity: %g.', vrange(1));
fprintf(fid, '\n - Maximum velocity: %g.',    vrange(2));
%
fprintf(fid, '\n - Modes used: ');
for n = 1:nmodc
  fprintf(fid, ' %d', mbase(n));
end
fprintf(fid, '.');
%
if (isempty(msw))
  msweep = mbase;
else
  msweep = msw;
end
%
fprintf(fid, '\n - Modes followed: ');
nmods = length(msweep);
for n = 1:nmods
  fprintf(fid, ' %d', msweep(n));
end
%***********************************************************************************************************************
% Set parameters
fl_model = method_param(fl_model);
% Set velocity range
fl_model.param.DVEL = vrange(1);
fl_model.param.VMAX = vrange(2);
%
SVQU = zeros(13,1);
SVQU(13) = fl_model.param.VMAX;
SVQU(12) = rho;
% Check if previous Mach number solutions are available
if (nres == 1)
  fl_model.Res = [];
  fl_model.Res.data = [];
end
%
%***********************************************************************************************************************
% Load structural and aerodynamic data
[Mhh, Chh, Khh, MFreq, MID, Qhh, Klist, Mach, lref, err] = load_ffa_data(strf, aerof);
if (err ~= 0)
  fprintf('\n Error occoured.\n\n');
  return;
end
fl_model.aero.Qhh = Qhh;
fl_model.aero.Mach = Mach;
fl_model.aero.Klist = Klist;
fl_model.aero.rho = rho;
fl_model.ref.lref = lref;
fl_counter = 0;
%
[fl_model.struct.Mhh, fl_model.struct.Chh, fl_model.struct.Khh, ...
 fl_model.struct.MFreq, fl_model.struct.MID, fl_model.aero.Qhh, err] = ...
                 select_modal_base(Mhh, Chh, Khh, MFreq, Qhh, MID, mbase, msweep);
%
if (err ~= 0)
  fprintf('\n Error occoured.\n\n');
  return;
end
fl_model.struct.SID = msweep;
%
%***********************************************************************************************************************
% Build interpolation coefficients
fl_model.interp.YD = [];
fl_model.interp.FAD = [];
fl_model.interp.RM = [];
%
[fl_model.interp.YD, fl_model.interp.FAD, fl_model.interp.RM, fl_model.aero.Qhh] =...
 spline_coefficients(fl_model.aero.Klist, fl_model.aero.Qhh);
%
% Open result file
dot_pos = find(aerof == '.');
res_file = [aerof(1:dot_pos(end)), 'log'];
fp = open_res_file(res_file);
% plot results is required
% if (plot_req) 
% 
%   figure(nfig); close; figure(nfig);
%   fcount = 1;
%   fmax = size(fl_model.param.linestyle,1);
%   subplot(2,1,1)
%   grid on;
%   hold on;
%   subplot(2,1,2)
%   grid on;
%   hold on;
% end
%***********************************************************************************************************************
% Flutter solution
fprintf(fid, ' - Flutter tracking started.');

MFreq = fl_model.struct.MFreq.*(2*pi);
% allocate output data
fl_model.param.IMODFL(nres).data = [];
%fl_model.param.VFLINT(nres) = [];
fl_model.param.IMODEG(nres).data = [];
%
IMODFL = 0;

for n = 1:nmods
%
  fprintf(fid, '\n    Mode: %d.', fl_model.struct.SID(n));
%
  index = find(fl_model.struct.SID(n) == fl_model.struct.MID); 
%   
  [FL_FOUND, OUT(n).ris, SVTNFL, SVQU, IMODFL, IVAR, NSTOP, OUT(n).factor,OUT(n).perm,OUT(n).base] = ...
                                      mode_sweepOPT(n, fl_model.struct.Mhh, fl_model.struct.Chh*fl_model.struct.Khh, fl_model.struct.Khh, ...
                                      fl_model.aero.Qhh, fl_model.aero.Klist, fl_model.aero.rho, fl_model.ref.lref, fl_model.struct.SID(n), ...
                                      fl_model.struct.MID, MFreq(index), SVQU, IMODFL); 
% 
%   if (IVAR)
%     error('Error in flutter tracking (code %d, %s).', IVAR, NSTOP);
%   end
%
%  PLOT V-g diagrams
%
%   if (plot_req)
%  
%     subplot(2,1,1)
%     plot(res(:,2),res(:,4),fl_model.param.linestyle(fcount,:));
%     title('Velocity-Frequency')
%     ylabel('Frequency');
% %
%     subplot(2,1,2)
%     plot(res(:,2),res(:,3),fl_model.param.linestyle(fcount,:));
%     title('Velocity-Damping')
%     ylabel('Damping')
% 
%     fcount = fcount + 1; 
%     if (fcount > fmax) 
%       fcount = 1;
%     end
% %
%     freq = num2str(fl_model.struct.MFreq(index));
%     leg{n} = strcat(freq,' Hz ');
% %
%   end
%
%   FIELD_L = 20;
%   fprintf(fp,'\n\nMODE NUMBER: %d, OMEGA: %.6f.\n\n', fl_model.struct.SID(n),fl_model.struct.MFreq(index));
%   fprintf(fp,'Reduced Frequency   Velocity            Damping             Frequency           Real(Eig)           Imag(Eig)           Damp_Vder           Freq_Vder\n\n');
%   nfp = size(res,1);
% %
% % store results
%   fl_model.Res.data(nres).k{n}        = res(:,1);
%   fl_model.Res.data(nres).Velocity{n} = res(:,2);
%   fl_model.Res.data(nres).g{n}        = res(:,3);
%   fl_model.Res.data(nres).Freq{n}     = res(:,4);
%   fl_model.Res.data(nres).RealE{n}    = res(:,5);
%   fl_model.Res.data(nres).ImagE{n}    = res(:,6);
%   fl_model.Res.data(nres).g_Vder{n}   = res(:,7);
%   fl_model.Res.data(nres).F_Vder{n}   = res(:,8);
% %
%   for j=1:nfp
%     str = num2str(res(j,1), '%.6f'); res1_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,2), '%.6f'); res2_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,3), '%.6f'); res3_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,4), '%.6f'); res4_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,5), '%.6f'); res5_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,6), '%.6f'); res6_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,7), '%.6f'); res7_str = [str, blanks(FIELD_L-length(str))];
%     str = num2str(res(j,8), '%.6f'); res8_str = [str, blanks(FIELD_L-length(str))];
%     fprintf(fp,'%s%s%s%s%s%s%s%s\n', res1_str, res2_str, res3_str, res4_str, res5_str, res6_str, res7_str, res8_str);
%   end;
% %
%   if (FL_FOUND)
%     fl_counter = fl_counter+1;
%     fl_model.param.IMODFL(nres).data = [fl_model.param.IMODFL(nres).data, IMODFL];
%     fl_model.param.IMODEG(nres).data(:,fl_counter) = SVTNFL;
%   end

end 
%
%   
pipoo = 1;     
% if (SVQU(13) < fl_model.param.VMAX) 
%   fprintf(fid, '\n    !! Warning: Lowest flutter detected at %g. !!', min(imag(fl_model.param.IMODEG(nres).data(end,:))));
% else
%   fprintf(fid, ' No flutter detected.'); 
% end
% 
% fprintf(fid, '\n completed.');
% if (plot_req) 
%   subplot(2,1,1);   
%   legend(leg); 
% end
% %
% fl_model.param.VFLINT(nres) = SVQU(13);
% %
% if ((fl_model.param.VFLINT(nres) < fl_model.param.VMAX) && (plot_req))
% %
%   subplot(2,1,2)
%   plot(fl_model.param.VFLINT(nres),0,'ro');
%   fl_text = strcat(' \leftarrow Flutter Speed: ', num2str(fl_model.param.VFLINT(nres)));
%   text(fl_model.param.VFLINT(nres), 0, fl_text, 'FontSize', 14);
% %
% end
% %
% fprintf(fp,'\n\n\n                    FINAL RESULTS\n');
% %
% if (SVQU(13) >= fl_model.param.VMAX)
%   fprintf(fp,'\nFLUTTER NOT FOUND');
% else
%   fprintf(fp,'\nFLUTTER VELOCITY (Interpolated)              %8.5f', fl_model.param.VFLINT(nres));
%   fprintf(fp,'\nFLUTTER MODES :                              %d\n',  fl_model.param.IMODFL(nres).data);
% end
% %
% %***********************************************************************************************************************
% % Calculate divergence pressure and velocity
% fprintf(fid, '\n - Divergence tracking started.');
%
% [Div_P, Div_V] = aero_divergence(fl_model.aero.rho, fl_model.aero.Qhh, fl_model.struct.Khh, fl_model.aero.Klist);
% %
% if (Div_P < 0)
%   fprintf(fp,'\nDIVERGENCE NOT FOUND');
%   fprintf(fid, '\n     Divergence not found.');
% else
% %
% fprintf(fp,'DIVERGENCE DYNAMIC PRESSURE :                %g\n', Div_P);
% fprintf(fp,'DIVERGENCE VELOCITY:                         %g\n', Div_V);
% fprintf(fid, '     Divergence found at %g.', Div_V);
% if (plot_req)
%   subplot(2,1,2)
%   plot(Div_V,0,'ro');
%   div_text = strcat('Divergence Speed: ',num2str(Div_V));
%   text(Div_V,0, div_text);
% end
% %
% fl_model.param.VFLINT(nres) = min(fl_model.param.VFLINT(nres), Div_V);
% end
% %
% fprintf(fid, '\n completed.');
% %
% fclose(fp);
% fprintf(fid, '\n - Results exported to %s file.\n', res_file);
%
% if (fl_model.param.VFLINT(nres) < fl_model.param.VMAX)
% 
%   for n = 1:nmods
%   
%    i = find(fl_model.Res.data(nres).g{n}(:)>0);  
%    %
%    if (~isempty(i))
%       fprintf(fid, '\n\tFollowed mode %d in unstable.', msw(n));
%       VFI = interp1([fl_model.Res.data(nres).g{n}(i(1)-1),fl_model.Res.data(nres).g{n}(i(1))],...
%             [fl_model.Res.data(nres).Velocity{n}(i(1)-1),fl_model.Res.data(nres).Velocity{n}(i(1))],...
%             0.0);
%       [v, h] = flutter_envelope(VFI, rho, Mach);
%       vf = [vf, v];
%       hf = [hf, h];
% 
%    end 
% 
%   end
% end
