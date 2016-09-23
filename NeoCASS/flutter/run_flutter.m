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

function [vf, hf] = run_flutter(fid, Mhh, Chh, Khh, MFreq, MID, Qhh, Klist, Mach, lref, mbase, msw, rho, vrange, plot_req, nfig, nres)
%
%***********************************************************************************************************************
%
% FUNCTION: Flutter tracking at constant Mach number
%
%  INPUTS:    NAME       TYPE                   DESCRIPTION
%
%  1          fid        integer                output pointer
%  2          Mhh        real                   gen. mass matrix
%  3          Chh        real                   gen. damping matrix
%  4          Khh        real                   gen.  stiffness matrix
%  4          MFreq      real                   modal frequencies
%  5          MID        real                   mode ID associated
%  6          Qhh        complex                gen. aero matrix  
%  7          Klist      real                   red. frequencies
%  8          Mach       real                   Mach number
%  9          lref       real                   reference chord (c/2)
%  10         mbase      integer array          identifiers of structural modes used
%  11         msw        integer array          identifiers of structural modes followed
%  12         rho        real                   flight density
%  13         vrange     real array             vel step and maximum vel
%  14         plot_req   boolean                enable Vg plots
%  15         nfig       integer                OPTIONAL if plot required -> figure index 
%
%  OUTPUTS:   NAME       TYPE                   DESCRIPTION
%  1          vf         real                   flutter speed EAS m/s
%  2          hf         real                   flutter altitude km
%  CALLED FROM: 
%
%  CREATION BY: Luca Cavagna (luca.cavagna@polimi.it)
%               Dipartimento di Ingegneria Aerospaziale
%               Politecnico di Milano
%
%
%  CREATION DATE: 2012-07-13
%
%     MODIFICATIONS:
%     DATE        VERS   PROGRAMMER    DESCRIPTION
%
%
%  REFERENCES:
%
global fl_model;
%
vf = [];
hf = [];
%
nmodc = length(mbase);
%
fprintf(fid, '\n - Running flutter boundary solver for constant Mach assessment...\n');
%
fprintf(fid, '\n\t - Flight density: %g.',      rho);
fprintf(fid, '\n\t - Incremental velocity: %g.', vrange(1));
fprintf(fid, '\n\t - Maximum velocity: %g.',    vrange(2));
%
fprintf(fid, '\n\t - Modes used: ');
for n = 1:nmodc
  fprintf(fid, ' %d', mbase(n));
end
fprintf(fid, '.');
%
if (isempty(msw))
  msweep = mbase;
else
  [v, msweep] = intersect(MID, msw);
end
%
fprintf(fid, '\n\t - Modes followed: ');
nmods = length(msweep);
for n = 1:nmods
  fprintf(fid, ' %d', MID(msweep(n)));
end
fprintf(fid, '.');
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
fl_model.aero.Qhh = Qhh;
fl_model.aero.Mach = Mach;
fl_model.aero.Klist = Klist;
fl_model.aero.rho = rho;
fl_model.ref.lref = lref;
fl_counter = 0; 
%
[fl_model.struct.Mhh, fl_model.struct.Chh, fl_model.struct.Khh, ...
 fl_model.struct.MFreq, fl_model.struct.MID, fl_model.aero.Qhh, err] = ...
                 select_modal_base(Mhh, Chh, Khh, MFreq, Qhh, MID, mbase);
%
if (err ~= 0)
  fprintf('\n Error occoured.\n\n');
  return;
end
fl_model.struct.SID = MID(msweep);
%
%***********************************************************************************************************************
% Build interpolation coefficients
fl_model.interp.YD = [];
fl_model.interp.FAD = [];
fl_model.interp.RM = [];
%
[fl_model.interp.YD, fl_model.interp.FAD, fl_model.interp.RM, fl_model.aero.Qhh] =...
 spline_coefficients(fl_model.aero.Klist, fl_model.aero.Qhh);
% plot results is required
if (plot_req) 

  figure(nfig); close; figure(nfig);
  fcount = 1;
  fmax = size(fl_model.param.linestyle,1);
  subplot(2,1,1)
  grid on;
  hold on;
  subplot(2,1,2)
  grid on;
  hold on;
end
%***********************************************************************************************************************
% Flutter solution
fprintf(fid, '\n - Flutter tracking started.');
MFreq = fl_model.struct.MFreq.*(2*pi);
% allocate output data
fl_model.param.IMODFL(nres).data = []; 
%fl_model.param.VFLINT(nres) = [];
fl_model.param.IMODEG(nres).data = [];
%
IMODFL = 0;

for n = 1:nmods
%
  fprintf(fid, '\n\tMode: %d.', fl_model.struct.SID(n));
%
  index = find(fl_model.struct.SID(n) == fl_model.struct.MID);
% 
  [FL_FOUND, res, SVTNFL, SVQU, IMODFL, IVAR, NSTOP] = mode_sweep(n, fl_model.struct.Mhh, fl_model.struct.Chh, fl_model.struct.Khh, ...
    fl_model.aero.Qhh, fl_model.aero.Klist, fl_model.aero.rho, fl_model.ref.lref, fl_model.struct.SID(n), ...
    fl_model.struct.MID, MFreq(index), SVQU, IMODFL);
%
  
  if (IVAR)
    error('Error in flutter tracking (code %d, %s).', IVAR, NSTOP);
  end
%
%  PLOT V-g diagrams
%
  if (plot_req)
 
    subplot(2,1,1)
    plot(res(:,2),res(:,4),fl_model.param.linestyle(fcount,:));
    title('Velocity-Frequency')
    ylabel('Frequency');
%
    subplot(2,1,2)
    plot(res(:,2),res(:,3),fl_model.param.linestyle(fcount,:));
    title('Velocity-Damping')
    ylabel('Damping')

    fcount = fcount + 1; 
    if (fcount > fmax) 
      fcount = 1;
    end
%
    freq = num2str(fl_model.struct.MFreq(index),'%.3g');
    leg{n} = strcat(freq,' Hz ');
%
  end
% store results
  fl_model.Res.data(nres).k{n}        = res(:,1);
  fl_model.Res.data(nres).Velocity{n} = res(:,2);
  fl_model.Res.data(nres).g{n}        = res(:,3);
  fl_model.Res.data(nres).Freq{n}     = res(:,4);
  fl_model.Res.data(nres).RealE{n}    = res(:,5);
  fl_model.Res.data(nres).ImagE{n}    = res(:,6);
  fl_model.Res.data(nres).g_Vder{n}   = res(:,7);
  fl_model.Res.data(nres).F_Vder{n}   = res(:,8);
 
%
%
  if (FL_FOUND)
    fl_counter = fl_counter+1;
    fl_model.param.IMODFL(nres).data = [fl_model.param.IMODFL(nres).data, IMODFL];
    fl_model.param.IMODEG(nres).data(:,fl_counter) = SVTNFL;
  end

end 
%
%  

if (SVQU(13) < fl_model.param.VMAX) 
  fprintf(fid, '\n ### Warning: flutter detected at %g.', min(imag(fl_model.param.IMODEG(nres).data(end,:))));
else
  fprintf(fid, '\n - No flutter detected.'); 
end

fprintf(fid, '\n   completed.');
if (plot_req) 
  subplot(2,1,1);   
  legend(leg); 
end
%
fl_model.param.VFLINT(nres) = SVQU(13);
%
if ((fl_model.param.VFLINT(nres) < fl_model.param.VMAX) && (plot_req))
%
  subplot(2,1,2)
  plot(fl_model.param.VFLINT(nres),0,'ro');
  fl_text = strcat(' \leftarrow Flutter Speed: ', num2str(fl_model.param.VFLINT(nres)));
  text(fl_model.param.VFLINT(nres), 0, fl_text, 'FontSize', 14);
%
end
%
%
%***********************************************************************************************************************
% Calculate divergence pressure and velocity
fprintf(fid, '\n - Structural divergence tracking started (elastic modes only).');
%
[Div_P, Div_V] = aero_divergence(fl_model.aero.rho, fl_model.aero.Qhh, fl_model.struct.Khh, fl_model.aero.Klist, fl_model.struct.MID);
%
if (Div_P < 0)
  fprintf(fid, '\n - No divergence detected.');
else
%
  fprintf(fid, '\n   ### Warning: divergence detected at %g Pa.', Div_P);
%
fl_model.param.VFLINT(nres) = fl_model.param.VFLINT(nres);% min(fl_model.param.VFLINT(nres), Div_V);
end
%
fprintf(fid, '\n   completed.');
%
if (fl_model.param.VFLINT(nres) < fl_model.param.VMAX)

  for n = 1:nmods
  
   i = find(fl_model.Res.data(nres).g{n}(:)>0); 
   if (~isempty(i)) &&  i(1)==1
      i = i(2:end);
   end
   %
   if (~isempty(i))
      fprintf(fid, '\n\tMode %d in unstable.', msw(n));
      VFI = interp1([fl_model.Res.data(nres).g{n}(i(1)-1),fl_model.Res.data(nres).g{n}(i(1))],... 
            [fl_model.Res.data(nres).Velocity{n}(i(1)-1),fl_model.Res.data(nres).Velocity{n}(i(1))],...
            0.0);
      [v, h] = flutter_envelope(VFI, rho, Mach);
      vf = [vf, v];
      hf = [hf, h];

   end 

  end
end
