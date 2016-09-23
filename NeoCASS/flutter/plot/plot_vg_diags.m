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

function plot_vg_diags(nfig, res, nset, Freq, Mach, linestyle)

if nset <= length(res.data)

  fcount = 1;
  freq = [];
  fmax = size(linestyle,1);

  figure(nfig); close; H = figure(nfig);
  set(H, 'Visible','on','MenuBar','none', 'Name', 'NeoCASS - Flutter diagrams', 'NumberTitle','off'); 
  
  NMODES = length(res.data(nset).k);

  for n=1:NMODES

    subplot(2,1,1); hold on;
    colour = linestyle(fcount,1);
    plot( (res.data(nset).Velocity{n}), (res.data(nset).Freq{n}), linestyle(fcount,:), ...
         'MarkerSize', 4, 'MarkerFaceColor', colour);
    title('V-F')
    ylabel('Frequency');
    xlabel('Velocity');
  %
    subplot(2,1,2); hold on;
    plot( (res.data(nset).Velocity{n}), (res.data(nset).g{n}),linestyle(fcount,:), ...
         'MarkerSize', 4, 'MarkerFaceColor', colour);
    title('V-g')
    ylabel('Damping')
    xlabel('Velocity');

    fcount = fcount + 1;
    if (fcount > fmax) 
      fcount = 1;
    end
  %
    freq_str = num2str(Freq(n));
    leg(n,:) = {strcat(freq_str,' Hz ')};
%
  end
  subplot(2,1,1);   
  legend(leg, 'Location', 'EastOutside', 'EdgeColor',[1 1 1]);
  grid on;
  subplot(2,1,2);   
  grid on;
% look for flutter envelope data
  if ~isempty(res.Env)

    nfig = nfig + 1;
    figure(nfig); close; H=figure(nfig);
    set(H, 'Visible','on','MenuBar','none', 'Name', 'NeoCASS - Flutter envelope', 'NumberTitle','off'); 

    index = find(res.Env(:,1));

    subplot(2,1,1);
    plot(Mach(index), res.Env(index,1), '-ro','MarkerSize', 6, 'MarkerFaceColor', 'r');  
    axis([0 1.0 0.0 350]);
    ylabel('VF_{EAS}');
    xlabel('Mach'); 
    grid;

    subplot(2,1,2);
    plot(Mach(index), res.Env(index,2), '-ro','MarkerSize', 6, 'MarkerFaceColor', 'r');  
    hold on;
    plot([0 1], [11 11],'-k');  
    plot([0 1], [25 25],'-k');  
    plot([1 1], [0 25],'-k');  
    axis([0 1.05 0.0 26]);
    ylabel('Altitude');
    xlabel('Mach');
    grid;
    set(gca,'XTick',[0.0:0.1:1]');
    set(gca,'YTick',[0.0:5:26]')

  end

else

  fprintf(1, '\n - Required set not available in flutter database.');

end
