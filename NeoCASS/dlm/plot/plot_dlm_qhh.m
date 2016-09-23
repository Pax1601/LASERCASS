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
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function plot_dlm_qhh(dlm_model, row, col, Mach_index, nfig)

if (nargin == 4)
  nfig = 1;
end

nrows = size(dlm_model.data.Qhh, 1);
ncols = size(dlm_model.data.Qhh, 2);
nmach = size(dlm_model.data.Qhh, 4);

if (row > nrows)
  fprintf(1, '\n - Required row index exceeds matrix size.');
  return;
end
if (col > ncols)
  fprintf(1, '\n - Required column index exceeds matrix size.');
  return;
end
if (Mach_index > nmach)
  fprintf(1, '\n - Required Mach index matrix size.');
  return;
end

figure(nfig); close; H=figure(nfig);
set(H, 'Visible','on', 'Name', 'NeoCASS - Aerodynamic matrix plot', 'NumberTitle','off', 'MenuBar','none'); 
nk = length(dlm_model.aero.k);

hold on;
xlabel('k');
ylabel('Q_{hh}');
data(1:nk) = dlm_model.data.Qhh(row, col, 1:nk, Mach_index);
plot(dlm_model.aero.k, real(data), ...
     'ok-', 'LineWidth', 1, 'MarkerSize', 6, 'MarkerFaceColor','k');
plot(dlm_model.aero.k, imag(data), ...
     'sr-', 'LineWidth', 1, 'MarkerSize', 6, 'MarkerFaceColor','r');

label1 = ['Re Q_{hh} (', num2str(row), ',', num2str(col),')'];
label2 = ['Im Q_{hh} (', num2str(row), ',', num2str(col),')'];

H = legend(label1, label2, 'Location', 'BestOutside');
set(H, 'EdgeColor', [1 1 1])

labelt = ['Mach ', num2str(dlm_model.aero.M(Mach_index))];
title(labelt);
