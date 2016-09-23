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


function plot_section(nfig, NODE, BETA, R, CG, SC)

scale = 1.2;

figure(nfig); close; figure(nfig);
hold on;
NODE = NODE(:, 1:2);
ns = size(NODE, 1);
np = size(BETA, 1);

plot(NODE(:,1), NODE(:,2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor','k');
plot(CG(1), CG(2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor','r');
plot(SC(1), SC(2), 'b+', 'MarkerSize', 8, 'MarkerFaceColor','b');

COORD = NODE - repmat(CG, ns, 1);
dist = max(sqrt(sum(COORD .* COORD, 2)));

P1 = [dist ; 0];
P2 = [0 ; dist];

P1 = R * P1;
P2 = R * P2;

plot([CG(1) CG(1)+P1(1)], [CG(2) CG(2)+P1(2)], 'k-');
plot([CG(1) CG(1)+P2(1)], [CG(2) CG(2)+P2(2)], 'k-');

for n=1:np;
  plot([NODE(BETA(n,1),1) NODE(BETA(n,2),1)], [NODE(BETA(n,1),2) NODE(BETA(n,2),2)], ...
       '-r', 'LineWidth',2)
end

V = axis;
axis(scale.*V);
