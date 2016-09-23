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

function AI_plot(ifig, IND, NLEVEL)
% ifig is the index of figure
% IND is the index of internal load that is needed
% IND = 1 : local Tx; (axial)
% IND = 2 : local Ty; (vertical shear)
% IND = 3 : local Tz; (horiz shear)
% IND = 4 : local Mx; (torsion)
% IND = 5 : local My; (out of plane bending)
% IND = 6 : local Mz; (int of plane bending)
% NLEVEL number of levels in contour
global beam_model

switch(IND)
  case 1
  name = 'Axial force [KN]'
  case 2
  name = 'Out of plane shear force [KN]'
  case 3
  name = 'In plane shear force [KN]'
  case 4
  name = 'Torsional moment [KNm]'
  case 5
  name = 'In plane bending [KNm]'
  case 6
  name = 'Out of plane bending [KNm]'
  otherwise
    error('Output index is between 1 and 6.');
end
nb = beam_model.Info.nbar;
CForces = beam_model.Res.Bar.CForces;
SCALE= 1000; % use KN as units
for i=1:nb
  AI(1,i) = CForces(1,IND,i);
  AI(2,i) = CForces(2,IND,i);
end
AI = AI ./SCALE;
% find the maximum of internal load chosen
AImax = max( max( abs(AI),[],2 ) );
figure(ifig); close; figure(ifig);
hold on
caxis('auto');
v = caxis;
AImax2 = max( max( (AI),[],2 ) );
AImin = min( min( (AI),[],2 ) );
v1 =    0;
v2 =    (AImax2-AImin)/AImax2*beam_model.Aero.ref.C_mgc
caxis([v1 v2]);
for i = 1 : nb
  Coord = beam_model.Bar.Colloc(:,:,i);
  Coord2(1,:) = Coord(1,:) + beam_model.Bar.Orient(i,:)*AI(1,i)/AImax2*beam_model.Aero.ref.C_mgc;
  Coord2(2,:) = Coord(2,:) + beam_model.Bar.Orient(i,:)*AI(2,i)/AImax2*beam_model.Aero.ref.C_mgc;
  X = [Coord(1,1),Coord(2,1);Coord2(1,1),Coord2(2,1)];
  Y = [Coord(1,2),Coord(2,2);Coord2(1,2),Coord2(2,2)];
  Z = [Coord(1,3),Coord(2,3);Coord2(1,3),Coord2(2,3)];
  mv = 0.5*(AI(1,i)+AI(2,i))/AImax2*beam_model.Aero.ref.C_mgc;
  C = (0.5*(AI(1,i) + AI(2,i)) - AImin)/AImax2*beam_model.Aero.ref.C_mgc;
  surface(X,Y,Z,C);
end
view(3); axis equal;
% add color bar
v = caxis;
bar =  mat2cell( linspace(AImin,AImax2,NLEVEL) , 1 , ones(NLEVEL,1));
colorbar('YTick',linspace(v(1),v(2),NLEVEL)','YLim',[v(1) v(2)],'YTickLabel',bar);
title(name);
