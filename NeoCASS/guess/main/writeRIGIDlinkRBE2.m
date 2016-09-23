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

% This function defines the igid link_thicks between the different struct
% Travaglini 30/october 2009
function writeRIGIDlinkRBE2(fid, stick, LEVEL)

if (LEVEL == 0)
  fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fid, '\n$ Rigid structural link');
  fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  cont = 1;
% extract master and slave nodes
  for i = 1 : length(stick.link_thick.Ma_thick)
    BULKdataRB2(fid,cont,stick.link_thick.Ma_thick(i),stick.link_thick.RBE2(i).DOF,stick.link_thick.RBE2(i).slave);
    cont = cont+1;
  end
%
% Append nodes for extra points (landing gears)
%
  cont = 0;
  for i=1:length(stick.ID.extrar_thick)
    c = stick.nodes.extrar_thick(:,i);
    cont = cont+1;
    BULKdataGRID(fid, stick.ID.extrar_thick(i), 0, c(1), c(2), c(3), 0, 0, 0);
    BULKdataCONM2(fid, cont, stick.ID.extrar_thick(i), eps, eps, eps, eps, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  end      
  for i=1:length(stick.ID.extral_thick)
    cont = cont+1;
    c = stick.nodes.extral_thick(:,i);
    BULKdataGRID(fid, stick.ID.extral_thick(i), 0, c(1), c(2), c(3), 0, 0, 0);
    BULKdataCONM2(fid, cont, stick.ID.extral_thick(i), eps, eps, eps, eps, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  end      
else
  fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fid, '\n$ Rigid structural link');
  fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  cont = 1;
  for i = 1 : length(stick.link.Ma)
    BULKdataRB2(fid,cont,stick.link.Ma(i),stick.link.RBE2(i).DOF,stick.link.RBE2(i).slave(:));
    cont = cont+1;
  end

end
%
fprintf(fid, '\n');
end