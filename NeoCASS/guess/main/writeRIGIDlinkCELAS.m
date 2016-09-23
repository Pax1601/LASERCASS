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
function writeRIGIDlinkCELAS(fid, stick, geo, pdcylin)

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n Rigid structural link_thicks');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
% first define a material with stiffness constants 10000 time major than
% that of wing and density 1000 time lower.

% The link_thick's element is circular with radius = 5% of fuselage radius

if stick.model.fuse
    R = geo.fus.R*0.05;
    A = pi*R^2;
    E = pdcylin.fus.es*10000;
    K=E*A;
    cont = 1;
else % tailbooms
    R = geo.tbooms.R*0.05;
    E = pdcylin.tbooms.es*10000;
    A = pi*R^2;
    K=E*A;
    cont = 1;
end
% extract master and slave nodes
for i = 1 : length(stick.link.Ma)
    N = length(stick.link.RBE2(i).slave);
    for j = 1 : N
        writeCELAS(fid,cont,K,stick.link.Ma(i),stick.link.RBE2(i).slave(j),stick.link.RBE2(i).DOF);
        cont = cont+1;
    end
    
end

fprintf(fid, '\n');
end
function writeCELAS(fid,ID,K,G1,G2,DOF)
fprintf(fid, 'CELAS   ');

%--------------------------------------------------------------------------
% Define field 2: ID
%
[IDstr] = cnvt2_8chs(ID);
fprintf(fid, '%c', IDstr);

%--------------------------------------------------------------------------
% Define field 3: K
%
[Kstr] = cnvt2_8chs(K);
fprintf(fid, '%c', Kstr);

%--------------------------------------------------------------------------
% Define field 4: G1
%
[G1str] = cnvt2_8chs(G1);
fprintf(fid, '%c', G1str);

%--------------------------------------------------------------------------
% Define field 4: G2
%
[G2str] = cnvt2_8chs(G2);
fprintf(fid, '%c', G2str);

%--------------------------------------------------------------------------
% Define field 4: DOF
%

fprintf(fid, '%s', DOF);fprintf(fid,'\n');

end