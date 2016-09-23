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


A = load('n64a206.dat');
I = find(A(:,1)==0 | A(:, 1)==1.);
header = zeros(1, 2);
up_data = A(I(1):I(2), :);
low_data = A(I(3):I(4), :);
header(1) = size(up_data, 1);
header(2) = size(low_data, 1);
out_data = [flipud(up_data); low_data];
fid = fopen('n64a206_new.dat', 'w');
fprintf(fid, '%2d %2d\n', header);
fprintf(fid, '%12.8f %12.8f\n', out_data');
fclose(fid) 
