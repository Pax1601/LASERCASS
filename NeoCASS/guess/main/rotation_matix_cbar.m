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

function [R] = rotation_matix_cbar(N, v)

% row = 3; col = sectors
[row, col] = size(N);
xel = zeros(3,col-1);
yel = zeros(3,col-1);
zel = zeros(3,col-1);
R = zeros(3,3,col-1);

for i = 1:col-1
    xel(:,i) = N(:,i+1)-N(:,i);
    xel(:,i) = xel(:,i)./norm(xel(:,i));
    zel(:,i) = cross(xel(:,i), v(:,i));
    zel(:,i) = zel(:,i)./norm(zel(:,i));
    yel(:,i) = cross(zel(:,i), xel(:,i));
    yel(:,i) = yel(:,i)./norm(yel(:,i));
end

% matrix
for i = 1:col-1
    R(:,:,i) = [xel(:,i), yel(:,i), zel(:,i)];
end
