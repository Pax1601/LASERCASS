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

function [X1, X2, X3] = VectOrien(N)

% VectOrien.m calculates vector components X1, X2 ,X3 to define orientation of
% each single element in order to write CBAR
%

[row, col] = size(N);   % col = nr of nodes

% Vector xel, along element
xel = zeros(3,1);
% Vector yel, calculated from cross product between zel and xel
yel = zeros(3,1);
% Vector zel, belongs to X-Y plane and perpendicular to xel
zel = zeros(3,1);
% Components to define element orientation for CBAR card
X1  = zeros(1,col-1);   % row vector
X2  = zeros(1,col-1);
X3  = zeros(1,col-1);

for i = 1:col-1 % extract nr of segments
    %
    % Vector xel, along element
    xel(1) = [N(1,i+1)-N(1,i)];
    xel(2) = [N(2,i+1)-N(2,i)];
    xel(3) = [N(3,i+1)-N(3,i)];
    xel = xel./norm(xel);
    %
    % Vector zel,  belongs to X-Y plane and perpendicular to xel
    zel(1) =  0;
    zel(2) = -1;
    zel(3) =  0;
    %
    % Vector yel, calculated from cross product between zel and xel
    yel = cross(zel, xel);
    %%%%%%%%%%%%%%%%%%%% NEW 2008-05-18
    normyel = norm(yel);
    yel = yel./normyel;
    %%%%%%%%%%%%%%%%%%%%
    X1(i) = yel(1);
    X2(i) = yel(2);
    X3(i) = yel(3); 
    
    %%%%%%%%%%%%%%%%%%%%% NEW 2008-05-08
    if abs(X1(i)) < 1e-14
        X1(i) = 0.0;
    end
    if abs(X2(i)) < 1e-14
        X2(i) = 0.0;
    end
    if abs(X3(i)) < 1e-14
        X3(i) = 0.0;
    end    
    %%%%%%%%%%%%%%%%%%%%%
    
end
