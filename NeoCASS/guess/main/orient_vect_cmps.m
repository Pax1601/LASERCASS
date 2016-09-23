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

function [X1, X2, X3] = orient_vect_cmps(N, P, corr)
% 
% Calculate vector components X1, X2 ,X3 to define orientation of each single element in order to write CBAR
% INPUT: N, nodes coordinate
%        P, aerodynamic patch corner
%        corr, number of beams within one aero patch
% 


[row, col] = size(N);   % col = nr of nodes
[rowP, colP] = size(P);
colP = colP / 4;        % number of aero patches

% initialize
xel = zeros(3, 1);     % along beam length
yel = zeros(3, 1);     % 
zel = zeros(3, 1);     % 
X1  = zeros(1, col-1); % x component of v vector
X2  = zeros(1, col-1); % 
X3  = zeros(1, col-1); % 
NOR = [];


% extract normal to aero patch
for j = 1:colP
    
    t = 4*(j-1);
    v1(:,1) = P(:,3+t) - P(:,1+t);
    v2(:,1) = P(:,2+t) - P(:,4+t);
    % normal to aero patch
    n(:,1) = cross(v1(:,1), v2(:,1));
    n(:,1) = n(:,1) ./ norm(n(:,1));
    % extend normal to aero patch over beam elements
    Nadd = n(:,1) * ones(1, corr(j));
    NOR = [NOR, Nadd];
    
end

[rowN, colN] = size(NOR);
if col-1-colN > 0
    NOR = [NOR(:,1) * ones(1,col-1-colN), NOR];
end

% extract element vector over beam elements
for i = 1:col-1
    
    % xel
    xel(:,1) = N(:,i+1) - N(:,i);
    xel = xel ./ norm(xel);

    % yel
    yel = NOR(:,i);
    
    % zel
    zel = cross(xel, yel);
    zel = zel ./ norm(zel);
    
    X1(1,i) = yel(1);
    X2(1,i) = yel(2);
    X3(1,i) = yel(3); 
    
    if abs(X1(i)) < 1e-14
        X1(i) = 0.0;
    end
    if abs(X2(i)) < 1e-14
        X2(i) = 0.0;
    end
    if abs(X3(i)) < 1e-14
        X3(i) = 0.0;
    end  
    
end
