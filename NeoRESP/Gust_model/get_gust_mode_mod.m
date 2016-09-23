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
%  FFAST Project
%
%  NeoSYM
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Lorenzo Travaglini   <>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by FFAST partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%
%   Author: Lorenzo Travaglini
%***********************************************************************************************************************
function dwn = get_gust_mode_mod(lattice,gust, cref, k, Node, ngust)


colloc = lattice.COLLOC*cref;
midpoint = lattice.MID_DPOINT*cref;

if gust.DIR(ngust) == 3 % vertical gust
    
    b = max(Node.Coord(:,2));
    
    c_disp = zeros(lattice.np,3);
    
    for i = 1: lattice.np
        y = colloc(i,2)/b;
        c_disp(i,3) = eval(gust.funs{ngust}); 
    end
    
    mscale = max(max(abs(c_disp)));
    c_disp = c_disp ./ mscale;
    
elseif gust.DIR(ngust) == 2 %lateral gust
    
    b = max(Node.Coord(:,1));
    
    c_disp = zeros(lattice.np,3);
    
    for i = 1: lattice.np
        x = colloc(i,1)/b;
        c_disp(i,2) = eval(gust.funs{ngust}); 
    end
    
    mscale = max(max(abs(c_disp)));
    c_disp = c_disp ./ mscale;
    
else
    error('gust must be in z or y direction');
end

% nk = length(k);
% dwn = zeros(lattice.np,1,nk);
dwn = zeros(lattice.np,1);
CNDISPL = (dot(c_disp, lattice.N(1:lattice.np,:), 2));

% for j = 1 : nk
    
    dwn(:,1) =  -CNDISPL;%/gust.Vinfty(ngust);%.*exp(-(1i*k(j)/cref) *( midpoint(1:lattice.np,1) -gust.X0(ngust)) );
    
% end
