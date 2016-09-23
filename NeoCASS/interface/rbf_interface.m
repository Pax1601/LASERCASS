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

function H = rbf_interface(str_data, aero_data, type, rmax, toll) 

  H = assembly_H_mat(str_data, aero_data, type, rmax, toll);

end
%***********************************************************************************************************************
function s = assembly_H_mat(cent, aer, type, rmax, toll)
%
  sol = assembly_coeff(cent, type, rmax, toll);
%
  [n dim] = size(cent);
  offset1 = n + 1;
  offset2 = n + 2;
%
  s = zeros(size(aer,1), n);
%
  naer = size(aer, 1);
  pp = zeros(1, offset1 + dim);
%
  for i = 1:naer
    for j = 1:n
      pp(1,j) = phi(  norm(aer(i,:) - cent(j,:) ), type, rmax);
    end
    pp(n+1) = 1;
    pp(offset2 : offset1 + dim) = aer(i,1:dim);    
    s(i,:) = pp * sol;
  end
end
%***********************************************************************************************************************
function coeff = assembly_coeff(cent, type, rmax, toll)
%
  if (toll == 0)
    toll = realmax;
  end

  [n dim] = size(cent);
  A = zeros(n, n);
  for i = 1:n
      for j = 1:i
          r = norm(cent(i,:) - cent(j,:));
          temp = phi(r, type, rmax);
          A(i,j) = temp;
          A(j,i) = temp;
      end
  end
%
  P = [ones(1,n) ; cent'];
  invA = pinv(A, 1/toll);
%
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
%
  coeff = [rbf ; alfa];
end
%***********************************************************************************************************************
function u = phi(dist, type, rmax)
%
  err = 1e-20;
  u = 0;
%
  switch (type)
%-----------------------------------------------------------------------
    case 1 % VOLUME SPLINE
      u = dist;
%-----------------------------------------------------------------------
    case 2 % THIN PLATE SPLINE
      if (dist > err)
        u = dist^2*log(dist);
      end
%-----------------------------------------------------------------------
    case 3 % GAUSSIAN
      alpha = 1.0;
      u = exp(-alpha * dist);
%-----------------------------------------------------------------------
    case 4 % EUCLID HAT
      if (dist < 2*rmax)
        u = pi * ( (1/12 * dist^3) - ...
            (rmax^2 * dist) + (4/3*rmax^3) );
      end
%-----------------------------------------------------------------------
    case 5 % WENDLAND C0
      if (dist < rmax)
        u = (1 - dist/rmax)^2;
      end
%-----------------------------------------------------------------------
    case 6 % WENDLAND C2
      if (dist < rmax)
        d = dist/rmax;
        u = (4*d + 1)* (1 - d)^4;
      end
%-----------------------------------------------------------------------
    case 7 % WENDLAND C4
      if (dist < rmax)
        d = dist/rmax;
        u = (35*d^2 + 18 * d + 3)* (1 - d)^6;
      end
%-----------------------------------------------------------------------
    case 8 % WENDLAND C6
      if (dist < rmax)
        d = dist/rmax;
        u = (32*d^3 + 25 * d^2 + 8 * d + 1)* (1 - d)^8;
      end
%-----------------------------------------------------------------------
  end
end
