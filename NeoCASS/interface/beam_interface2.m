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
% str_data: str coord Ns x 3
% aero_data: aero coord Na x 3
% toll: tolerance for pinv
% ORIG: reference origin 1 x 3
% Rmat: transformation matrix 3 x 3 from global to local
%
function H = beam_interface2(str_data, aero_data, toll, ORIG, Rmat) 
  for i=1: size(str_data,1)
    str_data(i,:) = (Rmat * (str_data(i,:) - ORIG)')';
  end
  for i=1: size(aero_data,1)
    aero_data(i,:) = (Rmat * (aero_data(i,:) - ORIG)')';
  end

 [H]  = assembly_H_mat(str_data, aero_data, toll);

end
%***********************************************************************************************************************
function s = assembly_H_mat(cent, aer, toll)
%
  sol = assembly_coeff(cent, toll,aer);
%
  EJ = 1;
  GJ = 1;
  EJ12 = 12 * EJ;
  EJ4 = 4 * EJ;
  EJ2 = 2 * EJ;
  GJ2 = 2 * GJ;
%
  naer = size(aer, 1);
  nstr = size(cent, 1);
  pp = zeros(3*naer, 3*nstr+3);
%
  xaer=aer(:,1); xcent=cent(:,1);
  yaer=aer(:,2); ycent=cent(:,2);
%
  for i = 1:naer
    for j = 1:nstr
      DY = yaer(i)-ycent(j);
      ABS = abs(DY);
%     first row
      pp(i,j) = (ABS^3) ./ EJ12 - xaer(i) * xcent(j) * ABS/GJ2;
      pp(i,j + nstr) = -(DY* ABS) / EJ4;
      pp(i,j + 2*nstr) = xaer(i) * ABS / GJ2;
%     second row
      pp(i+naer,j) = (DY * ABS) / EJ4;
      pp(i+naer,j + nstr) = -ABS / EJ2;
%     third row
      pp(i+2*naer,j) = xcent(j) * abs(yaer(i)-ycent(j))/ GJ2;

      pp(i+2*naer,j + 2*nstr) = -ABS / GJ2;
%     poly coeff
      pp(i,3*nstr+1) = 1; pp(i,3*nstr+2) = yaer(i); pp(i,3*nstr+3) = -xaer(i);
      pp(i+naer,3*nstr+2) = 1;
      pp(i+2*naer,3*nstr+3) = 1;
    end
  end
%
 s = pp * sol;
end
%***********************************************************************************************************************
function coeff = assembly_coeff(cent, toll, aer)
%
  if (toll == 0)
    toll = realmax;
  end
  nstr = size(cent, 1);
%  coeffB = assembly_coeffB(cent(:,2)', toll);
%  coeffT = assembly_coeffT(cent(:,2)', toll);
%  [k1, k2] = size(coeffB);
%  [k3, k4] = size(coeffT);
%  coeff = [coeffB(1:end-2,:), zeros(k1-2,k4) ;... 
%           zeros(k3-1, k2), coeffT(1:end-1,:); ...
%           coeffB(end-1:end,:), zeros(2, k4); ...
%           zeros(1,k2) coeffT(end,:)];
% return
  EJ = 1;
  GJ = 1;
  EJ12 = 12 * EJ;
  EJ4 = 4 * EJ;
  EJ2 = 2 * EJ;
  GJ2 = 2 * GJ;
  ycent = cent(:,2);
  xcent = cent(:,1);

  for i = 1:nstr
    for j = 1:nstr
      DY = ycent(i)-ycent(j);
      ABS = abs(DY);
%     first row
      A(i,j) = (ABS^3) ./ EJ12 - xcent(i) * xcent(j) * ABS/GJ2;
      A(i,j + nstr) = -(DY* ABS) / EJ4;
      A(i,j + 2*nstr) = xcent(i) * ABS / GJ2;
%     second row
      A(i+nstr,j) = (DY * ABS) / EJ4;
      A(i+nstr,j + nstr) = -ABS / EJ2;
%     third row
      A(i+2*nstr,j) = xcent(j) * abs(ycent(i)-ycent(j)) / GJ2;
      A(i+2*nstr,j + 2*nstr) = -ABS / GJ2;
    end
  end
%
  invA = pinv(A, 1/toll);
%
  P = [ones(1, nstr), zeros(1,2*nstr); ...
       ycent', ones(1,nstr), zeros(1,nstr); ...
      -xcent', zeros(1,nstr), ones(1,nstr)];
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
  coeff = [rbf ; alfa];
%
end
%***********************************************************************************************************************
function coeff = assembly_coeffB(cent, toll)
%
  if (toll == 0)
    toll = realmax;
  end
  nstr = length(cent);
  R1T = [ones(1, nstr) ; cent];
  R2T = [zeros(1, nstr); ones(1, nstr)];
  P = [R1T, R2T];
  EJ = 1;
  EJ12 = 12 * EJ;
  EJ4 = 4*EJ;
  EJ2 = 2*EJ;
%
  A11 = zeros(nstr, nstr);
  A12 = zeros(nstr, nstr);
  A22 = zeros(nstr, nstr);
  for j=1:nstr
    for k=1:nstr
      DY = cent(j)-cent(k);
      ABS = abs(DY);
      A11(j,k) = (ABS^3) / EJ12;
      A12(j,k) = -(DY*ABS) / EJ4;
      A22(j,k) = -ABS/ EJ2;
    end
  end
  A =[ A11, A12; -A12, A22];
  invA = pinv(A, 1/toll);
%
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
  coeff = [rbf ; alfa];
%
end
%***********************************************************************************************************************
function coeff = assembly_coeffT(cent, toll)
%
  if (toll == 0)
    toll = realmax;
  end
  nstr = length(cent);
  P = [ones(1, nstr)];
  GJ = 1;
  GJ2 = 2*GJ;
%
  A = zeros(nstr, nstr);
  for j=1:nstr
    for k=1:nstr
      A(j,k) = -(abs(cent(j)-cent(k)).^3) ./ GJ2;
    end
  end
  invA = pinv(A, 1/toll);
%
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
  coeff = [rbf ; alfa];
%
end