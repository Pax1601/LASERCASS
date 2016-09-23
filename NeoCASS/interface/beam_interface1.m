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

function [H, Ht] = beam_interface1(str_data, aero_data, toll) 

 [H, Ht]  = assembly_H_mat(str_data, aero_data, toll);

end
%***********************************************************************************************************************
function [s, st] = assembly_H_mat(cent, aer, toll)
%
  sol = assembly_coeff(cent, toll);
  solt = assembly_coefft(cent, toll);
  nstr  = length(cent);
%
  EJ = 1;
  GJ = 1;
  EJ12 = 12 * EJ;
  EJ4 = 4*EJ;
  EJ2 = 2*EJ;
  GJ2 = 2*GJ;
%
  naer = length(aer);
% displacement and bending interpolation
  pp = zeros(2*naer, 2 + 2*nstr);
%
  for i = 1:naer
    for j = 1:nstr
      pp(i,j) = (abs(aer(i)-cent(j)).^3) ./ EJ12;
      pp(i,j + nstr) = -(((aer(i)-cent(j))).*abs(aer(i)-cent(j))) ./ EJ4;
      pp(i+naer,j) = (((aer(i)-cent(j))).*abs(aer(i)-cent(j))) ./ EJ4;
      pp(i+naer,j + nstr) = -(abs((aer(i)-cent(j))))./ EJ2;
      pp(i,end) = aer(i);
      pp(i,end-1) = 1;
      pp(i+naer,end) = 1;
      pp(i+naer,end-1) = 0;
    end
  end
  s = pp * sol;
% torque interpolation
  ppt = zeros(naer, 1 + nstr);
  for i = 1:naer
    for j = 1:nstr
      ppt(i,j) = -(abs(aer(i)-cent(j)).^3) ./ GJ2;
    end
  end
  ppt(:,end) = 1;
  st = ppt * solt;
max(pp)
end
%***********************************************************************************************************************
function coeff = assembly_coeff(cent, toll)
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
  A21 = zeros(nstr, nstr);
  A22 = zeros(nstr, nstr);
  for k=1:nstr
    A11(k,:) = (abs(cent(k)-cent).^3) ./ EJ12;
    A21(k,:) = -(((cent(k)-cent)).*abs(cent(k)-cent)) ./ EJ4;
    A22(k,:) = -(abs((cent(k)-cent)))./ EJ2;
  end
  A =[ A11, A21; -A21, A22];
  invA = pinv(A, 1/toll);
%
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
  coeff = [rbf ; alfa];
%
end
%***********************************************************************************************************************
function coeff = assembly_coefft(cent, toll)
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
  for k=1:nstr
    A(k,:) = -(abs(cent(k)-cent).^3) ./ GJ2;
  end
  invA = pinv(A, 1/toll);
%
  Mp = pinv(P * invA * P', 1/toll);
  alfa = Mp * P * invA;
  rbf = invA - invA*(P') * Mp * P * invA;   
  coeff = [rbf ; alfa];
%

end