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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************


function [M, G] = spar_mass(X, data)
%
T  = X(1);
B1 = X(2);
T1 = X(3);
B2 = X(4);
T2 = X(5);
BU = X(6);
TU = X(7);
SP = X(8);
%
%------------------------------------------------------------------
% CAP
Acap = B1 * T1 + B2 * T2;
Y = (B1*T1^2+T2*B2*(2*T1+B2))/2/Acap;
%------------------------------------------------------------------
% dimensions
H = data.geo.h;        % section height
He = H - 2*Y;          % effective depth
Hu = He - (B2+T1-Y);   % stiffener length 
Hc = Hu - (B2+T1-Y)/2; % web cleared depth (not used here)
%------------------------------------------------------------------
% OBJ: total mass for spar section
%
% SAME HEIGHTS
% STIFFENER
% smeared stiffener weight
Mst_sme = (2*BU*TU)*H/SP;
% OBJ: total mass for spar section
M = T * H + 2 * Acap + Mst_sme;
G = [H; 2*T1; 2*B1; 2*T2; 2*B2; 2*TU*H/SP; 2*BU*H/SP; -2*BU*TU*H/SP^2];
%------------------------------------------------------------------
% OBJ: total mass for spar section
%
% DIFFERENT HEIGHTS
% STIFFENER
% smeared stiffener weight
%Mst_sme = (2*BU*TU)*Hu/SP;
%------------------------------------------------------------------
% OBJ: total mass for spar section
%M = T * He + 2 * Acap + Mst_sme;
% gradient of OBJ
%G = [H - (2*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)
%    2*T1 - T*(T1^2/(B1*T1 + B2*T2) - (2*T1*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2) - (2*BU*TU*(T1^2/(2*(B1*T1 + B2*T2)) - (T1*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2))/SP
%    2*B1 + T*((2*B1*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2 - 2) + (2*BU*TU*((B1*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2 - 2))/SP
%    2*T2 - T*((2*((T2*(B2 + 2*T1))/2 + (B2*T2)/2))/(B1*T1 + B2*T2) - (2*T2*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2) - (2*BU*TU*(((T2*(B2 + 2*T1))/2 + (B2*T2)/2)/(B1*T1 + B2*T2) - (T2*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2 + 1))/SP
%    2*B2 + T*((2*B2*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2 - (B2*(B2 + 2*T1))/(B1*T1 + B2*T2)) + (2*BU*TU*((B2*((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2))/(B1*T1 + B2*T2)^2 - (B2*(B2 + 2*T1))/(2*(B1*T1 + B2*T2))))/SP
%    -(2*TU*(B2 - H + T1 + ((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2)/(B1*T1 + B2*T2)))/SP
%    -(2*BU*(B2 - H + T1 + ((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2)/(B1*T1 + B2*T2)))/SP
%    (2*BU*TU*(B2 - H + T1 + ((B1*T1^2)/2 + (B2*T2*(B2 + 2*T1))/2)/(B1*T1 + B2*T2)))/SP^2];
