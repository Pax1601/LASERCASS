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
%	
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function I0 = exp_I0_coeff(k1, u1)

%if u1 < 0
%    fprintf(' \nu1 cannot be less than zero...u1 = %.2f;\n',u1)
%    return
%end

a =   [ 0.24186198;
        -2.7918027; 
        24.991079; 
        -111.59196; 
        271.43549; 
        -305.75288;           
        -41.183630; 
        545.98537; 
        -644.78155; 
        328.72755; 
        -64.279511];
c = .372;
%I0 = sum(a.*(n*c - i*k1).*exp(-n*c*u1) ./(n.^2*c^2 + k1^2));

I0 =  a(1) .* (c - i.*k1)    .* exp(-c*u1)    ./(c^2 + k1.^2)      + ...
      a(2) .* (2*c - i.*k1)  .* exp(-2*c*u1)  ./(2^2*c^2 + k1.^2)  + ... 
	  a(3) .* (3*c - i.*k1)  .* exp(-3*c*u1)  ./(3^2*c^2 + k1.^2)  + ...
	  a(4) .* (4*c - i.*k1)  .* exp(-4*c*u1)  ./(4^2*c^2 + k1.^2)  + ...
	  a(5) .* (5*c - i.*k1)  .* exp(-5*c*u1)  ./(5^2*c^2 + k1.^2)  + ...
	  a(6) .* (6*c - i.*k1)  .* exp(-6*c*u1)  ./(6^2*c^2 + k1.^2)  + ...
	  a(7) .* (7*c - i.*k1)  .* exp(-7*c*u1)  ./(7^2*c^2 + k1.^2)  + ...
	  a(8) .* (8*c - i.*k1)  .* exp(-8*c*u1)  ./(8^2*c^2 + k1.^2)  + ...
	  a(9) .* (9*c - i.*k1)  .* exp(-9*c*u1)  ./(9^2*c^2 + k1.^2)  + ...
	 a(10) .* (10*c - i.*k1) .* exp(-10*c*u1) ./(10^2*c^2 + k1.^2) + ...
	 a(11) .* (11*c - i.*k1) .* exp(-11*c*u1) ./(11^2*c^2 + k1.^2); 
