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

function [Q2] = Q2_approx(M, beta, s, r, nu, k, lattice, deltap, CORE_RADIUS)

MAXV = realmax;
MINV = -MAXV;
TOLL = 1.0e-1;

x = deltap(1);
y = deltap(2);
z = deltap(3);

r1 = sqrt( ( y - nu)^2 + z^2 );
R  = sqrt( ( x - nu*tan(lattice.lambda(s)))^2 + (beta * r1)^2 );
u1 = ( M * R - x + nu * tan(lattice.lambda(s)) ) / (beta^2 * r1);
% k1 is non-dimensional because k and r1 are non-dimensional
k1 = k .* r1;
X0 = (x - nu * tan(lattice.lambda(s)));
v1 = (M*r1^2 - X0*R) / (X0^2+r1^2);
%if (v1 >= 0)
% NEW UPDATE: use numerical integration for the whole domain v1->1-eps 
% tolerance on integration convergence raised
[I2,c1] = quadv(@(eta)FintI2(eta,k1),v1,1-eps,TOLL);
  % check accuracy
  %TOLL = 1.0e-3;
  %[I22,c2] = quadv(@(eta)FintI2(eta,k1),v1,1-eps,TOLL);
  %fprintf('\n%d %d %g %g %g %g',c2,c1,real(I2(2)),real(I22(2)),imag(I2(2)),imag(I22(2)));

%else
%  if (v1 <= -1)
%    I2 = quadv(@(eta)FintI2(eta,k1),v1,-1+eps,TOLL) +...
%             quadl(@(eta)FintI2(eta,k1),-1+eps,1-eps,TOLL);
%  else
%    I2 = quadv(@(eta)FintI2(eta,k1),v1,1-eps,TOLL);
%  end
%end
%
K2 = 3.*I2 + i.*k1.*(M * r1 / R)^2 .* exp(-i .* k1 .* u1) ./ sqrt(1+ u1^2) +...
                  M * r1 / R .* (( 1 + u1^2) .* (beta * r1 / R)^2 + 2 +...
                  M * r1 * u1 / R) .* exp(-i .* k1 .* u1) ./ (1 + u1^2)^1.5;
%
Dg = lattice.gamma(r) - lattice.gamma(s);
T2_star = z*(z * cos(Dg) - (y - nu) * sin(Dg));

% Steady contribution
%K20 = 2 + (x - nu * tan(lattice.lambda(s)))*(2 + beta^2*r1^2/R^2)/R;
%K2 = K2 - K20;
Q2 = ( K2.*exp(-i .* k .* (x - nu * tan(lattice.lambda(s)))) ) .* T2_star;

end
% Function for numerical integration
% see Landhal - Kernel Function for non planar oscillating surfaces in a subsonic flow 
function F = FintI2(eta, k)
  F = (1-eta.^2) .* exp( (-i.*k.*eta)./(sqrt(1-eta.^2)) );
end
