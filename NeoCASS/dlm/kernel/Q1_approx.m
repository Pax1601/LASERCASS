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

function [Q1] = Q1_approx(M, beta, s, r, nu, k, lattice, deltap, CORE_RADIUS)

MAXV = realmax;
MINV = -MAXV;

x = deltap(1);
y = deltap(2);
z = deltap(3);

r1 = sqrt( ( y - nu)^2 + z^2 );
R  = sqrt( ( x - nu*tan(lattice.lambda(s)))^2 + (beta * r1)^2 );

if abs(r1) < CORE_RADIUS
    % avoid singularity
    if x > 0
        u1 = MINV;
    else
        u1 = MAXV;
    end
else
    u1 = ( M * R - x + nu * tan(lattice.lambda(s)) ) / (beta^2 * r1);
end

% k1 is non-dimensional because k and r1 are non-dimensional
k1 = k .* r1;

if u1 >= 0
    if u1 == MAXV
        I1 = 0;
    else
        I1 = I1approx(u1, k1);
    end
else
    if u1 == MINV
        I1 = 2 .* real(I1approx(0, k1));
    else
        I1 = 2 .* real(I1approx(0, k1)) - real(I1approx(-u1, k1)) + i .* imag(I1approx(-u1, k1));
    end
end

% Planar part of kernel numerator:
if (u1 == MAXV || u1 == MINV)
    K1 = -I1;
else if (u1 < MAXV && u1 > MINV)
    K1 = -I1 - M * r1 / R .* exp(-i .* k1 .* u1) ./ sqrt(1 + u1^2);
    end
end
% Steady contribution
%K10 = -1-(x - nu * tan(lattice.lambda(s)))/R;
%K1 = K1 - K10;
%
T1 = cos(lattice.gamma(r) - lattice.gamma(s));
Q1 = ( K1 .* exp(-i .* k .* (x - nu * tan(lattice.lambda(s)))) ) .* T1;
%
end
