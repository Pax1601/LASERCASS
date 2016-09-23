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
% Modified by Travaglini, vectoral operation.
%***********************************************************************************************************************

function [Q2] = Q2_approxT(M, beta, s, r, nu, k, lattice, deltap, CORE_RADIUS)

MAXV = realmax;
MINV = -MAXV;
TOLL = 1.0e-1;

x = deltap(:,1);
y = deltap(:,2);
z = deltap(:,3);

r1 = sqrt( ( y - nu).^2 + z.^2 );
R  = sqrt( ( x - nu*tan(lattice.lambda(s))).^2 + (beta * r1).^2 );
u1 = ( M * R - x + nu * tan(lattice.lambda(s)) )./ (beta^2 * r1);
% k1 is non-dimensional because k and r1 are non-dimensional
k1 = r1*k;
X0 = (x - nu * tan(lattice.lambda(s)));
v1 = (M*r1.^2 - X0.*R) ./ (X0.^2+r1.^2);
%if (v1 >= 0)
% NEW UPDATE: use numerical integration for the whole domain v1->1-eps 
% tolerance on integration convergence raised
% I2 = zeros(size(k1));

% for i = 1 : length(v1)
nv = length(v1);
nk = length(k);
 
% ntrap = 400;
% 
% X =linspace(1,0,ntrap)'* (v1-((1-eps)))' +(1-eps);
% X = reshape(meshgrid(X,k)',ntrap,nv,nk);
% K1 = reshape(meshgrid(k1,1:ntrap),ntrap,nv,nk);
% F = (1-X.^2) .* exp( (-1i*X.*K1)./(sqrt(1-X.^2)) );
% I2 = sum((F(2:end,:,:)+F(1:end-1,:,:)).*(X(2:end,:,:)-X(1:end-1,:,:))*0.5,1);
% I2 = reshape(I2,nv,nk);
%     [I2,c1] = quadv(@(eta)FintI2(eta,k1),v1,ones(length(v1),1)*(1-eps),TOLL);
% end
  % check accuracy
  %TOLL = 1.0e-3;
  [I2,c1] = quadvVECT(@(eta)FintI2(eta,k1),v1,ones(size(v1))*(1-eps),TOLL);
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
K2 = 3.*I2 + 1i.*k1.*meshgrid((M .* r1 ./ R).^2,k)' .* exp(-1i.* k1 .* meshgrid(u1,k)') ./ meshgrid(sqrt(1+ u1.^2),k)' +... 
                  meshgrid(M * r1 ./ R, k)' .* meshgrid(( 1 + u1.^2) .* (beta * r1 ./ R).^2 + 2 +...
                  M * r1 .* u1 ./ R, k)' .* exp(-1i * k1 .* meshgrid(u1,k)') ./ meshgrid( (1 + u1.^2).^1.5 ,k)';
%
Dg = lattice.gamma(r) - lattice.gamma(s);
T2_star = z.*(z .* cos(Dg) - (y - nu) .* sin(Dg));

% Steady contribution
%K20 = 2 + (x - nu * tan(lattice.lambda(s)))*(2 + beta^2*r1^2/R^2)/R;
%K2 = K2 - K20;
Q2 = ( K2.*exp(-1i*(x - nu * tan(lattice.lambda(s))) * k ) ) .* meshgrid(T2_star,k)';

end
% Function for numerical integration
% see Landhal - Kernel Function for non planar oscillating surfaces in a subsonic flow 
function F = FintI2(eta, k)
  F = meshgrid(1-eta.^2,k(1,:))' .* exp( (-1i*meshgrid(eta,k(1,:))'.*k)./meshgrid(sqrt(1-eta.^2),k(1,:))' ); 
%   F = (1-eta.^2) .* exp( (-1i*eta*k)./(sqrt(1-eta.^2)) );
end
