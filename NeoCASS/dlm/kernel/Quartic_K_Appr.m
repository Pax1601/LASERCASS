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

function [D1rs, D2rs,F] = Quartic_K_Appr(M, beta, s, r, k, lattice, dlm_model, deltap)

x = deltap(1);
y = deltap(2);
z = deltap(3);
dx = lattice.dx(s);
% Sending box semiwidth
e  = lattice.e(s);
e2 = e^2;
e3 = e^3;
e4 = e^4;

% shorter notation
y2 = y^2; 
y4 = y^4; 
y6 = y^6; 
z2 = z^2; 
z4 = z^4; 
z6 = z^6;
cek = y2 + z2 - e2; 
cekF = 2*e*abs(z)/(cek);
cek2 = cek^2;
CORE_RADIUS = e * dlm_model.param.CORE_RADIUS;

if abs(z) < CORE_RADIUS ; % PLANAR CONFIGURATION
    
    %----------------------------------------------------------------------
    % Quartic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approx(M, beta, s, r, -e, k, lattice, deltap, CORE_RADIUS);      % inboard
    Q1_e2 = Q1_approx(M, beta, s, r, -e/2, k, lattice, deltap, CORE_RADIUS);    % inboard intermediate
    Q10   = Q1_approx(M, beta, s, r, 0, k, lattice, deltap, CORE_RADIUS);       % center
    Q1e2  = Q1_approx(M, beta, s, r, e/2, k, lattice, deltap, CORE_RADIUS);     % outboard intermediate
    Q1e   = Q1_approx(M, beta, s, r, e, k, lattice, deltap, CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A1 = -1/(6*e2) *(Q1_e - 16*Q1_e2 + 30*Q10 - 16*Q1e2 + Q1e);
    B1 =  1/(6*e)  *(Q1_e - 8*Q1_e2 + 8*Q1e2 - Q1e);
    C1 =  Q10;
    D1 = -2/(3*e3) *(Q1_e - 2*Q1_e2 + 2*Q1e2 - Q1e);
    E1 =  2/(3*e4) *(Q1_e - 4*Q1_e2 + 6*Q10 - 4*Q1e2 + Q1e);
    
    % F in the planar case
    F = 2*e/(y2-e2);
    
    % PLANAR DOWNWASH FACTOR
    D1rs = dx/(8*pi).*(((y2-z2).*A1+y.*B1+C1+y*(y2-3*z2).*D1+(y4-6*y2*z2+z4).*E1).*F +...
                  (y.*A1+.5.*B1+.5*(3*y2-z2).*D1+2*y*(y2-z2).*E1)*log(((y-e)^2+z2)/((y+e)^2+z2)) +...
                  (A1+2*y.*D1+(3*y2-z2+1/3*e2).*E1).*2*e);
    % NON PLANAR DOWNWASH FACTOR
    D2rs = 0;
    
else                % NON PLANAR CONFIGURATION
    
    %----------------------------------------------------------------------
    % Quartic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approx(M, beta, s, r, -e, k, lattice, deltap, CORE_RADIUS);      % inboard
    Q1_e2 = Q1_approx(M, beta, s, r, -e/2, k, lattice, deltap, CORE_RADIUS);    % inboard intermediate
    Q10   = Q1_approx(M, beta, s, r, 0, k, lattice, deltap, CORE_RADIUS);       % center
    Q1e2  = Q1_approx(M, beta, s, r, e/2, k, lattice, deltap, CORE_RADIUS);     % outboard intermediate
    Q1e   = Q1_approx(M, beta, s, r, e, k, lattice, deltap, CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A1 = -1/(6*e2) *(Q1_e - 16*Q1_e2 + 30*Q10 - 16*Q1e2 + Q1e);
    B1 =  1/(6*e)  *(Q1_e - 8*Q1_e2 + 8*Q1e2 - Q1e);
    C1 =  Q10;
    D1 = -2/(3*e3) *(Q1_e - 2*Q1_e2 + 2*Q1e2 - Q1e);
    E1 =  2/(3*e4) *(Q1_e - 4*Q1_e2 + 6*Q10 - 4*Q1e2 + Q1e);
    
    %----------------------------------------------------------------------
    % Quartic approx. for NON-PLANAR DOWNWASH FACTOR:
    Q2_e  = Q2_approx(M, beta, s, r, -e, k, lattice, deltap, CORE_RADIUS);      % inboard
    Q2_e2 = Q2_approx(M, beta, s, r, -e/2, k, lattice, deltap, CORE_RADIUS);    % inboard intermediate
    Q20   = Q2_approx(M, beta, s, r, 0, k, lattice, deltap, CORE_RADIUS);       % center
    Q2e2  = Q2_approx(M, beta, s, r, e/2, k, lattice, deltap, CORE_RADIUS);     % outboard intermediate
    Q2e   = Q2_approx(M, beta, s, r, e, k, lattice, deltap, CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A2 = -1/(6*e2) *(Q2_e - 16*Q2_e2 + 30*Q20 - 16*Q2e2 + Q2e);
    B2 =  1/(6*e)  *(Q2_e - 8*Q2_e2 + 8*Q2e2 - Q2e);
    C2 =  Q20;
    D2 = -2/(3*e3) *(Q2_e - 2*Q2_e2 + 2*Q2e2 - Q2e);
    E2 =  2/(3*e4) *(Q2_e - 4*Q2_e2 + 6*Q20 - 4*Q2e2 + Q2e);
    % Place arcotangent in the correct quadrant
    d1 = 0.0; % cek = 0 condition
    d2 = 0.5;
    if (cek > 0)
      d1 = 1;
      d2 = 0;
    elseif (cek <0)
      d1 = 1;
      d2 = 1;
    end
%
    if abs( 2*e*z/cek ) > .3
      ATAN = atan(2*e*abs(z)/cek);
      if ATAN <0
        ATAN = ATAN + pi;
      end
        epss = e2/z2 *(1 - cek/(2*e*abs(z)) *ATAN);
	  else
        % series
        n = [2:7];
        epss = 4*e4/cek2 *sum((-1).^n ./(2*n-1) .*(2*e*z/cek) .^(2*n-4));
    end
    % F in the non-planar case
    F = d1*2*e/cek *(1 - epss*z2/e2) + d2*pi/abs(z);
    % PLANAR DOWNWASH FACTOR
	  D1rs = dx/(8*pi)*(((y2-z2).*A1+y.*B1+C1+y*(y2-3*z2).*D1+(y4-6*y2*z2+z4).*E1).*F +...
                    (y.*A1+.5.*B1+.5*(3*y2-z2).*D1+2*y*(y2-z2).*E1)*log(((y-e)^2+z2)/((y+e)^2+z2)) +...
                    (A1+2*y.*D1+(3*y2-z2+1/3*e2).*E1).*2*e);
    % NON PLANAR DOWNWASH FACTOR
    if abs(cek/(2*e*z)) > 0.1
        epss = e2/z2 *(1 - cek/(2*e*abs(z)) *atan(2*e*abs(z)/cek));
        Delta = (e/abs(z))^2 *(1-d1-d2*pi/abs(z)*(cek/(2*e)));
        D2rs = e*dx/(8*pi*cek) *(1/(((y+e)^2+z2)*((y-e)^2+z2))*...
                 (2*(y2+z2+e2)*(e2.*A2+C2)+4*y*e2.*B2+2*y*(y4-2*e2*y2+2*y2*z2+3*e4+2*e2*z2+z4).*D2 +...
                  2*(3*y6-7*e2*y4+5*y4*z2+6*e4*y2+6*e2*y2*z2-3*e2*z4-z6+y2*z4-2*e4*z2).*E2) -...
                 (d1*epss+Delta)/e2*((y2+z2).*A2+y.*B2+C2+y*(y2+3*z2).*D2+(y4+6*y2*z2-3*z4).*E2)) +...
               dx/(8*pi) .* (D2./2.*log(((y-e)^2+z2)/((y+e)^2+z2))+2*(e+y*log(((y-e)^2+z2)/((y+e)^2+z2))).*E2);
    else
            D2rs = dx/(16*pi*z2) *(((y2+z2).*A2+y.*B2+C2+y*(y2+3*z2).*D2+(y4+6*y2*z2-3*z4).*E2).*F +...
                     1/((y+e)^2+z2)*(((y2+z2)*y+(y2-z2)*e).*A2+(y2+z2+y*e).*B2+(y+e).*C2 +...
                     (y4-z4+(y2-3*z2)*y*e).*D2+((y4-2*y2*z2-3*z4)*y+(y4-6*y2*z2+z4)*e).*E2) -...
                     1/((y-e)^2+z2)*(((y2+z2)*y-(y2-z2)*e).*A2+(y2+z2-y*e).*B2+(y-e).*C2 +...
                     (y4-z4-(y2-3*z2)*y*e).*D2+((y4-2*y2*z2-3*z4)*y-(y4-6*y2*z2+z4)*e).*E2) +...
                     (z2*log(((y-e)^2+z2)/((y+e)^2+z2))).*D2 +...
                     4*z2*(e+y*log(((y-e)^2+z2)/((y+e)^2+z2))).*E2);
    end
end
