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
% Modified by Lorenzo Travaglini
% The new version work on matrices
%**************************************************************************
%*********************************************
function [D1rs, D2rs,F] = Quartic_K_ApprT(M, beta, s, r, k, lattice, dlm_model, deltap)

% M      scalar
% beta   scalar
% s      scalar
% r      vector 
% k      scalar or vector
% deltap vector
nr = length(r);
nk = length(k);
D1rs = zeros(nr,nk);
D2rs = D1rs;
F = D1rs;

x = deltap(:,1);
y = deltap(:,2);
z = deltap(:,3);
dx = lattice.dx(s);
% Sending box semiwidth
e  = lattice.e(s);
e2 = e^2;
e3 = e^3;
e4 = e^4;

% shorter notation
y2 = y.^2; 
y4 = y.^4; 
y6 = y.^6; 
z2 = z.^2; 
z4 = z.^4; 
z6 = z.^6;
cek = y2 + z2 - e2; 
cekF = 2*e*abs(z)./(cek);
cek2 = cek.^2;
CORE_RADIUS = e * dlm_model.param.CORE_RADIUS;
IND = find(abs(z)< CORE_RADIUS );
A1 = zeros(nr,nk); 
B1 = A1; C1 = A1; D1 = A1; E1 = A1;
A2 = A1;
B2 = A2; C2 = A2; D2 = A2; E2 = A2;

if ~isempty(IND)  % PLANAR CONFIGURATION
    
    %----------------------------------------------------------------------
    % Quartic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q1_e2 = Q1_approxT(M, beta, s, r(IND), -e/2, k, lattice, deltap(IND,:), CORE_RADIUS);    % inboard intermediate
    Q10   = Q1_approxT(M, beta, s, r(IND), 0, k, lattice, deltap(IND,:), CORE_RADIUS);       % center
    Q1e2  = Q1_approxT(M, beta, s, r(IND), e/2, k, lattice, deltap(IND,:), CORE_RADIUS);     % outboard intermediate
    Q1e   = Q1_approxT(M, beta, s, r(IND), e, k, lattice, deltap(IND,:), CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A1(IND,:)  = -1/(6*e2) *(Q1_e  - 16*Q1_e2  + 30*Q10  - 16*Q1e2  + Q1e );
    B1(IND,:)  =  1/(6*e)  *(Q1_e  - 8*Q1_e2  + 8*Q1e2  - Q1e );
    C1(IND,:)  =  Q10 ;
    D1(IND,:)  = -2/(3*e3) *(Q1_e  - 2*Q1_e2  + 2*Q1e2  - Q1e );
    E1(IND,:)  =  2/(3*e4) *(Q1_e  - 4*Q1_e2  + 6*Q10  - 4*Q1e2  + Q1e );
    
    % F in the planar case
    F(IND,:)  = meshgrid(2*e./(y2(IND) -e2),k)';
    
    % PLANAR DOWNWASH FACTOR
    D1rs(IND,:)  = dx/(8*pi).*(( meshgrid( y2(IND) -z2(IND),k )'.*A1(IND,:) +meshgrid(y(IND),k)' .*B1(IND,:) +C1(IND,:) +meshgrid(y(IND) .*(y2(IND) -3*z2(IND)),k)'...
                   .*D1(IND,:) +meshgrid((y4(IND) -6*y2(IND).*z2(IND) +z4(IND) ),k)'.*E1(IND,:) ).*F(IND,:)  +...
                  (meshgrid(y(IND),k)' .*A1(IND,:)+.5.*B1(IND,:)+.5.*meshgrid(3*y2(IND)-z2(IND),k)'.*D1(IND,:)+2.*meshgrid(y(IND).*(y2(IND)-z2(IND)),k)'.*E1(IND,:))...
                  .*meshgrid(log(((y(IND)-e).^2+z2(IND))./((y(IND)+e).^2+z2(IND))),k)' +...
                  (A1(IND,:)+meshgrid(2*y(IND),k)'.*D1(IND,:)+meshgrid(3*y2(IND)-z2(IND)+1/3*e2,k)'.*E1(IND,:)).*2*e);
    % NON PLANAR DOWNWASH FACTOR
    D2rs(IND,:) = zeros(length(IND),nk); 
end
    
if length(IND)<nr                % NON PLANAR CONFIGURATION
    
    IND = find(abs(z)>= CORE_RADIUS );
    %----------------------------------------------------------------------
    % Quartic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q1_e2 = Q1_approxT(M, beta, s, r(IND), -e/2, k, lattice, deltap(IND,:), CORE_RADIUS);    % inboard intermediate
    Q10   = Q1_approxT(M, beta, s, r(IND), 0, k, lattice, deltap(IND,:), CORE_RADIUS);       % center
    Q1e2  = Q1_approxT(M, beta, s, r(IND), e/2, k, lattice, deltap(IND,:), CORE_RADIUS);     % outboard intermediate
    Q1e  = Q1_approxT(M, beta, s, r(IND), e, k, lattice, deltap(IND,:), CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A1(IND,:) = -1/(6*e2) *(Q1_e - 16*Q1_e2 + 30*Q10 - 16*Q1e2 + Q1e);
    B1(IND,:) =  1/(6*e)  *(Q1_e - 8*Q1_e2 + 8*Q1e2 - Q1e);
    C1(IND,:) =  Q10;
    D1(IND,:) = -2/(3*e3) *(Q1_e - 2*Q1_e2 + 2*Q1e2 - Q1e);
    E1(IND,:) =  2/(3*e4) *(Q1_e - 4*Q1_e2 + 6*Q10 - 4*Q1e2 + Q1e);
    
    %----------------------------------------------------------------------
    % Quartic approx. for NON-PLANAR DOWNWASH FACTOR:
%     Q2_e = zeros(length(IND),length(k));
%     Q2_e2 = Q2_e;
%     Q20 = Q2_e;
%     Q2e2 = Q2_e;
%     Q2e = Q2_e;
%     for i = 1 : length(IND)
%         Q2_e(i,:)  = Q2_approx(M, beta, s, r(IND(i)), -e, k, lattice, deltap(IND(i),:), CORE_RADIUS);      % inboard
%         Q2_e2(i,:) = Q2_approx(M, beta, s, r(IND(i)), -e/2, k, lattice, deltap(IND(i),:), CORE_RADIUS);    % inboard intermediate
%         Q20(i,:)   = Q2_approx(M, beta, s, r(IND(i)), 0, k, lattice, deltap(IND(i),:), CORE_RADIUS);       % center
%         Q2e2(i,:)  = Q2_approx(M, beta, s, r(IND(i)), e/2, k, lattice, deltap(IND(i),:), CORE_RADIUS);     % outboard intermediate
%         Q2e(i,:)   = Q2_approx(M, beta, s, r(IND(i)), e, k, lattice, deltap(IND(i),:), CORE_RADIUS);       % outboard
%     end
    Q2_e  = Q2_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q2_e2 = Q2_approxT(M, beta, s, r(IND), -e/2, k, lattice, deltap(IND,:), CORE_RADIUS);    % inboard intermediate
    Q20   = Q2_approxT(M, beta, s, r(IND), 0, k, lattice, deltap(IND,:), CORE_RADIUS);       % center
    Q2e2  = Q2_approxT(M, beta, s, r(IND), e/2, k, lattice, deltap(IND,:), CORE_RADIUS);     % outboard intermediate
    Q2e   = Q2_approxT(M, beta, s, r(IND), e, k, lattice, deltap(IND,:), CORE_RADIUS);       % outboard
    % Quartic approx. coeff.
    A2(IND,:) = -1/(6*e2) *(Q2_e - 16*Q2_e2 + 30*Q20 - 16*Q2e2 + Q2e);
    B2(IND,:) =  1/(6*e)  *(Q2_e - 8*Q2_e2 + 8*Q2e2 - Q2e);
    C2(IND,:) =  Q20;
    D2(IND,:) = -2/(3*e3) *(Q2_e - 2*Q2_e2 + 2*Q2e2 - Q2e);
    E2(IND,:) =  2/(3*e4) *(Q2_e - 4*Q2_e2 + 6*Q20 - 4*Q2e2 + Q2e);
    % Place arcotangent in the correct quadrant
    LIND = length(IND);
    d1 = zeros(LIND,1); % cek = 0 condition
    d2 = 0.5*ones(LIND,1);
%     if (cek > 0)
%       d1 = 1;
%       d2 = 0;
%     elseif (cek <0)
%       d1 = 1;
%       d2 = 1;
%     end
%
    d1(cek(IND)>0) = ones(length(cek(cek(IND)>0)),1);
    d2(cek(IND)>0) = zeros(length(cek(cek(IND)>0)),1);
    
    d1(cek(IND)<0) = ones(length(cek(cek(IND)<0)),1);
    d2(cek(IND)<0) = ones(length(cek(cek(IND)<0)),1);
    
    IND2 = find(abs( 2*e*z(IND)./cek(IND) ) > .3);
    epss = zeros(size(IND));
    if ~(isempty(IND2))
        
      IND3 = IND(IND2);  
      ATAN = atan(2*e*abs(z(IND3))./cek(IND3));
      ATAN(ATAN <0) = ATAN(ATAN <0)+pi;
      epss(IND3) = e2./z2(IND3).*(1 - cek(IND3)./(2*e*abs(z(IND3))) .*ATAN);
    end
    if length(IND2)< length(IND)
%         IND2 = find(abs( 2*e*z(IND)./cek(IND) ) <= .3);
        IND3 = IND(abs( 2*e*z(IND)./cek(IND) ) <= .3); 
        % series
        n = 2:7;
        epss(IND3) = 4*e4./cek2(IND3) .*sum(meshgrid((-1).^n ./(2*n-1),IND3) .*meshgrid(2*e*z(IND3)./cek(IND3),n)'.^meshgrid(2*n-4,IND3),2);
    end
    % F in the non-planar case
    F(IND,:) = meshgrid(d1.*2*e./cek(IND) .*(1 - epss(IND).*z2(IND)/e2) + d2.*pi./abs(z(IND)),k)';
    % PLANAR DOWNWASH FACTOR
	D1rs(IND,:) = dx/(8*pi)*((meshgrid(y2(IND)-z2(IND),k)'.*A1(IND,:)+meshgrid(y(IND),k)'.*B1(IND,:)+C1(IND,:)+...
                  meshgrid(y(IND).*(y2(IND)-3*z2(IND)),k)'.*D1(IND,:)+meshgrid(y4(IND)-6*y2(IND).*z2(IND)+z4(IND),k)'.*E1(IND,:)).*F(IND,:) +...
                    (meshgrid(y(IND),k)'.*A1(IND,:)+.5.*B1(IND,:)+.5*meshgrid(3*y2(IND)-z2(IND),k)'.*D1(IND,:)+...
                     meshgrid(2*y(IND).*(y2(IND)-z2(IND)),k)'.*E1(IND,:)).*meshgrid(log(((y(IND)-e).^2+z2(IND))./((y(IND)+e).^2+z2(IND))),k)' +...
                    (A1(IND,:)+meshgrid(2*y(IND),k)'.*D1(IND,:)+meshgrid((3*y2(IND)-z2(IND)+1/3*e2),k)'.*E1(IND,:)).*2*e);
    % NON PLANAR DOWNWASH FACTOR
    IND2 = find(abs( cek(IND)./(2*e*z(IND)))>.1);
    if ~(isempty(IND2))
      IND3 = IND(IND2);
        epss(IND2) = e2./z2(IND3) .*(1 - cek(IND3)./(2*e*abs(z(IND3))).*atan(2*e*abs(z(IND3))./cek(IND3)));
        Delta = (e./abs(z(IND3))).^2 .* (1-d1(IND2)-d2(IND2)*pi./abs(z(IND3)).*(cek(IND3)/(2*e)));
        D2rs(IND3,:) = meshgrid(e*dx./(8*pi*cek(IND3)),k)'.*(1./meshgrid(((y(IND3)+e).^2+z2(IND3)).*((y(IND3)-e).^2+z2(IND3)),k)'.*...
                 (meshgrid(2*(y2(IND3)+z2(IND3)+e2),k)'.*(e2.*A2(IND3,:)+C2(IND3,:))+meshgrid(4*y(IND3)*e2,k)'.*...
                 B2(IND3,:)+meshgrid(2*y(IND3).*(y4(IND3)-2*e2*y2(IND3)+2*y2(IND3).*z2(IND3)+3*e4+2*e2*z2(IND3)...
                 +z4(IND3)),k)'.*D2(IND3,:) +...
                  meshgrid(2*(3*y6(IND3)-7*e2.*y4(IND3)+5*y4(IND3).*z2(IND3)+6*e4*y2(IND3)+6*e2*y2(IND3).*z2(IND3)-...
                  3*e2*z4(IND3)-z6(IND3)+y2(IND3).*z4(IND3)-2*e4*z2(IND3)),k)'.*E2(IND3,:)) -...
                 meshgrid((d1(IND2).*epss(IND2)+Delta),k)'./e2.*(meshgrid(y2(IND3)+z2(IND3),k)'...
                 .*A2(IND3,:)+meshgrid(y(IND3),k)'.*B2(IND3,:)+C2(IND3,:)+...
                 meshgrid(y(IND3).*(y2(IND3)+3*z2(IND3)),k)'.*D2(IND3,:)+meshgrid((y4(IND3)+6*y2(IND3).*z2(IND3)...
                 -3*z4(IND3)),k)'.*E2(IND3,:))) +...
                 dx/(8*pi) .* (D2(IND3,:)./2.*meshgrid(log(((y(IND3)-e).^2+z2(IND3))./((y(IND3)+e).^2+z2(IND3))),k)'...
                 +2*meshgrid(e+y(IND3).*log(((y(IND3)-e).^2+z2(IND3))./((y(IND3)+e).^2+z2(IND3))),k)'.*E2(IND3,:));
    end
    if length(IND2)< length(IND)
        IND3 = IND(abs( cek(IND)./(2*e*z(IND)) ) <= .1);
            D2rs(IND3,:) = dx./(16*pi*meshgrid(z2(IND3),k)').*((meshgrid(y2(IND3)+z2(IND3),k)'.*A2(IND3,:)+meshgrid(y(IND3),k)'...
                            .*B2(IND3,:)+C2(IND3,:)+meshgrid(y(IND3).*(y2(IND3)+3*z2(IND3)),k)'...
                           .*D2(IND3,:)+meshgrid(y4(IND3)+6*y2(IND3).*z2(IND3)-3*z4(IND3),k)'.*E2(IND3,:)).*F(IND3,:) +...
                     meshgrid(1./((y(IND3)+e).^2+z2(IND3)),k)'.*(meshgrid((y2(IND3)+z2(IND3)).*y(IND3)+(y2(IND3)-z2(IND3))*e,k)'.*...
                     A2(IND3,:)+meshgrid(y2(IND3)+z2(IND3)+y(IND3)*e,k)'.*B2(IND3,:)+meshgrid(y(IND3)+e,k)'.*C2(IND3,:) +...
                     meshgrid(y4(IND3)-z4(IND3)+(y2(IND3)-3*z2(IND3)).*y(IND3)*e,k)'.*D2(IND3,:)+meshgrid((y4(IND3)-2*y2(IND3)...
                     .*z2(IND3)-3*z4(IND3)).*y(IND3)+(y4(IND3)-6*y2(IND3).*z2(IND3)+z4(IND3))*e,k)'.*E2(IND3,:)) -...
                     meshgrid(1./((y(IND3)-e).^2+z2(IND3)),k)'.*(meshgrid((y2(IND3)+z2(IND3)).*y(IND3)-...
                     (y2(IND3)-z2(IND3))*e,k)'.*A2(IND3,:)+meshgrid(y2(IND3)+z2(IND3)-y(IND3)*e,k)'.*B2(IND3,:)+meshgrid(y(IND3)...
                     -e,k)'.*C2(IND3,:) +...
                     meshgrid(y4(IND3)-z4(IND3)-(y2(IND3)-3*z2(IND3)).*y(IND3)*e,k)'.*D2(IND3,:)+meshgrid(( y4(IND3)-...
                     2.*y2(IND3).*z2(IND3)-3*z4(IND3)).*y(IND3)-(y4(IND3)-6*y2(IND3).*z2(IND3)+z4(IND3))*e,k)'.*E2(IND3,:)) +...
                     meshgrid(z2(IND3).*log(((y(IND3)-e).^2+z2(IND3))./((y(IND3)+e).^2+z2(IND3))),k)'.*D2(IND3,:) +...
                     meshgrid(4*z2(IND3).*(e+y(IND3).*log(((y(IND3)-e).^2+z2(IND3))./((y(IND3)+e).^2+z2(IND3)))),k)'.*E2(IND3,:));
    end
end

