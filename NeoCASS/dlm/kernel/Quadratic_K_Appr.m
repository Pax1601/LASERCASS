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

function [D1rs, D2rs,F] = Quadratic_K_Appr(M, beta, s, r, k, lattice, dlm_model, deltap)


nr = length(r);
nk = length(k);
D1rs = zeros(nr,nk);
D2rs = D1rs;
F = D1rs;

% Extract from dlm
x = deltap(:,1);
y = deltap(:,2);
z = deltap(:,3);
dx = lattice.dx(s);

% Sending box semiwidth
e  = lattice.e(s);
e2 = e^2;
e3 = e^3;
e4 = e^4;

% shorter notations
y2 = y.^2;
z2 = z.^2;
cek = y2 + z2 - e2;
cekF = 2*e*abs(z)./(cek);
cek2 = cek.^2;
CORE_RADIUS = e * dlm_model.param.CORE_RADIUS;

IND = find(abs(z)< CORE_RADIUS );
A1 = zeros(nr,nk);
B1 = A1; C1 = A1; 
A2 = A1;
B2 = A2; C2 = A2; 

if ~isempty(IND)    % PLANAR CONFIGURATION
    
    %----------------------------------------------------------------------
    % Quadratic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q10   = Q1_approxT(M, beta, s, r(IND), 0, k, lattice, deltap(IND,:), CORE_RADIUS);      % center
    Q1e   = Q1_approxT(M, beta, s, r(IND), e, k, lattice, deltap(IND,:),  CORE_RADIUS);      % outboard
    % Quadratic approx. coeff.
    A1(IND,:) = (Q1_e - 2.*Q10 + Q1e)./(2*e2);
    B1(IND,:) = (Q1e - Q1_e)./(2*e);
    C1(IND,:) =  Q10;
    % F in the planar case
    F(IND,:) =  meshgrid(2*e./(y2(IND) -e2),k)';
    % PLANAR DOWNWASH FACTOR
    D1rs(IND,:) = dx / (8*pi) .*((meshgrid( y2(IND) -z2(IND),k )' .* A1(IND,:) + meshgrid(y(IND),k)' .* B1(IND,:) + C1(IND,:)).*F(IND,:) + (.5 .* B1(IND,:) + meshgrid(y(IND),k)' .* A1(IND,:))...
                .* meshgrid(log(( (y(IND) - e).^2 + z2(IND)) ./ ( (y(IND)+e).^2 + z2(IND) ) ),k)' + 2 * e .* A1(IND,:));
    % NON PLANAR DOWNWASH FACTOR
    D2rs(IND,:) = 0;
    
end    
if  length(IND)<nr           % NON PLANAR CONFIGURATION
    
    IND = find(abs(z)>= CORE_RADIUS );
    %----------------------------------------------------------------------
    % Quadratic approx. for PLANAR DOWNWASH FACTOR:
    Q1_e  = Q1_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q10   = Q1_approxT(M, beta, s, r(IND),  0, k, lattice, deltap(IND,:), CORE_RADIUS);      % center
    Q1e   = Q1_approxT(M, beta, s, r(IND),  e, k, lattice, deltap(IND,:), CORE_RADIUS);      % outboard
    % Quadratic approx. coeff.
    A1(IND,:) = (Q1_e - 2 * Q10 + Q1e) ./ ( 2*e2 );
    B1(IND,:) = (Q1e - Q1_e) ./ ( 2*e );
    C1(IND,:) =  Q10;
	
    %----------------------------------------------------------------------
    % Quadratic approx. for NON-PLANAR DOWNWASH FACTOR:
    Q2_e  = Q2_approxT(M, beta, s, r(IND), -e, k, lattice, deltap(IND,:), CORE_RADIUS);      % inboard
    Q20   = Q2_approxT(M, beta, s, r(IND),  0, k, lattice, deltap(IND,:), CORE_RADIUS);       % center
    Q2e   = Q2_approxT(M, beta, s, r(IND),  e, k, lattice, deltap(IND,:), CORE_RADIUS);       % outboard
    
	% Quadratic approx. coeff.
    A2(IND,:) = (Q2_e - 2*Q20 + Q2e)./(2*e2);
    B2(IND,:) = (Q2e - Q2_e)./(2*e);
    C2(IND,:) = Q20;
    LIND = length(IND);
    d1 = zeros(LIND,1); % cek = 0 condition
    d2 = 0.5*ones(LIND,1);
  %
%     d1 = 0.0; % cek = 0 condition
%     d2 = 0.5;
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
    alpha = zeros(size(IND));
    
    if ~(isempty(IND2))    %abs( 2*e*z/cek ) > .3
        IND3 = IND(IND2);
        ATAN = atan(2*e*abs(z(IND3))./cek(IND3));
        %       if ATAN <0
        %         ATAN = ATAN + pi;
        %       end
        ATAN(ATAN <0) = ATAN(ATAN <0)+pi;
        
        alpha(IND3) = e2./z2(IND3) .*(1 - cek(IND3)./(2*e*abs(z(IND3))) .*ATAN);
    end
    if length(IND2)< length(IND)
        IND3 = IND(abs( 2*e*z(IND)./cek(IND) ) <= .3); 
        % series
        n = 2:7;
        alpha(IND3) = 4*e4./cek2(IND3) .* sum(meshgrid((-1).^n ./(2*n-1),IND3) .*meshgrid((2*e*z(IND3)./cek(IND3)),n)'.^meshgrid((2*n-4),IND3),2);
    end
    % F in the non-planar case
    F(IND,:) = meshgrid(d1.*2*e./cek(IND) .*(1 - alpha(IND).*z2(IND)/e2) + d2.*pi./abs(z(IND)),k)';
%      d1*2*e/cek *(1 - alpha*z2/e2) + d2*pi/abs(z);
    % PLANAR DOWNWASH FACTOR
    D1rs(IND,:) = dx / (8*pi) .*((meshgrid( y2(IND) -z2(IND),k )' .* A1(IND,:) + meshgrid(y(IND),k)' .* B1(IND,:) + C1(IND,:)).*F(IND,:) + (.5 .* B1(IND,:) + meshgrid(y(IND),k)' .* A1(IND,:))...
                .* meshgrid(log(( (y(IND) - e).^2 + z2(IND)) ./ ( (y(IND)+e).^2 + z2(IND) ) ),k)' + 2 * e .* A1(IND,:));
    
    
    % NON PLANAR DOWNWASH FACTOR
    
    IND2 = find(abs( 1./cekF(IND))<.1);
    
    if ~(isempty(IND2)) % abs(1/cekF) < 0.1
        
        IND3 = IND(IND2); 
        D2rs(IND3,:) = dx./(16*pi*meshgrid(z2(IND3),k)') .*( (meshgrid((y2(IND3)+z2(IND3)),k)'.*A2+meshgrid(y(IND3),k)'.*B2(IND3,:)+C2(IND3,:)).*F(IND3,:)+...
                                ( meshgrid(((y2(IND3)+z2(IND3)).*y(IND3)+(y2(IND3)-z2(IND3))*e),k)'.*A2(IND3,:)+meshgrid((y2(IND3)+z2(IND3)+y(IND3)*e),k)'.*B2(IND3,:)+meshgrid((y(IND3)+e),k)'.*C2(IND3,:))/(meshgrid((y(IND3)+e).^2+z2(IND3),k)')-...
                                (meshgrid(((y2(IND3)+z2(IND3)).*y(IND3)-(y2(IND3)-z2(IND3))*e),k)'.*A2(IND3,:)+meshgrid((y2(IND3)+z2(IND3)-y(IND3)*e),k)'.*B2(IND3,:)+meshgrid((y(IND3)-e),k)'.*C2(IND3,:))/meshgrid(((y(IND3)-e).^2+z2(IND3)),k)' );
                            
%                             dx/(16*pi*z2) .*( ((y2+z2).*A2+y.*B2(IND3,:)+C2(IND3,:)).*F(IND3,:)+...
%                                 (((y2+z2)*y+(y2-z2)*e).*A2(IND3,:)+(y2+z2+y*e).*B2(IND3,:)+(y+e).*C2(IND3,:))/((y+e)^2+z2)-...
%                                 (((y2+z2)*y-(y2-z2)*e).*A2(IND3,:)+(y2+z2-y*e).*B2(IND3,:)+(y-e).*C2(IND3,:))/((y-e)^2+z2) );
    end
    if length(IND2)< length(IND)
        
        IND2 = find(abs( 1./cekF(IND))>= .1);
        IND3 = IND(IND2);
        Delta = (e./abs(z(IND3))).^2.*(1-d1(IND2)-d2(IND2).*pi./abs(z(IND3)).*(cek(IND3)/(2*e)));
        D2rs(IND3,:) = meshgrid(e*dx./(8*pi*cek(IND3)),k)' .*( (2*meshgrid(y2(IND3)+z2(IND3)+e2,k)'.*(e2.*A2(IND3,:)+C2(IND3,:))+meshgrid(4*y(IND3)*e2,k)'.*B2(IND3,:))./...
                                   meshgrid(((y(IND3)+e).^2+z2(IND3)).*((y(IND3)-e).^2+z2(IND3)),k)' -...
                                   meshgrid((d1(IND2).*alpha(IND2)+Delta)/e2,k)'.*(meshgrid(y2(IND3)+z2(IND3),k)'.*A2(IND3,:)+meshgrid(y(IND3),k)'.*B2(IND3,:)+C2(IND3,:)) ); 
    end

end
