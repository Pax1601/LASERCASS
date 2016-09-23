function [r,J] = MyIdentification_RightMFD_Adjugate_Fun_4LM(thetaold,ord,p,H,H0,weight,ro)
% =========================================================================
%                                MyIdentification_RightMFD_Adjugate_Fun_4LM
% =========================================================================
%
% Description: assembling of the residual and jacobian matrix 
%              to be used with the adjugate-based method 
%              for RMFD identification
% 
% -------------------------------------------------------------------------
%
%   Copyright (C) 2012 Paolo Mantegazza   <mantegazza@aero.polimi.it>
%   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
%  
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License as
%   published by the Free Software Foundation; either version 3 of the
%   License, or (at your option) any later version.
%  
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%  
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
%  
%   Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of
%        aerodynamic transfer matrices', AIAA journal, submitted for publication.
%
% =========================================================================

% -------------------------------------------------------------------------
% Pre-processing
% -------------------------------------------------------------------------

% auxiliary variables
if isempty(ro), ro = 2; end
ordD = ord;         % D0 + ... + Di*p^orderD
ordN = ord+ro;      % N0 + ... + Ni*p^orderN

dy = size(H,1);
du = size(H,2);
Iy = eye(dy);  
nk = length(p);

% Extract Di and Ni matrices
% -------------------------------------------------------------------------
[D,N] = extractmatrices( 'right', 'adj', thetaold, H0, ordD, ordN );

% -------------------------------------------------------------------------
% Assembling coefficient's matrix and rhs
% -------------------------------------------------------------------------
kinit = 2;

% initialize matrices
r = zeros( dy*(nk+1-kinit), du );
J = zeros( dy*(nk+1-kinit), du*ordD + dy*ordN );

for k = kinit:nk
    
    % extract GAF at p = jk
    Hk = squeeze(H(:,:,k));
    
    % evaluate approximated aerodynamic transfer matrix
    Nk = zeros(dy,du);
    Dk = zeros(du,du);
    ii = 1;
    for m = 0:ordD,  Dk = Dk + (( p(k)^m ) * D{ii+m});  end
    for m = 0:ordN,  Nk = Nk + (( p(k)^m ) * N{ii+m});  end
    Happrox = Nk/Dk;
    
    % determinant 
    detD = det(Dk);
    detDinv = 1/detD;
    Mthreshold = 10^9;
    if abs(detDinv) > Mthreshold, detDinv = Mthreshold; end
    
    % Jacobian
    Jac = zeros( dy, du*ordD + dy*ordN );
    Jac(:, 1:du) =  H0 - Happrox;
    for m = 1 : (ordD-1)
        Jac(:, du + (m-1)*du + (1:du) ) = - (p(k)^m)*Happrox; % derivatives w.r.t. Di
    end
    for m = 1 : ordN
        Jac(:, du*ordD + (m-1)*dy + (1:dy) ) = p(k)^m*Iy;  % derivatives w.r.t. Ni
    end
    Jac = Jac*detDinv;
    
    % RHS
    b = ( Hk*Dk - Nk )*detDinv;
    
    % Weighting
    if k == 2, W = weight^0.5; else W = 1; end
    
    % Assembling b and Jac for each p value
    r( (k-kinit)*dy + (1:dy), :) = W*b;
    J( (k-kinit)*dy + (1:dy), :) = W*Jac;

end

% stack of real and imaginary part
rRe = real(r); JRe = real(J);
rIm = imag(r); JIm = imag(J);
clear r J
J = [JRe; JIm];
r = [rRe; rIm];
