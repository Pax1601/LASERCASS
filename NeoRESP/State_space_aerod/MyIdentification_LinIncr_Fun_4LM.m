function [r,J] = MyIdentification_LinIncr_Fun_4LM(thetaold,ord,p,H,H0,weight,ro)
% =========================================================================
%                                          MyIdentification_LinIncr_Fun_4LM
% =========================================================================
%
% Description: linearized incremental method
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
dy = size(H,1);
du = size(H,2);
Iu = eye(du);
nk = length(p);

% Extract Di and Ni matrices
% -------------------------------------------------------------------------
if isempty(ro), ro = 2; end
orderD = ord;
orderN = ord + ro;
[D,N] = extractmatrices( 'left', 'lin', thetaold, H0, orderD, orderN );

% -------------------------------------------------------------------------
% Assembling coefficient's matrix and rhs
% -------------------------------------------------------------------------
kinit = 2;

% initialize matrices
r = zeros( du*(nk+1-kinit), dy );
J = zeros( du*(nk+1-kinit), dy*(ord) + du*(ord+ro) );

for k = kinit:nk
    
    % extract GAF at p = jk
    Hk = squeeze(H(:,:,k));
    
    % evaluate approximated aerodynamic transfer matrix
    Dk = zeros(dy,dy);
    Nk = zeros(dy,du);
    ii = 1;
    for m = 0:orderD,  Dk = Dk + ( p(k)^m ) * D{ii+m};  end
    for m = 0:orderN,  Nk = Nk + ( p(k)^m ) * N{ii+m};  end
    Happrox = Dk\Nk;
    
    % Jacobian
    Jac  = zeros( du, dy*orderD + du*orderN );
    Jac(:, 1:dy) =  H0' - Happrox';
    for m = 1 : (orderD-1)
        Jac(:, dy + (m-1)*dy + (1:dy) ) = - ( (p(k)^m)*Happrox )'; % derivatives w.r.t. Di
    end
    for m = 1 : orderN
        Jac(:, dy*ord + (m-1)*du + (1:du) ) =  ( p(k)^m*Iu )';  % derivatives w.r.t. Ni
    end    

    % RHS
    b = ( Dk*Hk - Nk )'; 
    
    % Weighting
    if k == 2, W = weight^0.5; else W = 1; end
    
    % Assembling b and Jac for each p value
    r( (k-kinit)*du + (1:du), :) = W*b;
    J( (k-kinit)*du + (1:du), :) = W*Jac;
      
end

% stack of real and imaginary part
rRe = real(r); JRe = real(J);
rIm = imag(r); JIm = imag(J);
clear r J
J = [JRe; JIm];
r = [rRe; rIm];

 