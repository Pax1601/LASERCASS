function [Happrox,C,E0,E1,E2] = MyIdentification_LinearLS_ECrefitting(p,H,H0,A,B,weight,ro)
% =========================================================================
%                                     MyIdentification_LinearLS_ECrefitting
% =========================================================================
%
% Description: Linear Least-Squares with contraints at p=0.
%              unknown: E,C matrix
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

% auxiliary variables
dy = size(H,1);
du = size(H,2);
dx = size(A,1);
Ix = eye(dx);
Iu = eye(du);
nk = length(p);
W = 1;
% matrices initialization 
kinit = 2;
rhs = zeros( du*(nk+1-kinit), dy );
if ro==1
  J   = zeros( du*(nk+1-kinit), dx + du );
  for k = kinit : nk

      % evaluation of GAF matrix
      Hk = squeeze(H(:,:,k));

      % residual and Jacobian
      b = Hk - real(H0);
      b = b';
      Jac   = [ ( (A\B{1}) + ( p(k)*Ix - A )\( B{1} + p(k)*B{2} + p(k)^2*B{3} + p(k)^3*B{4} ) )' ...  % derivatives w.r.t. C
          (p(k)*Iu)'  ] ; % derivatives w.r.t. E1 
      % weighting
      if k == 2, W = weight^0.5; else W = 1; end

      % assembling matrices
      rhs( (k-kinit)*du + (1:du), :) = W*b;
      J( (k-kinit)*du + (1:du), :)   = W*Jac; 

  end
else
  J   = zeros( du*(nk+1-kinit), dx + du );
  for k = kinit : nk

      % evaluation of GAF matrix
      Hk = squeeze(H(:,:,k));

      % residual and Jacobian
      b = Hk - real(H0) - p(k) * imag(H0)/abs(p(1));
      b = b';
      Jac   = [ ( (A\B{1}) + p(k)*(A\B{2}) -p(k)*(A\(A\B{1})) + ( p(k)*Ix - A )\( B{1} + p(k)*B{2} + p(k)^2*B{3} + p(k)^3*B{4} ) )' ...  % derivatives w.r.t. C
          (p(k)^2*Iu)' ] ; % derivatives w.r.t. E1 and E2

      % weighting
      if k == 2, W = weight^0.5; else W = 1; end

      % assembling matrices
      rhs( (k-kinit)*du + (1:du), :) = W*b;
      J( (k-kinit)*du + (1:du), :)   = W*Jac; 

  end
end
% stack of real and imaginary part
rRe = real(rhs); JRe = real(J);
rIm = imag(rhs); JIm = imag(J);
clear rhs J
J   = [JRe; JIm];
rhs = [rRe; rIm];


% -------------------------------------------------------------------------
% LS solution
% -------------------------------------------------------------------------
theta = pinv(J)*rhs;

% extract data from solution
C  = (theta( 1:dx, : ))';
%E1 = (theta( dx + (1:du), : ))';
if ro == 1
  E2 = zeros(dy,du);
else
  E2 = (theta( dx + (1:du), : ))';
end
E0 = real(H0) + C*(A\B{1}); %from H0 constraint
E1 = imag(H0)/abs(p(1)) + C*(A\B{2}) - C*(A\(A\B{1})); %from H0 constraint

ii = 1;
Happrox = zeros(dy,du,nk);
for k = 1:nk
    Bk = B{ii+0} + p(k)*B{ii+1} + p(k)^2*B{ii+2} + p(k)^3*B{ii+3};
    Ek = E0      + p(k)*E1      + p(k)^2*E2;
    Happrox(:,:,k) = C*(( p(k)*Ix - A )\Bk) + Ek;
end

