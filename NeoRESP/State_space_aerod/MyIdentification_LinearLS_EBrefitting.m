function [Happrox,B0,B1,B2,B3,E0,E1,E2] = MyIdentification_LinearLS_EBrefitting(p,H,H0,A,C,weight,ro)
% =========================================================================
%                                     MyIdentification_LinearLS_EBrefitting
% =========================================================================
%
% Description: fitting of (E,B) matrices by using a linear least-squares 
%              approximation imposing the constraint at k = 0
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
% -------------------------------------------------------------------------
dy = size(H,1);
du = size(H,2);
dx = size(A,1);
Iy = eye(dy);  
Ix = eye(dx); 
nk = length(p);
W = 1;
if ro==1
  nB = 2;  % number of unknown B matrices
  nE = 1;  % number of unknown E matrices             
else
  nB = 4;  % number of unknown B matrices
  nE = 1;  % number of unknown E matrices             
end
kinit = 2;

% matrices initialization 
% -------------------------------------------------------------------------
rhs = zeros( dy*(nk+1-kinit), du );
J   = zeros( dy*(nk+1-kinit), dx*nB + dy*nE);

% matrices assembling 
% -------------------------------------------------------------------------
if ro==1
  for k = kinit:nk

      % evaluation of GAF matrix
      Fk = squeeze(H(:,:,k));

      % residual and jacobian
      b   = Fk - real(H0);
      Jac = [ ...
          C*( ( p(k)*Ix - A )\Ix + (A\Ix) ) ...                          % differentiation w.r.t B0
          C*( ( p(k)*Ix - A )\( [ p(k)*Ix ] ) )... % differentiation w.r.t B1, B2
          [ p(k)*Iy]                                         % differentiation w.r.t E1
          ];

      % weighting
      if k == 2, W = weight^0.5; else W = 1; end

      % assembling
      rhs( (k-kinit)*dy + (1:dy), :) = W*b;
      J( (k-kinit)*dy + (1:dy), :)   = W*Jac; 

  end
else
  for k = kinit:nk

      % evaluation of GAF matrix
      Fk = squeeze(H(:,:,k));

      % residual and jacobian
      b   = Fk - real(H0) - p(k)*imag(H0)/abs(p(1));
      Jac = [ ...
          C*( ( p(k)*Ix - A )\Ix + (A\Ix) -p(k)*(A\(A\Ix))) ...                          % differentiation w.r.t B0
          C*( ( p(k)*Ix - A )\(  p(k)*Ix  ) +p(k) * (A\Ix))... % differentiation w.r.t B1
          C*( ( p(k)*Ix - A )\( [p(k)^2*Ix  p(k)^3*Ix ] ) )...% differentiation w.r.t B2, B3
          [ p(k)^2*Iy ]                                         % differentiation w.r.t E1, E2
          ];

      % weighting
      if k == 2, W = weight^0.5; else W = 1; end

      % assembling
      rhs( (k-kinit)*dy + (1:dy), :) = W*b;
      J( (k-kinit)*dy + (1:dy), :)   = W*Jac; 

  end
end
% stack of real and imaginary part
rRe = real(rhs); JRe = real(J);
rIm = imag(rhs); JIm = imag(J);
clear rhs J
J   = [JRe; JIm];
rhs = [rRe; rIm];

% LS solution
% -------------------------------------------------------------------------
theta = pinv(J)*rhs;
   
% exctract matrices from LS solution
% -------------------------------------------------------------------------
ny = 0; nx = 0;
if ro == 1
  B0 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  B1 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  B2 = zeros(dx,du);
  B3 = zeros(dx,du);
  E1 = theta(ny*dy+nx*dx+(1:dy),:);
  E2 = zeros(dy,du);
else
  B0 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  B1 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  B2 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  B3 = theta(ny*dy+nx*dx+(1:dx),:); nx=nx+1;
  E2 = theta(ny*dy+nx*dx+(1:dy),:); ny=ny+1;
%  E2 = theta(ny*dy+nx*dx+(1:dy),:);
end
%
E0 = real(H0) + C*(A\B0);
E1 = imag(H0)/abs(p(1)) + C*(A\B1) - C*(A\(A\B0)); %from H0 constraint

% Evaluate approximation
% -------------------------------------------------------------------------
Happrox = zeros(dy,du,nk);
for k = 1:length(p)
    Bk = B0 + p(k)*B1 + p(k)^2*B2 + p(k)^3*B3;
    Ek = E0 + p(k)*E1 + p(k)^2*E2;
    Happrox(:,:,k) = C*( ( p(k)*Ix - A )\Bk ) + Ek; 
end

