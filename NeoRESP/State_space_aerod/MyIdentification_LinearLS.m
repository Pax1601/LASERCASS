function [D,N] = MyIdentification_LinearLS(n,p,H,H0,weight,ro)
% =========================================================================
%                                                 MyIdentification_LinearLS
% =========================================================================
%
% Description:
% linear least squares identification of matrices Di, Ni,
% minimizing D*H = N, where
% D = D0 + p*D1 + ... p^n*eye(n),
% N = N0 + p*N1 + ... p^n*Nn + p^(n+1)*N_(n+1) + p^(n+ro)*N_(n+ro)
% and the constraint
% D0*H(0) = N0, at p = 0 is applied.
% Unknown matrix parameters are assembled in theta variable as:
% theta = [ D0^T,  D1^T,  ...  D_(n-1)^T,  N1^T,  N2^T,  ...  N_(n+ro)^T ]^T
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
if n < 1
    error('order n must be > 0')
end
nk = length(p);
dy = size(H,1);
du = size(H,2);
Iu = eye(du);
%
if isempty(ro), ro = 2; end
ordD = n;      % D0 + ... + Di*p^orderD
ordN = n+ro;   % N0 + ... + Ni*p^orderN

% -------------------------------------------------------------------------
% Assembling coefficient's matrix and rhs
% -------------------------------------------------------------------------
kinit = 2;

rhs = zeros( du*(nk+1-kinit), dy );
J   = zeros( du*(nk+1-kinit), dy*ordD + du*ordN );

for k = kinit:nk
    
    % Evaluate aerodynamic transfer matrix
    Hk = squeeze(H(:,:,k));
    
    % RHS
    b = ( (p(k)^ordD) * Hk  )';
    
    % Coefficient's matrix
    Jac = zeros( du , dy*ordD + du*ordN );
    Jac(:,1:dy) = ( H0 - Hk )';
    for m = 1 : (ordD-1)
        Jac(:, dy + (m-1)*dy + (1:dy) ) = -( (p(k)^m)*Hk )'; % derivatives w.r.t. Di
    end
    for m = 1 : ordN
        Jac(:, dy*n + (m-1)*du + (1:du) ) = ( (p(k)^m)*Iu )';  % derivatives w.r.t. Ni
    end
    
    % weighting
    if k == 2, W = weight^0.5; else W = 1; end
    
    % Assembling b and Jac for each p value
    rhs( (k-kinit)*du + (1:du), :) = W*b; 
    J( (k-kinit)*du + (1:du), :)   = W*Jac;
    
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
% theta = J\rhs;
theta = pinv(J)*rhs;

% elements below a threshold value are set to zero.
threshold = 10^-12;
theta(abs(theta) < threshold) = 0;

% -------------------------------------------------------------------------
% Post-processing
% -------------------------------------------------------------------------
% Extract Di and Ni matrices 
[D,N] = extractmatrices( 'left', 'lin', theta, H0, ordD, ordN );
