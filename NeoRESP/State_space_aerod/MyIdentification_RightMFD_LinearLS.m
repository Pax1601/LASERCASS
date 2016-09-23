function [Di,Ni] = MyIdentification_RightMFD_LinearLS(n,p,H,H0,weight,ro)
% =========================================================================
%                                        MyIdentification_RightMFD_LinearLS
% =========================================================================
%
% Description:
% linear least squares identification of matrices Di, Ni,
% minimizing H = N/D, where
% D = D0 + p*D1 + ... p^n*eye(n),
% N = N0 + p*N1 + ... p^n*Nn + p^(n+1)*N_(n+1) + p^(n+2)*N_(n+2)
% and the constraint
% D0*H(0) = N0, at p = 0 is applied.
% Unknown matrix parameters are assembled in theta variable as:
% theta = [ D0^T,  D1^T,  ...  D_(n-1)^T,  N1^T,  N2^T,  ...  N_(n+2)^T ]^T
% -------------------------------------------------------------------------
%
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
%
% =========================================================================

% -------------------------------------------------------------------------
% Pre-processing 
% -------------------------------------------------------------------------

% auxiliary variables
if n < 1
    error('order n must be > 0')
end
if isempty(ro), ro = 2; end
orderD = n;      % D0 + ... + Di*p^orderD
orderN = n+ro;   % N0 + ... + Ni*p^orderN

dy = size(H,1);
du = size(H,2);
Iy = eye(dy);  
nk = length(p);

% -------------------------------------------------------------------------
% Assembling coefficient's matrix and rhs
% -------------------------------------------------------------------------
kinit = 2;

% matrices initialization
rhs = zeros( dy*(nk+1-kinit), du );
J   = zeros( dy*(nk+1-kinit), du*orderD + dy*orderN );

for k = kinit:nk  
    
    % Evaluate GAF matrix at p=jk
    Hk = squeeze(H(:,:,k));
    
    % RHS
    b =  (p(k)^orderD) * Hk;
    
    % Coefficient's matrix
    Jac = zeros( dy, du*orderD + dy*orderN );
    Jac(:,1:du) = H0 - Hk;
    for m = 1:orderD-1
        Jac(:, du + (m-1)*du + (1:du) ) = - (p(k)^m)*Hk;  % derivatives w.r.t. Di
    end
    
    for m = 1:orderN
        Jac(:, du*orderD + (m-1)*dy + (1:dy) ) =  (p(k)^m)*Iy;  % derivatives w.r.t. Ni
    end
       
    % Weighting
    if k == 2, W = weight^0.5; else W = 1; end 
     
    rhs( (k-kinit)*dy + (1:dy), : ) = W*b;
    J(   (k-kinit)*dy + (1:dy), : ) = W*Jac;
    
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

% elements below a threshold value are setted to zero.
threshold = 10^-12;
theta(abs(theta) < threshold) = 0;

% -------------------------------------------------------------------------
% Post-processing
% -------------------------------------------------------------------------

% Extract Di,Ni matrices from theta solution
[Di,Ni] = extractmatrices( 'right', 'lin', theta, H0, orderD, orderN );
