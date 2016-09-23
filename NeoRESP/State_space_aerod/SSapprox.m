function Happrox = SSapprox(p,A,B,C,E0,E1,E2)
% =========================================================================
%                                                                  SSapprox
% =========================================================================
%
% Description: evaluate the approximated aerodynamic transfer matrix at the 
%              given vector of reduced frequencies p, by knowing matrices 
%              A, B, C, E0, E1, and E2.
%
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

nk = length(p);
nx = size(A,1);
nu = size(B,2);
ny = size(C,1);
Ix = eye(nx);

Happrox = zeros(ny,nu,nk);
for k = 1:nk
    Happrox(:,:,k) = E0 + p(k)*E1 + p(k)^2*E2 + C*( ( p(k)*Ix - A )\ B );
end
