
function Happrox = MFDapprox(ord,p,D,N,type,ro)
% =========================================================================
%                                                                 MFDapprox
% =========================================================================
%
% Description: evaluate the approximated aerodynamic transfer matrix 
%              by using the MFD matrices D, N
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

if isempty(ro),
    ro = 2;
end
%
orderD = ord;
orderN = ord + ro;
%
nk = length(p);
nu = size(N{1},2);
ny = size(N{1},1);

% initialize matrix
Happrox = zeros(ny,nu,nk);

for k = 1:nk
    
    clear Dk Nk
    switch type
        case{'lmfd','LMFD','left'},   Dk = zeros(ny,ny);
        case{'rmfd','RMFD','right'},  Dk = zeros(nu,nu);
    end
    Nk = zeros(ny,nu);
    
    ii = 1;
    for m = 0:orderD,    Dk = Dk + (( p(k)^m ) * D{ii+m});  end
    for m = 0:orderN,  Nk = Nk + (( p(k)^m ) * N{ii+m});  end
    
    switch type
        case{'lmfd','LMFD','left'},  Happrox(:,:,k) = Dk\Nk;
        case{'rmfd','RMFD','right'}, Happrox(:,:,k) = Nk/Dk;
    end
    
end