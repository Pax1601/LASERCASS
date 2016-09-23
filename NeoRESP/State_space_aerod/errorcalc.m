function varargout = errorcalc(Ha,Happrox, flagpr)
% =========================================================================
%                                                                 errorcalc
% =========================================================================
%
% Description:
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

% number of reduced frequencies
nk = size(Ha,3);

r = zeros(nk,1);
for k = 1:nk
    r(k) = norm( squeeze(Ha(:,:,k) - Happrox(:,:,k)), 'fro' ) / norm(squeeze(Ha(:,:,k)), 'fro' );
end
err = norm(r);

if flagpr
    fprintf(1,'error: %12.6e\n', err);
end

% assign output if required
if nargout == 1
    varargout{1} = err;
end