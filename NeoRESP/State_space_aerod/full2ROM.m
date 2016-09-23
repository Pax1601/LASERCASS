function [Ared,Bred,Cred,Dred,ordred] = full2ROM(A,B,C,D,alg,order)
% =========================================================================
%                                                                  full2ROM
% =========================================================================
%
% Description: Hankel-based model order reduction. 
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
%   Changes:
%   - saturation of determinant weighting;
%   - enforcement of MFD stability choosable between minimal eigensolution
%     changes or poles placement;
%   - added initial touch-up of the MFD output matrix;
%   - balanced reduction chosen to be the best between truncation and
%     low frequency residualisation of the balanced reduction (norm
%     bound given up in favour of a better fit, whenever possible)
%   - state equation residual extended to third order;
%   - low frequency equality constraint simplified through
%     weighting in the final touch;
%   
% =========================================================================

SSsys = ss(A,B,C,D);

% alg = 'schur', 'hankel' or 'balance'
try
  disp('REDUCE function will be used.')
  [SSredsys,redinfo] = reduce(SSsys,order,'algorithm',alg);
catch
  disp('Not available. Robust control toolbox may not be present.');
  disp('BALRED function will be used');
  %
  if isempty(order)
    hankelsv(SSsys);
    order = input('Please enter the desired order: (>=0) ');
    if isempty(order) || isequal(order,0)
      order = size(A,1);
    end
  end
  [SSredsys] = balred(SSsys,order,'Elimination','Truncate');
end
%
[Ared,Bred,Cred,Dred] = ssdata(SSredsys);
ordred = size(Ared,1);
