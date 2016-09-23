%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080308      1.0     L.Cavagna        Creation
%
%*******************************************************************************
% Results in linear interpolation on MASTER_C coordinates of a function
% X 
function Y = pp_line_param_cstr(SLAVE_C, MASTER_C, DESVAR, DES_INDEX)

nm = size(MASTER_C, 3);
ns = size(SLAVE_C, 3);

X = DESVAR.X(DES_INDEX) .* DESVAR.Xnorm(DES_INDEX);
%
if (nm > 1)
  MASTER_C = reshape(MASTER_C,[3, nm, 1])';
else
  nm = size(MASTER_C,1);
end
%
if (length(DES_INDEX) ~= nm)
  error('Design variables index is different from master coordinates points.');
end
%
if (ns > 1)
  SLAVE_C = reshape(SLAVE_C,[3, ns, 1])';
else
  ns = size(SLAVE_C,1);
end
%
%
if (nm < 2)
  error('At least two points must be required for linear interpolation.');
end
%
MCGAXIS = zeros(nm,1);
SCGAXIS = zeros(ns,1);
%
ORIGIN = MASTER_C(1,:); END = MASTER_C(end,:);
axis = END - ORIGIN; axis = axis ./ norm(axis);
%
for k=1:nm
  MCGAXIS(k) = dot(axis, MASTER_C(k,:) - ORIGIN);
end
%
for k=1:ns
  SCGAXIS(k) = dot(axis, SLAVE_C(k,:) - ORIGIN);
end
% check for correct allignement
if (~isempty(find(MCGAXIS(2:nm-1) > MCGAXIS(end))))
  error('Unable to determine correctly interpolation axis. Change last element declared');
end
% sort elements
[MCGAXIS(2:end-1), index] = sort(MCGAXIS(2:end-1));
% sort relative PARAM
XMM = X(2:end-1); X(2:end-1) = XMM(index);
%
if (~isempty(find(SCGAXIS > MCGAXIS(end))))
  error('Unable to determine correctly interpolation axis. Change last element declared');
end
if (~isempty(find(SCGAXIS < 0)))
  error('Unable to determine correctly interpolation axis. Change first element declared');
end
%
% Linear piece-wise interpolation
%
PP = interp1(MCGAXIS, X,'linear','pp');
Y = ppval(PP, SCGAXIS);
%plot(SCGAXIS, Y, 'xr', MCGAXIS, X,'-ko');

end
