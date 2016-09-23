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
%**************************************************************************
%  FFAST Project
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     011010      1.5     L.Travaglini     Creation
%
%**************************************************************************

function Acc = get_acceleration(T)


nt = length(T);
dt2 = (T(2)-T(1))^2;

row = zeros(1,3*nt+4);
col = row;
val = row;

% point 1

row(1:4) = [1,1,1,1];
col(1:4) = [1,2,3,4];
val(1:4) = [2,-5,4,-1]./(dt2);

% point 2

row(5:8) = [2,2,2,2];
col(5:8) = [2,3,4,5];
val(5:8) = [2,-5,4,-1]./(dt2);

% point n

row(9:end-8) = reshape(meshgrid(3:nt-2,[1,2,3]),1,3*(nt-4));
col(9:end-8) = reshape((meshgrid([1,3,5],3:nt-2) + (meshgrid(0:nt-5,[1,2,3]))')',1,3*(nt-4));
val(9:end-8) = reshape(meshgrid([1,-2,1],3:nt-2)',1,3*(nt-4))./(4*dt2);

% point end-1

row(end-7:end-4) = [nt-1,nt-1,nt-1,nt-1];
col(end-7:end-4) = [nt-3,nt-2,nt-1,nt];
val(end-7:end-4) = flipud([2,-5,4,-1]')'./(dt2);

% point end

row(end-3:end) = [nt,nt,nt,nt];
col(end-3:end) = [nt-3,nt-2,nt-1,nt];
val(end-3:end) = flipud([2,-5,4,-1]')'./(dt2);

Acc = sparse(row,col,val,nt,nt);
 