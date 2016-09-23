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

function Vel = get_velocity(T)


nt = length(T);
dt = T(2)-T(1);

row = zeros(1,3*nt);
col = row;
val = row;

row(1:3) = [1,1,1];
col(1:3) = [1,2,3];
val(1:3) = [-3/2,2,-0.5]./dt;
% val(1:4) = [-13/3,9.5,-7,11/6]./dt;

row(4:end-3) = reshape(meshgrid(2:nt-1,[1,2,3]),1,3*(nt-2));
col(4:end-3) = reshape((meshgrid([1,2,3],2:nt-1) + (meshgrid(0:nt-3,[1,2,3]))')',1,3*(nt-2));
val(4:end-3) = reshape(meshgrid([-1,0,1],2:nt-1)',1,3*(nt-2))./(2*dt);

row(end-2:end) = [nt,nt,nt];
col(end-2:end) = [nt-2,nt-1,nt];
val(end-2:end) = [0.5,-2,3/2]./dt;

Vel = sparse(row,col,val,nt,nt);
