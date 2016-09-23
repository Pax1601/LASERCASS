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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%   Author: <andreadr@kth.se>
%
% Extract main mechanical properties for single-closed cell tail booms. 
% 
% Called by:    Prop_Sec.m
% 
% Calls:
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080723      1.0     A. Da Ronch      Creation
%     091119      1.3.9   Travaglini       Modification
%
% Cleaned by Travaglini
%*******************************************************************************
function str = Prop_Sec_Tbooms(geo, str)

%--------------------------------------------------------------------------------------------------
% Initialize structure
str.tbooms.m  = [];    % Mass distribution along fuselage reference line       [kg], vector
str.tbooms.I1 = [];    % Second moment of inertia w.r.t. Y axis                [m4], vector
str.tbooms.I2 = [];    % Second moment of inertia w.r.t. Z axis                [m4], vector
str.tbooms.J  = [];    % Torsional constant                                    [m4], vector
str.tbooms.K1 = [];    % Area factor for shear                                 [??], vector
str.tbooms.K2 = [];    % Area factor for shear                                 [??], vector

%--------------------------------------------------------------------------------------------------
% Structural mass from regression analysis - the remaining mass up to total
% mass is considered as NSM, thus included in the correspondent function
str.tbooms.m  = str.tbooms.WI;
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Second moment of inertia, I1 = Ixx, I2 = Iyy, using structural sizing thicknesses
str.tbooms.I1 = pi.*str.tbooms.tbar_B.*geo.tbooms.R^3;
str.tbooms.I2 = str.tbooms.I1;
%--------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------
% Torsional constant for single closed cell
% str.tbooms.J  = zeros(geo.tbooms.lenx+1, 1);

str.tbooms.J  = 4 .*(pi.*geo.tbooms.R.^2).^2 ./ (geo.tbooms.P./str.tbooms.tbar_B);
str.tbooms.K1 = zeros(size(str.tbooms.J));
str.tbooms.K2 = str.tbooms.K1;
%--------------------------------------------------------------------------------------------------
% 
% % Export half-model
% if isequal(stick.model.symmXZ, 0)
%     str.tbooms.m  = str.fus.m ./2;
%     str.tbooms.I1 = str.fus.I1 ./2;
%     str.tbooms.I2 = str.fus.I2 ./2;
%     str.tbooms.J  = str.fus.J ./2;
%     str.tbooms.K1 = str.fus.K1 ./2;
%     str.tbooms.K2 = str.fus.K2 ./2;
% end
