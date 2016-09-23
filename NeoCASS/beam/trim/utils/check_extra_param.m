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
%***********************************************************************************************************************
%  SimSAC Project
%
%  SMARTCAD
%  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Alessandro Degaspari <degaspari@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%***********************************************************************************************************************
%	
%   Author: Luca Cavagna, Pierangelo Masarati, DIAPM
%***********************************************************************************************************************

function [EXTRA] = check_extra_param(nv, PARAM, VALUE)

np = 4;

EXTRA = [];

EXTRA.Value = zeros(1, np);
EXTRA.Fixed = zeros(1, np);

EXTRA.Value(4) = 0.8; % set default value

for i = 1:nv

	switch cell2mat(PARAM(i))
	
		case 'VGUST' % GUST VELOCITY
		
		EXTRA.Value(1) = VALUE(i);
		EXTRA.Fixed(1) = 1;

		case 'VSINK' % LANDING SINK VELOCITY
		
		EXTRA.Value(2) = abs(VALUE(i));
		EXTRA.Fixed(2) = 1;

		case 'STROKE' % LANDING MAX STROKE
		
		EXTRA.Value(3) = abs(VALUE(i));
		EXTRA.Fixed(3) = 1;

    case 'LNDGEFF' % landing struct efficiency

		EXTRA.Value(4) = abs(VALUE(i));
		EXTRA.Fixed(4) = 1;

	end

end
