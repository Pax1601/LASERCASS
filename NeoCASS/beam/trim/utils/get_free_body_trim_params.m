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

function FM = get_free_body_trim_params(nv, PARAM, VALUE)

np = 15;

FM = [];

FM.Value = zeros(1, np);
FM.Fixed = logical(zeros(1, np));

for i = 1:nv

	switch cell2mat(PARAM(i))
	
		case 'THRUST' % thrust percentage
		
		FM.Value(1) = VALUE(i);
		FM.Fixed(1) = true;
		
		case 'ANGLEA'
		
		FM.Value(2) = VALUE(i);
		FM.Fixed(2) = true;
	
		case 'SIDES'

		FM.Value(3) = VALUE(i);
		FM.Fixed(3) = true;
		
		case 'ROLL'
		
		FM.Value(4) = VALUE(i);
		FM.Fixed(4) = true;

	
		case 'PITCH'
		
		FM.Value(5) = VALUE(i);
		FM.Fixed(5) = true;
		
		case 'YAW'
		
		FM.Value(6) = VALUE(i);
		FM.Fixed(6) = true;
		
		case 'URDD1'
		
		FM.Value(7) = VALUE(i);
		FM.Fixed(7) = true;

		case 'URDD2'
		
		FM.Value(8) = VALUE(i);
		FM.Fixed(8) = true;
		
		case 'URDD3'

		FM.Value(9) = VALUE(i);
		FM.Fixed(9) = true;

		case 'URDD4'

		FM.Value(10) = VALUE(i);
		FM.Fixed(10) = true;
		
		case 'URDD5'
		
		FM.Value(11) = VALUE(i);
		FM.Fixed(11) = true;

		case 'URDD6'
		
		FM.Value(12) = VALUE(i);
		FM.Fixed(12) = true;
		
		case 'HEAD'

		FM.Value(13) = D2R(VALUE(i));
		FM.Fixed(13) = true;

		case 'CLIMB'

		FM.Value(14) = D2R(VALUE(i));
		FM.Fixed(14) = true;

		case 'BANK'

		FM.Value(15) = D2R(VALUE(i));
		FM.Fixed(15) = true;

	end

end
