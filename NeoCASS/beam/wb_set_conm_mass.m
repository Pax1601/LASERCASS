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
% Calculates rigid body mass matrix due to concentrated masses
% This function is called as first for WEIGHT AND BALANCE module
% It can also be called when the aircraft is deformed to determine new inertial properties.
%
function [NEWCG, NEWCGM, NEWMRP] = wb_set_conm_mass(ntot, NODEINDEX, COORD, ROT, GRDPNT, CONM)

NEWMRP = zeros(6,6);
NEWCGM = zeros(6,6);
NEWCG  = zeros(1,3);
CALC = false;

if ntot > 0

	S = zeros(1,3);
	J = zeros(3,3);
	
	mtot = 0;
	for n=1:ntot

		%if NODEINDEX(CONM.Node(n)) % check if node is structural 
			S = S + CONM.M(1,1,n) .* (COORD(CONM.Node(n),1:3) + (ROT(:,:,CONM.Node(n)) * CONM.Offset(n,:)')'); % sum static moments
			mtot = mtot + CONM.M(1,1,n); % sum masses
			CALC = true;	
		%end
	end					

	if CALC

		NEWCG = S ./ mtot; % center of gravity

		for n=1:ntot

			%if NODEINDEX(CONM.Node(n)) % check if node is structural 

			% translate everything to new CG in the BASIC reference frame
			v = (COORD(CONM.Node(n),1:3) ) - NEWCG;
			J = J + ROT(:,:,CONM.Node(n)) * CONM.M(4:6,4:6,n) * ROT(:,:,CONM.Node(n))' - CONM.M(1,1,n) .* (crossm(v) * crossm(v))...
          + 2 * crossm(v) * ROT(:,:,CONM.Node(n))' * CONM.M(1:3,4:6,n) * ROT(:,:,CONM.Node(n));
			%end		
		end

		NEWCGM(1:3,1:3) = diag([mtot, mtot, mtot]);
		NEWCGM(4:6,4:6) = J;

		% calculate MASS matrix in the GRID POINT required by the user
		v = zeros(1,3);
    R = eye(3);
    if GRDPNT ~= 0
    	v = COORD(GRDPNT, 1:3);     
      R = ROT(:,:,GRDPNT);
		end
		v = R'*(NEWCG - v)';
		NEWMRP(1:3,1:3) = NEWCGM(1:3,1:3);
		NEWMRP(4:6,4:6) = NEWCGM(4:6,4:6);
		S = zeros(3,3);

		S(2,1) =  mtot * v(3);
		S(3,1) = -mtot * v(2);
		S(1,2) = -S(2,1);
		S(3,2) =  mtot * v(1);
		S(1,3) = -S(3,1);
		S(2,3) = -S(3,2);

		NEWMRP(4:6, 1:3) = S;
		NEWMRP(1:3, 4:6) = S';
		NEWMRP(4:6,4:6)  = R'*NEWCGM(4:6,4:6)*R - mtot .* (crossm(v) * crossm(v)); % translate to reference point
	end

end
