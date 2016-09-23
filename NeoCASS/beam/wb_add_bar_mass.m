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
% Calculates rigid body mass matrix due to bar and beam elements
% This function is called after set_conm_mass function
% It can be called to determine new intertia during the deformation
%
function [NEWCG, NEWCGM, NEWMRP] = wb_add_bar_mass(ntot, COORD, ROT, CG, GRDPNT, MCG, MRP, ELEM)

if ntot >0

	S = zeros(1,3);
	J = zeros(3,3);
	mtot = MCG(1,1);
	J = MCG(4:6,4:6);
  EL_CONTR = false;

	for n=1:ntot
		
		offset = zeros(1,3);
		for j=1:3
      if (ELEM.M(1,1,j,n) > 0.0)
			  offset(3) =  ELEM.M(5,1,j,n) / ELEM.M(1,1,j,n);
			  offset(2) =  -ELEM.M(6,1,j,n) / ELEM.M(1,1,j,n);
			  offset(1) =  ELEM.M(6,2,j,n) / ELEM.M(1,1,j,n);
			  offset = (ROT(:,:,ELEM.Conn(n,j)) * offset')';
			  % get new static moments respect to the old CG
			  S = S + ELEM.M(1,1,j,n) .* (COORD(ELEM.Conn(n,j),1:3) + offset - CG);
			  mtot = mtot + ELEM.M(1,1,j,n);
        EL_CONTR = true;
      end
		end
	end					

  if (~EL_CONTR)

    NEWCG = CG;
    NEWCGM = MCG;
    NEWMRP = MRP;

  else

	  NEWCG = S ./ mtot + CG; % get aircraft new CG

	  for n=1:ntot

		  v = zeros(1,3);
		  for j=1:3

        if (ELEM.M(1,1,j,n) > 0.0)

			  v(3) =  ELEM.M(5,1,j,n) / ELEM.M(1,1,j,n);
			  v(2) =  -ELEM.M(6,1,j,n) / ELEM.M(1,1,j,n);
			  v(1) =  ELEM.M(6,2,j,n) / ELEM.M(1,1,j,n);

%			  v = (ROT(:,:,ELEM.Conn(n,j)) * v')';
			  v = COORD(ELEM.Conn(n,j),1:3) - NEWCG;

			  J = J + ROT(:,:,ELEM.Conn(n,j)) * ELEM.M(4:6,4:6,j,n) * ROT(:,:,ELEM.Conn(n,j))' - ...
			          ELEM.M(1,1,j,n) .* (crossm(v) * crossm(v)) +...
                2 * crossm(v) * ROT(:,:,ELEM.Conn(n,j))' * ELEM.M(1:3,4:6,j,n)* ROT(:,:,ELEM.Conn(n,j)); 
        end
		  end
	  end

	  NEWCGM(1:3,1:3) = diag([mtot, mtot, mtot]);
	  NEWCGM(4:6,4:6) = J;

	  % calculate MASS matrix in the GRID POINT required by the user
	  v = zeros(1,3);
    R = eye(3);
	  if GRDPNT ~= 0

		  v = COORD(GRDPNT, 1:3);
    
	  end

	  NEWMRP = zeros(6,6);
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

else

  NEWCG = CG;
  NEWCGM = MCG; 
  NEWMRP = MRP;

end
