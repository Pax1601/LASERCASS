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
function [SLAVE_DOF, COEFF, NEWFIXED] = set_constr_eq(nc, CONTROL, FIXED, nlink, LINK)

SLAVE_DOF = zeros(1, nc);
COEFF     = zeros(1, nc);

slave_found = false; 
master_found = false;

NEWFIXED = FIXED;

for i = 1:nlink

	slave_found = false;
	master_found = false;

	for j = 1:nc % slave loop
	
		if strcmp(cell2mat(CONTROL.Name(j)), cell2mat(LINK.Slave(i)))
		
			slave_found = true;
	
			for k = 1:nc % master loop
			
				if k~=j
				
					if strcmp(cell2mat(CONTROL.Name(k)), cell2mat(LINK.Master(i)))
							
						master_found = true;
						NEWFIXED(j) = uint8(1);
						SLAVE_DOF(j) = k; 
						if COEFF(j)
							error('Control surface %s already constrained by AELINK card.', cell2mat(LINK.Slave(i)));
						end
						
						COEFF(j) = LINK.Coeff(i);
						if FIXED(j)
							error('Constraint equation applied to a control surface already contrained by current TRIM card.');
						end
						
						if COEFF(k)
							error('Control surface %s is declared both as master and slave.', cell2mat(CONTROL.Name(k)));
						end
					
					end
				end
			
			end % end master loop
			
			if ~master_found
				error('Unable to find independent control surface %s.', cell2mat(LINK.Master(i)));
			end
		
		end
	end % end slave loop

	if ~slave_found
		error('Unable to find dependent control surface %s.', cell2mat(LINK.Slave(i)));
	end

end % end link loop

end
