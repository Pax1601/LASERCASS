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
%   Author: Luca Cavagna, Andrea Da Ronch, DIAPM
%***********************************************************************************************************************

function dlm_model = set_struct(fid, M, k, cref, symm, order, DX, DLM_NP)

fprintf(fid, '\n - Assemblying internal database...'); 
dlm_model = [];

% flight section
dlm_model.aero.M = [];          % Mach number
dlm_model.aero.beta = [];       % Prandtl-Glauert coefficient 
dlm_model.aero.k = [];          % reduced frequency = omega*dlm.b/2U
dlm_model.aero.V = [1 0 0];     % wind direction cosines
dlm_model.aero.cref = cref;     % reference length cref/2 is used for reduced frequency
% data section
dlm_model.data.D = [];
dlm_model.data.Cp = [];
dlm_model.data.Qhh = [];
dlm_model.data.c_displ = [];
dlm_model.data.dwnwash = [];
dlm_model.data.n_displ = [];
% param section
dlm_model.param.order = 0;      % 1 if quadratic, 2 if quartic
dlm_model.param.symm = 0;
dlm_model.param.MAXV = realmax;
dlm_model.param.CORE_RADIUS = 1.0e-3;
%
%***********************************************************************************************************************

if (~isempty(find(M >= 1.0)))
    error('Mach number out of upper range (1.0).');
end
if (~isempty(find(M < 0.0)))
    error('Mach number out of lower range (0.0).');
end

dlm_model.aero.M = unique(M);

dlm_model.aero.beta = sqrt(1 - M.^2);

if (isempty(k)) 
    error('No reduced frequency required.');
end

if (~isempty(find(k < 0.0)))
    error('Negative reduced frequency k required.');
end
dlm_model.aero.k = unique(k);
% check mesh discretization
KMAX = max(dlm_model.aero.k);
KMIN_DLM = min((2*pi/DLM_NP)./DX);
fprintf(fid,'\n\t Max red. frequency KMAX required: %g.', KMAX);
if (KMAX > KMIN_DLM)
  fprintf(fid,'\n\t### Warning: aerodynamic mesh too coarse to sample KMAX with %d points.', DLM_NP);
  fprintf(fid,'\n\t             Aerodynamic maximum red. frequency: %g. ', KMIN_DLM);
end
%
dlm_model.param.order = order;

switch (order)

	case 1
	
		fprintf(fid, '\n\t Doublet approximation: parabolic.');
	
	case 2

		fprintf(fid, '\n\t Doublet approximation: quartic.');
	
    otherwise
		
		fprintf(fid, '\n\t Wrong value for order approximation. Approximation set to parabolic (1) by default.');
		dlm_model.param.order = 1;

end

dlm_model.param.symm = symm;

switch (symm)

	case -1

		fprintf(fid, '\n\t Antisymmetry boundary condition along xz plane required.');

	case 0
	
		fprintf(fid, '\n\t No symmetry along xz plane required.');
	
	case 1

		fprintf(fid, '\n\t Symmetry boundary condition along xz plane required.');

    otherwise
		
		fprintf(fid, '\n\t Wrong value for symmetry boundary condition. Value set to 0 (no-symmetry) by default.');
		dlm_model.param.symm = 0;

end

fprintf(fid, '\n   done.');
