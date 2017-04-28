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
%
%
function [CForces, CStrains, CStresses, SafeM] = get_bar_force_strain(nbar, Bar, PBar, Mat, Node, NDispl, FUSE_DP)

	% store beam internal forces
	CForces = zeros(2, 6, nbar);
	% store beam strains and curvatures
	CStrains = zeros(2, 6, nbar);
	% store beam stresses
    CStresses = [];
	% store beam normal stresses
	CStresses.Norm = zeros(nbar,4);
	% store beam tangential stresses
	CStresses.Shear = zeros(nbar,4);
  %
  SafeM = [];
  SafeM.Tmax_Norm = zeros(nbar,1);  SafeM.Tmin_Norm = zeros(nbar,1);  SafeM.SM_Norm = zeros(nbar,1);  SafeM.SM_SBuck = zeros(nbar,1);
  SafeM.Tmax_Shear = zeros(nbar,1); SafeM.Tmin_Shear = zeros(nbar,1); SafeM.SM_Shear = zeros(nbar,1); SafeM.SM_PBuck = zeros(nbar,1);

	for n=1:nbar

    R = set_R_mat(Bar.R(:,:,4,n), Bar.R(:,:,5,n));
		%
		D = set_D_mat(Bar.D(:,:,1,n), Bar.D(:,:,2,n));
		%
		n1 = Bar.Conn(n, 1);
		n2 = Bar.Conn(n, 2);
		n3 = Bar.Conn(n, 3);
		% offset global coords
		f1 = Node.R(:,:, n1) * Bar.Offset(n, 1:3)';
		f2 = Node.R(:,:, n2) * Bar.Offset(n, 4:6)';
		f3 = Node.R(:,:, n3) * Bar.Offset(n, 7:9)';
		% node global coords
		c1 = f1 + Node.Coord(n1,:)';
		c2 = f2 + Node.Coord(n2,:)';
		c3 = f3 + Node.Coord(n3,:)';
		% set N matrix		
		%
		N = set_N_mat(c1, c2, c3, c1, c2, c3, f1, f2, f3);
		sol = zeros(18,1);
		sol = [NDispl(n1, 1:6), NDispl(n2, 1:6), NDispl(n3, 1:6)];
		% get strains
		E = R' * N * sol';
		% get internal forces
		F = D * E;
		% store forces
		CStrains(1, :, n) = E(1:6)';
		CStrains(2, :, n) = E(7:12)';

		CForces(1, :, n) = F(1:6)';
		CForces(2, :, n) = F(7:12)';
        p = Bar.PID(n);

        [CStresses.Norm(n,:), CStresses.Shear(n,:), SafeM.Tmax_Norm(n), SafeM.Tmin_Norm(n), SafeM.SM_Norm(n), ...
              SafeM.Tmax_Shear(n), SafeM.Tmin_Shear(n), SafeM.SM_Shear(n), SafeM.SM_SBuck(n), SafeM.SM_PBuck(n)] = stress_recovery(F, p, PBar, Mat, FUSE_DP);

	end

end
