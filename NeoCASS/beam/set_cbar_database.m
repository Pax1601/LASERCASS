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
%     080101      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
function BAR = set_cbar_database(ncbar, BAR, PBAR, MAT, NODE, COORD)

	for n=1:ncbar

		% assembly stiffnes matrix		
		p = BAR.PID(n);
		i = PBAR.Mat(p);
		rho = MAT.Rho(i) + PBAR.RhoNSV(p); % structural density
		L = zeros(1,3); % body length
		L(1) = norm(NODE.Coord(BAR.Conn(n,1),:) + BAR.Offset(n, 1:3) - BAR.Colloc(1, :, n));
		L(2) = norm(BAR.Colloc(1, :, n) - BAR.Colloc(2, :, n));
		L(3) = norm(NODE.Coord(BAR.Conn(n,3),:) + BAR.Offset(n, 7:9) - BAR.Colloc(2, :, n));
		%
		
		EA  =  PBAR.A(p) * MAT.E(i);

		Kshear = PBAR.Kshear(p,:);

		if Kshear(1) == 0
		
			GAy =  EA;
		else
		
			GAy =  PBAR.A(p) * MAT.G(i) * Kshear(1);
		
		end

		if Kshear(2) == 0
		
			GAz =  EA;
		else
		
			GAz =  PBAR.A(p) * MAT.G(i) * Kshear(2);
		
		end
		
		GJ  =  PBAR.J(p) * MAT.G(i);
		EJy =  PBAR.I(p,2) * MAT.E(i);
		EJz =  PBAR.I(p,1) * MAT.E(i);
		EJyz = PBAR.I(p,3) * MAT.E(i);
		%
		D = diag([EA, GAy, GAz, GJ, EJy, EJz]);
		% set moment
		D(5,6) = EJyz;
		D(6,5) = D(5,6);
		% store section stiffness matrices in the two collocation points
		BAR.D(:, :,1, n) = D;
		BAR.D(:, :,2, n) = D;

		% set lumped nodes mass matrix
		M  = zeros(1,3);
		J  = zeros(3,3);
		MJ = zeros(1,3);
		v  = zeros(1,3);
		Jx = zeros(1,3);
		M(1) = (PBAR.A(p) * rho  +  PBAR.RhoNS(p)) * L(1);
		M(2) = (PBAR.A(p) * rho  +  PBAR.RhoNS(p)) * L(2);
		M(3) = M(1);
		
		MJ(1) = 1/12 * (M(1) * L(1)^2);  % rotational inertia respect to CG1
		MJ(2) = 1/12 * (M(2) * L(2)^2);  % rotational inertia respect to CG2
		MJ(3) = MJ(1);                   % rotational inertia respect to CG3
		% inertia along neutral axis
		Jx(1) = ( (rho + PBAR.RhoNS(p)/PBAR.A(p)) * L(1) ) * (PBAR.I(p,1) + PBAR.I(p,2)); 
		Jx(2) = ( (rho + PBAR.RhoNS(p)/PBAR.A(p)) * L(2) ) * (PBAR.I(p,1) + PBAR.I(p,2));
		Jx(3) = Jx(1);
		
		BAR.M(1:3,1:3, 1, n) = diag([M(1), M(1), M(1)]);
		BAR.M(1:3,1:3, 2, n) = diag([M(2), M(2), M(2)]);
		BAR.M(1:3,1:3, 3, n) = BAR.M(1:3,1:3, 1, n);
		% body 1 MASS MATRIX in NODE reference frame
		S = zeros(3,3);
		v = L(1)/2 .* BAR.R(:,1,1,n)' + BAR.Offset(n, 1:3); % cg body 1
		S(2,1) =  M(1) * v(3); S(3,1) = -M(1) * v(2); S(1,2) = -S(2,1); S(3,2) =  M(1) * v(1); S(1,3) = -S(3,1); S(2,3) = -S(3,2);
		BAR.M(4:6,1:3,1,n) = S; BAR.M(1:3,4:6,1,n) = S';
		
		J = diag([Jx(1), MJ(1), MJ(1)]);
    % avoid round-off errors
    RJ = BAR.R(:, :, 1, n) * sqrt(J);
		BAR.M(4:6,4:6, 1, n) = RJ * RJ' - M(1) .* (crossm(v) * crossm(v));

		% body 2 MASS MATRIX in NODE reference frame
		S = zeros(3,3);
		v = BAR.Offset(n, 1:3); % cg body 2
		S(2,1) =  M(2) * v(3); S(3,1) = -M(2) * v(2); S(1,2) = -S(2,1); S(3,2) =  M(2) * v(1); S(1,3) = -S(3,1); S(2,3) = -S(3,2);
		BAR.M(4:6,1:3,2,n) = S; BAR.M(1:3,4:6,2,n) = S';

		J = diag([Jx(2), MJ(2), MJ(2)]);
    RJ = BAR.R(:, :, 2, n) * sqrt(J); 
		BAR.M(4:6,4:6,2, n) = RJ * RJ' - M(2) .* (crossm(v) * crossm(v));

		% body 3 MASS MATRIX in NODE reference frame
		S = zeros(3,3);
		v = -L(3)/2 .* BAR.R(:,1,3,n)' + BAR.Offset(n, 7:9); % cg body 3
		S(2,1) =  M(3) * v(3); S(3,1) = -M(3) * v(2); S(1,2) = -S(2,1); S(3,2) =  M(3) * v(1); S(1,3) = -S(3,1); S(2,3) = -S(3,2);
		BAR.M(4:6,1:3,3,n) = S; BAR.M(1:3,4:6,3,n) = S';

		J = diag([Jx(3), MJ(3), MJ(3)]);
    RJ = BAR.R(:, :, 3, n) * sqrt(J); 
		BAR.M(4:6,4:6,3, n) = RJ * RJ' - M(3) .* (crossm(v) * crossm(v));

	end % end CBAR loop

end
