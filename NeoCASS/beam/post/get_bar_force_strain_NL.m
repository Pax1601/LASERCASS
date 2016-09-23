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

function [STRAIN, FORCE, CStresses, SafeM] = get_bar_force_strain_NL(nbar, Bar, PBar, Mat, Node, SOL, NR, BARR, PO, KR)

	STRAIN = zeros(2, 6, nbar);
	FORCE = zeros(2, 6, nbar);
	% store beam stresses
  CStresses = [];
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

		n1 = Bar.Conn(n, 1);
		n2 = Bar.Conn(n, 2);
		n3 = Bar.Conn(n, 3);

		% offset global coords
		f1 = Node.R(:,:, n1) * Bar.Offset(n, 1:3)';
		f2 = Node.R(:,:, n2) * Bar.Offset(n, 4:6)';
		f3 = Node.R(:,:, n3) * Bar.Offset(n, 7:9)';
		
		x = [Node.Coord(n1,1)+f1(1) Node.Coord(n2,1)+f2(1) Node.Coord(n3,1)+f3(1)];
		y = [Node.Coord(n1,2)+f1(2) Node.Coord(n2,2)+f2(2) Node.Coord(n3,2)+f3(2)];
		z = [Node.Coord(n1,3)+f1(3) Node.Coord(n2,3)+f2(1) Node.Coord(n3,3)+f3(3)];

			% COLLOC 1
		eta = -1/sqrt(3);
		NIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC1
		NI = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
		NI1 = NI(1) .* eye(3); NI2 = NI(2) .* eye(3); NI3 = NI(3) .* eye(3);
		% COLLOC2
		eta = +1/sqrt(3);
		NIIp = [(0.5 * (2 * eta-1)) (-2 * eta) (0.5 * (2 * eta+1))]; % shape functions derivative evaluated in COLLOC2 
		NII = [(0.5 * eta * (eta-1)) (1-eta^2) (0.5 * eta * (eta+1))];
		NII1 = NII(1) .* eye(3); NII2 = NII(2) .* eye(3); NII3 = NII(3) .* eye(3);

		JI(1) = dot(NIp, x);
		JI(2) = dot(NIp, y);
		JI(3) = dot(NIp, z);
		NIp = NIp ./ norm(JI);
		JII(1) = dot(NIIp, x);
		JII(2) = dot(NIIp, y);
		JII(3) = dot(NIIp, z);
		NIIp = NIIp ./ norm(JII);

		NID1 = NIp(1) .* eye(3); NID2 = NIp(2) .* eye(3); NID3 = NIp(3) .* eye(3);
		NIID1 = NIIp(1) .* eye(3); NIID2 = NIIp(2) .* eye(3); NIID3 = NIIp(3) .* eye(3);

		% offset global coords
		f1 = NR(:,:, n1) * Bar.Offset(n, 1:3)';
		f2 = NR(:,:, n2) * Bar.Offset(n, 4:6)';
		f3 = NR(:,:, n3) * Bar.Offset(n, 7:9)';
		% node global coords
		c1 = f1 + Node.Coord(n1,:)' + SOL(n1, 1:3)';
		c2 = f2 + Node.Coord(n2,:)' + SOL(n2, 1:3)';
		c3 = f3 + Node.Coord(n3,:)' + SOL(n3, 1:3)';
		
		r1 = SOL(n1,4:6)';
		r2 = SOL(n2,4:6)';
		r3 = SOL(n3,4:6)';
		
		gI = NI1 * r1 + NI2 * r2 + NI3 * r3;
		gII = NII1 * r1 + NII2 * r2 + NII3 * r3;
		
		STRAIN(1, 1:3, n) = ( (BARR(:, :, 4, n))' * (NID1 * c1 + NID2 * c2 + NID3 * c3) )'                  - PO(1,:,n);
		STRAIN(2, 1:3, n) = ( (BARR(:, :, 5, n))' * (NIID1 * c1 + NIID2 * c2 + NIID3 * c3) )'               - PO(2,:,n);

		STRAIN(1, 4:6, n) = ( (BARR(:, :, 4, n))' * Gmat (gI) * ( NID1 * r1 + NID2 * r2 + NID3 * r3) )'     + KR(1,:,n);
		STRAIN(2, 4:6, n) = ( (BARR(:, :, 5, n))' * Gmat (gII) * ( NIID1 * r1 + NIID2 * r2 + NIID3 * r3) )' + KR(2,:,n);

		FORCE(1,:, n) = ( Bar.D(:,:,1,n) * STRAIN(1,:,n)' )';
		FORCE(2,:, n) = ( Bar.D(:,:,2,n) * STRAIN(2,:,n)' )';
    F = [FORCE(1,:,n), FORCE(2,:,n)]';
    p = Bar.PID(n);

    [CStresses.Norm(n,:), CStresses.Shear(n,:), SafeM.Tmax_Norm(n), SafeM.Tmin_Norm(n), SafeM.SM_Norm(n), ...
          SafeM.Tmax_Shear(n), SafeM.Tmin_Shear(n), SafeM.SM_Shear(n), SafeM.SM_SBuck(n), SafeM.SM_PBuck(n)] = stress_recovery(F, p, PBar, Mat, 0);


%    [CStresses.Norm(:,:,n), CStresses.Shear(:,:,n), SafeM.Tmax_Norm(n), SafeM.Tmin_Norm(n), SafeM.SM_Norm(n), ...
%          SafeM.Tmax_Shear(n), SafeM.Tmin_Shear(n), SafeM.SM_Shear(n), SafeM.SM_SBuck(n), SafeM.SM_PBuck(n)] = stress_recovery(F, p, PBar, Mat);

	end

end
