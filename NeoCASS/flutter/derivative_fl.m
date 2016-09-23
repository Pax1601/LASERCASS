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



function d = derivative_fl(COEF,nmodc,PERM,SVQU6,FREQ)

nmodc1 = nmodc+1;

L = 0;
INIDO = 1;
A = zeros(6,1);

for N = INIDO:3
    
    NGLN = nmodc + N;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  SOLVES COMPLEX SISTEM (nmodc order) GETTING  DELTA#Q AS
    %  FUNCTIONS OF DELTASIGMA AND DELTA OMEGA; SOLUTION IS THEN MOVED
    %  IN THE COLUMN OF KNOWN TERMS...
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    [COEF(:,NGLN)] = comp_linsys_solver(COEF,COEF(:,NGLN),nmodc,nmodc1,PERM);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  SOLVES THE ADDED EQUATION AS A SYSTEM OF TWO REAL EQUATIONS WHERE DELTASIGMA
    %  AND DELTAOMEGA ARE UNKNOWNS, BY SUBSTISTUTION OF DELTAQ OBTAINED BY COMPLEX_LINEAR_SOLVER
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    TEMP1 = real(COEF(nmodc1,NGLN));
    TEMP2 = imag(COEF(nmodc1,NGLN));
    
    for J = 1:nmodc
        
        TEMP1 = TEMP1 - real(COEF(nmodc1,J))*real(COEF(J,NGLN))+ imag(COEF(nmodc1,J))*imag(COEF(J,NGLN));
        TEMP2 = TEMP2-imag(COEF(nmodc1,J))*real(COEF(J,NGLN))- real(COEF(nmodc1,J))*imag(COEF(J,NGLN));
        
    end;
    
    L = L+1;
    A(L) = TEMP1;
    L = L + 1;
    A(L) = TEMP2;
    
    
    
end;

A11 = A(1);
A21 = A(2);
A12 = A(3);
A22 = A(4);
A13 = A(5);
A23 = A(6);
%-----------------
DISCR = A11*A22 - A12*A21;
TEMP1 = (A13*A22-A23*A12)/DISCR;
TEMP2 = (A23*A11-A13*A21)/DISCR;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      TEMP1 AND TEMP2 CONTAINS DELTASIGMA AND DELTAOMEGA RESPECTIVELY,
%      USED DURING SOLUTION METHOD 1 AND 2. WHEN METHOD 3 IS USED,
%      SIGMA AND OMEGA DERIVATIVES WITH RESPECT TO VELOCITY ARE HERE
%      PLACED;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


CT = (TEMP1+1i*TEMP2);
SVDR = CT;
TEMP1 = imag(SVDR);
TEMP2 = real(SVDR);

% SVQU(6) = SVQU(1)/FREQ;

d = (TEMP2-SVQU6*TEMP1)/FREQ ;
% SVDR(nmodc1) = CT;
