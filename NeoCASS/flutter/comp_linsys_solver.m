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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%      RISOLVE IL SISTEMA LINEARE COMPLESSO AX=B NOTA LA MATRICE
%      A FATTORIZZATA DALLA CFCTFL : LA SOLUZIONE E' POSTA NEL VETTORE
%      TERMINI NOTI B.
%
%      Input:
%      Output:
%      SUBROUTINE CSOLFL (A,B,N,NRDIM,PERM)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPLICIT REAL*8 (A-H,O-Z)
% COMPLEX*16 A,B,S
% C
% DIMENSION A(1),B(1),PERM(1)
% C
% DATA IK/0/
% C

function[B]=comp_linsys_solver(A,B,N,NRDIM,PERM);

IK = 0;

for I=1:N
  K = PERM(I);
  if (K == I)
  else % correctly permute row of the RHS
    S = B(K);
    B(K) = B(I);
    B(I) = S;
  end
end

for I=2:N
  IM1 = I-1;  
  S = B(I);
  IK = I;
  for K=1:IM1
    S = S-A(IK)*B(K); 
    IK = IK+NRDIM;
  end
  B(I) = S;
end

B(N) = B(N)/A(IK);

for I=2:N

    IM1 = N-I+1;
    INF = IM1+1;
    IK = (IM1-1)*NRDIM+IM1;
    IKS = IK;
    S = B(IM1);
    
    for K=INF:N
    
        IK = IK+NRDIM;
        S = S-A(IK)*B(K);
    
    end;

    B(IM1) = S/A(IKS);

end;
