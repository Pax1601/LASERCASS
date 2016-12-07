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
%      FATTORIZZA LA MATRICE COMPLESSA JA)
%
%        NRDIM  --> DIMENSIONI EFFETTIVE DELLA MATRICE [A]
%        N      --> ORDINE DEL SISTEMA
%
%      ATTENZIONE : QUESTA ROUTINE FATTORIZZA LA MATRICE JA) TENENDO
%      CONTO ANCHE DELLA SUA RIGA N + 1 PER ELIMINARE LA SINGOLARITA'
%      CHE SI PRESENTA NEL CALCOLO DEL FLUTTER.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [A, PERM, INDER] = comp_mat_fac(A, N, NRDIM);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      MODIFICA PER EVITARE LA SINGOLARITA'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

NR = N + 1;
PERM = 0;
% look for minimum number in each row and store its reciprocal
for I=1:NR
  X = 0;
  KI = I;
  for K=1:N
    if (abs(A(KI)) < X);
      KI = KI + NRDIM;
    else
      X = abs(A(KI));
      KI = KI + NRDIM;
    end
  end
  if (X == 0);
    INDER = 1;
    return;
  else
    PERM(I) = 1./X;
  end
end
%
IC=0;

for I = 1:N % column loop
  IM1 = I-1;
  IP1 = I+1;
  IPVT = I;
  X = 0;
  for K=I:NR % row loop
    KI = IC+K;
    S = A(KI);
    if (I == 1) 

    else
      KJ = K;
      for J=1:IM1
        IJ = IC+J;
        S = S - A(KJ) * A(IJ);
        KJ = KJ + NRDIM;
      end
      A(KI) = S;
    end

    if (X > (abs(S)*PERM(K)))
    else
      IPVT = K;
      X = abs(S)*PERM(K);
    end
  end

  if (X <= 0)
    INDER = 1;
    return
  end

  if (IPVT == I)

    else

        KI = IPVT;
        IJ = I;

        for J=1:N

            S = A(KI);
            A(KI) = A(IJ);
            A(IJ) = S;
            KI = KI+NRDIM;
            IJ = IJ+NRDIM;

        end;

        PERM(IPVT) = PERM(I);

    end;

    PERM(I) = IPVT;

    if (I == N)

    else

        IJ = IC+I;
        C = A(IJ);
        KC = IC+NRDIM;

        for K = IP1:NR

            KI = IC+K;
            A(KI) = A(KI)/C;

            if (K > N)
            else
                if (I == 1)
                else
                    IJ = I;
                    KI = KC+I;
                    S = A(KI);
                    for J=1:IM1
                        KJ = KC+J;
                        S = S-A(IJ)*A(KJ);
                        IJ = IJ+NRDIM;
                    end;
                    A(KI) = S;
                end;
            end;
            KC = KC+NRDIM;
        end;
    end;

    IC = IC+NRDIM;

end

KC = IC-NRDIM;
IC = KC+N;
KC = KC+NR;
A(KC) = A(KC)/A(IC);

for J=1:N

    I = N-J+1;
    KC = (I-1)*NRDIM;
    IC = NR;
    S = 0+i*0;

    for K=1:I

        S = S+A(IC)*A(KC+K);
        IC = IC+NRDIM;

    end;

    A(KC+NR) = S;

end;

INDER = 0;

