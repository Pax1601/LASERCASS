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


function [YD, FAD, RM, AER] = spline_coefficients(Kfreq, AER)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% COEFFICIENT SPLINE: routine used to calculate spline coefficients used to
% interpolate generalized arodynamic forces at given reduced frequencies.
% 
% INPUT: 
%        Kfreq - reduced frequency vector
%        AER - QHHL reordered (odd-even) pages Matrix
%
% OUTPUT: YAD:
%         FAD:
%         AER:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nmodc = size(AER, 1);
nmodc2 = size(AER, 2);
FAD = zeros(nmodc,nmodc2,2);
YD = zeros(nmodc,nmodc2,2);
NKfreq = length(Kfreq);
RM = zeros(2*NKfreq,1);
RM(1) = 0;
IPAG = 0;
NF2 = NKfreq*2;
NF1 = NKfreq -1;

for L=1:NF1

  ELM1 = 2./(Kfreq(L+1)-Kfreq(L));
  L2 = L+L;
  RM(L2-1) = RM(L2-1)+ELM1+ELM1;
  RM(L2) = ELM1;
  RM(L2+1) = ELM1+ELM1;

end;

RM(NF2) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check on RM(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (RM(1)<=0)
    IERR = 28;
    return;
end

RM(1) = 1./(sqrt(RM(1)));
J = 0;

for I=2:NKfreq

    J = J+2;
    RM(J) = RM(J)*RM(J-1);
    RM(J+1) = RM(J+1)-RM(J)*RM(J);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check on RM(J+1)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if(RM(J+1)<=0)

        IERR=28;
        return;

    end;

    RM(J+1) = 1./sqrt(RM(J+1));

end;

IA = 1;
IB = 2;

IPAG =1;

for NR = 1:nmodc

    for NC = 1:nmodc2

        FAD(NR,NC,IA) = AER(NR,NC,IPAG);

    end;

end;

for L = 1:NF1

    ELM1 = 2./(Kfreq(L+1)-Kfreq(L));
    IPAG = 2*L +1;

    for NR = 1:nmodc

        for NC = 1:nmodc2

            FAD(NR,NC,IB) = AER(NR,NC,IPAG);

        end;
    end;

    for I = 1:nmodc

        for K = 1:nmodc2

            ELM2 = 1.5*ELM1*ELM1*(FAD(I,K,IA)-FAD(I,K,IB));
            YD(I,K,IA) = YD(I,K,IA)-ELM2;
            YD(I,K,IB) = -ELM2;

        end;

    end;

    IPAG = IPAG-1;
   
    for NR = 1:nmodc
        
        for NC = 1:nmodc2
        
            AER(NR,NC,IPAG)=YD(NR,NC,IA);
    
        end;
    
    end;
    
    IAB = IA;
    IA = IB;
    IB = IAB;
    
end;

IPAG = NF2;

for NR = 1:nmodc

    for NC= 1:nmodc2
    
        AER(NR,NC,IPAG) = YD(NR,NC,IA);
    
    end;

end;

for NR=1:nmodc

    for NC=1:nmodc2

        YD(NR,NC,IA) = AER(NR,NC,2);

    end;

end;

for I=1:nmodc

    for K=1:nmodc2

        YD(I,K,IA) = YD(I,K,IA)*RM(1);

    end;

end;

for NR = 1:nmodc

    for NC = 1:nmodc2

        AER(NR,NC,2) = YD(NR,NC,IA);

    end;

end;

KK = 0;

for II = 2:NKfreq

    IPAG = 2*II;
    for NR = 1:nmodc

        for NC = 1:nmodc2

            YD(NR,NC,IB) = AER(NR,NC,IPAG);

        end;

    end;

    KK = KK+2;
    for I = 1:nmodc

        for K = 1:nmodc2

            YD(I,K,IB) = (YD(I,K,IB)-YD(I,K,IA)*RM(KK))*RM(KK+1);

        end;

    end;

    for NR = 1:nmodc

        for NC = 1:nmodc2

            AER(NR,NC,IPAG) = YD(NR,NC,IB);

        end;

    end;

    IAB = IA;
    IA = IB;
    IB = IAB;

end;

YD = zeros(nmodc,nmodc,IA);

for II = 1:NKfreq

    J = NKfreq+1-II;
    KK = J+J;
    IPAG = 2*J;

    for NR = 1:nmodc

        for NC = 1:nmodc2

            YD(NR,NC,IB) = AER(NR,NC,IPAG);

        end;

    end;

    for I = 1:nmodc

        for K = 1:nmodc2

            YD(I,K,IB) = (YD(I,K,IB)-YD(I,K,IA)*RM(KK))*RM(KK-1);

        end;

    end;

    for NR = 1:nmodc

        for NC = 1:nmodc2

            AER(NR,NC,IPAG) = YD(NR,NC,IB);

        end;

    end;

    IAB = IA;
    IA = IB;
    IB = IAB;

end;
return;
