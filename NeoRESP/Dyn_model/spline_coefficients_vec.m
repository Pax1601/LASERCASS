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


function [YD, FAD, RM, AER] = spline_coefficients_vec(Kfreq, AER)

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

ELM1 = 2./(diff(Kfreq));
RM(2:2:end-2) = ELM1;
% RM(1:2:end-3) = 2*ELM1;
RM(3:2:end-1) = 2*ELM1;
RM(1:2:end-3) = RM(1:2:end-3)+2*ELM1';
RM(NF2) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check on RM(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (RM(1)<=0)
    IERR = 28;
    return;
end

RM(1) = 1./(sqrt(RM(1)));
% RM(2:2:end-2) = RM(2:2:end-2).*RM(1:2:end-3);
% RM(3:2:end-1) = RM(3:2:end-1)-RM(2:2:end-2).^2;
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

if any(RM(3:2:end-1)<=0)
    IERR=28;
    return
end


IA = 1;
IB = 2;

IPAG =1;
FAD(:,:,IA) = AER(:,:,IPAG);


for L = 1:NF1
    
    ELM1 = 2./(Kfreq(L+1)-Kfreq(L));
    IPAG = 2*L +1;
    
    FAD(:,:,IB) = AER(:,:,IPAG);
    
    ELM2 = 1.5*ELM1*ELM1*(FAD(:,:,IA)-FAD(:,:,IB));
    YD(:,:,IA) = YD(:,:,IA)-ELM2;
    YD(:,:,IB) = -ELM2;
    
    IPAG = IPAG-1;
    
    AER(:,:,IPAG)=YD(:,:,IA);
    
    IAB = IA;
    IA = IB;
    IB = IAB;
    
end

IPAG = NF2;
AER(:,:,IPAG) = YD(:,:,IA);
YD(:,:,IA) = AER(:,:,2);
YD(:,:,IA) = YD(:,:,IA)*RM(1);
AER(:,:,2) = YD(:,:,IA);
KK = 0;

for II = 2:NKfreq
    
    IPAG = 2*II;
    
    YD(:,:,IB) = AER(:,:,IPAG);
    
    KK = KK+2;
    
    YD(:,:,IB) = (YD(:,:,IB)-YD(:,:,IA)*RM(KK))*RM(KK+1);
    
    AER(:,:,IPAG) = YD(:,:,IB);
    
    IAB = IA;
    IA = IB;
    IB = IAB;
    
end;

YD = zeros(nmodc,nmodc2,IA);

for II = 1:NKfreq
    
    J = NKfreq+1-II;
    KK = J+J;
    IPAG = 2*J;
    
    YD(:,:,IB) = AER(:,:,IPAG);
    YD(:,:,IB) = (YD(:,:,IB)-YD(:,:,IA)*RM(KK))*RM(KK-1);

    AER(:,:,IPAG) = YD(:,:,IB);
    
    IAB = IA;
    IA = IB;
    IB = IAB;
    
end;
return;
