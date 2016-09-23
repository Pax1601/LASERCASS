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
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
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
%   Author: Alessandro Scotti
%   Modification 28/10/10 (Travaglini) vectorial computation
%
%***********************************************************************************************************************
%
% Calculates by interpolation/extrapolation the generalized forces and their derivatives
% with respect to the reduced frequency
%
%***********************************************************************************************************************
%
function[FORINT, DERINT] = aero_interp_vec(AER, Method, VALFRE, Kfreq)
%
nmodc = size(AER, 1);
nmodc2 = size(AER, 2);
NKfreq = length(Kfreq);
%
switch(Method)
    case {1,3}
        Method1 = 1;
    case{2}
        Method1 = 2;
end
cic = 0;
FREQ = abs(VALFRE);
NG2 = nmodc * nmodc2;
NWORDS = NG2*2;
FORINT=zeros(nmodc,nmodc2);
DERINT=zeros(nmodc,nmodc2);
L = 1:NG2;
%
% Determine if extrapolation is required
% for I = 2:NKfreq
%     if (FREQ < Kfreq(I))
%         cic = 1;
%         break;
%     end
% end
ind = find(FREQ<Kfreq(2:end));
if ~isempty(ind)
    I = ind(1)+1;
    cic = 1;    
end
%
% Interpolation
if (cic == 1)
    DELTA = Kfreq(I) - Kfreq(I-1);
    X = (FREQ - Kfreq(I - 1))/DELTA;
    A1 = 1. -3*X^2 + 2*X^3;
    A2 = DELTA*(X - 2*X^2 + X^3);
    A3 = 3*X^2 - 2*X^3;
    A4 = (-X^2 + X^3)*DELTA;
    D1 = 6*(X^2 - X)/DELTA;
    D2 = 1-4*X + 3*X^2;
    D4 = -2*X + 3*X^2;
    IPAG = 2*I - 2;
%     NDER = IPAG*NG2 +L;
%     NAER = (IPAG-1)*NG2 + L;
    if Method1 == 1
        DERINT(:,:) = AER(:,:,IPAG-1)*D1 + AER(:,:,IPAG)*D2;
        FORINT(:,:) = (AER(:,:,IPAG-1)*A1 + AER(:,:,IPAG)*A2);
    else
        FORINT(:,:) = (AER(:,:,IPAG-1)*A1 + AER(:,:,IPAG)*A2);
    end
    
    IPAG = 2*I ;
%     NDER = IPAG*NG2 + L;
%     NAER = (IPAG - 1)*NG2 + L;
    if (Method1 == 1);
        DERINT(:,:) = (DERINT(:,:) - AER(:,:,IPAG-1)*D1 + AER(:,:,IPAG)*D4);
        FORINT(:,:) = (FORINT(:,:) + AER(:,:,IPAG-1)*A3 + AER(:,:,IPAG)*A4);
    else
        FORINT(:,:) = (FORINT(:,:) + AER(:,:,IPAG-1)*A3 + AER(:,:,IPAG)*A4);
    end
    
else % extrapolation
    FR = FREQ-Kfreq(NKfreq);
    IPAG = 2*NKfreq ;
%     NDER = IPAG*NG2 + L;
%     NAER = (IPAG - 1)*NG2 + L;
    if (Method1 == 1)
        DERINT(:,:) = (AER(:,:,IPAG));
        FORINT(:,:) = (AER(:,:,IPAG-1) + FR*AER(:,:,IPAG));
    else
        FORINT(:,:) = (AER(:,:,IPAG-1) + FR*AER(:,:,IPAG));
    end
end
% Revert sign in interpolation if negative reduced frequency is found
if (VALFRE < 0)
    L=2:2:NG2;
    if (Method1 == 1)
        I = L-1;
        DERINT(I) = -DERINT(I);
        FORINT(L) = -FORINT(L);
    else
        FORINT(L) = -FORINT(L);
    end
    
%     for L=2:2:NG2
%         if (Method1 == 1)
%             I = L-1;
%             DERINT(I) = -DERINT(I);
%             FORINT(L) = -FORINT(L);
%         else
%             FORINT(L) = -FORINT(L);
%         end
%     end
end
%
