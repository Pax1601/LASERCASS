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
%   Modified by Travaglini, vectoral operation.
%***********************************************************************************************************************

function [Q1] = Q1_approxT(M, beta, s, r, nu, k, lattice, deltap, CORE_RADIUS)

MAXV = realmax;
MINV = -MAXV;

x = deltap(:,1);
y = deltap(:,2);
z = deltap(:,3);

r1 = sqrt( ( y - nu).^2 + z.^2 );
R  = sqrt( ( x - nu*tan(lattice.lambda(s))).^2 + (beta * r1).^2 );
IND = find(abs(r1) < CORE_RADIUS);
u1 = ones(size(x));
% if s == 5
%     perch1 = 1;
% end
% if s == 7
%     stop = 1;
% end
if ~isempty(IND)
    % avoid singularity
    u1(IND(x(IND)>0)) = ones(size(x(x(IND)>0)))*MINV;
    u1(IND(x(IND)<=0)) = ones(size(x(x(IND)<=0)))*MAXV;
    
%     if x > 0
%         u1 = MINV;
%     else 
%         u1 = MAXV;
%     end
end
if length(IND)~=length(x)
    IND = find(abs(r1) >= CORE_RADIUS);
    u1(IND) = ( M * R(IND) - x(IND) + nu * tan(lattice.lambda(s)) )./ (beta^2 * r1(IND));
end

% k1 is non-dimensional because k and r1 are non-dimensional
k1 = r1*k;
IND = find(u1>=0);
I1 = zeros(size(k1));
if ~isempty(IND)
    INDM = find(u1(IND)==MAXV);
    I1(IND(INDM),:) = zeros(length(INDM),length(k));
    if length(INDM)~= length(IND)
       INDM = find(u1(IND)~=MAXV);
       I1(IND(INDM),:) =  I1approxT(u1(IND(INDM)), k1(IND(INDM),:));
    end
%     if u1 == MAXV
%         I1 = 0;
%     else
%         I1 = I1approx(u1, k1);
%     end
end
clear INDM
if length(IND)~=length(x)
    IND = find(u1<0);
    INDM = find(u1(IND)==MINV);
    if  ~isempty(INDM) 
        I1((IND(INDM)),:) = 2*real(I1approxT(zeros(size(u1(IND(INDM)) ) ), k1(IND(INDM),:)));
    end
    
    if length(INDM)~= length((IND))
        INDM = find(u1(IND)~=MINV);
       
        TOT = I1approxT(-u1(IND(INDM)),k1(IND(INDM),:));
        I1(IND(INDM),:) = 2*real(I1approxT(zeros(size(INDM)), k1(IND(INDM),:) ))...
                              - real(TOT) + 1i.* imag(TOT);
    end
%     if u1 == MINV
%         I1 = 2 .* real(I1approx(0, k1));
%     else
%         I1 = 2 .* real(I1approx(0, k1)) - real(I1approx(-u1, k1)) + i .* imag(I1approx(-u1, k1));
%     end
end

% Planar part of kernel numerator:
IND = find((u1 == MAXV | u1 == MINV));
K1 = zeros(size(I1));
if ~isempty(IND)
   K1(IND,:) = -I1(IND,:);
end
IND = find((u1  < MAXV & u1 > MINV));
if ~isempty(IND)
   K1(IND,:) = -I1(IND,:) - meshgrid(M * r1(IND)./ R(IND),k)'.* exp(-1i .* k1(IND,:).*meshgrid(u1(IND),k)')./ meshgrid(sqrt(1 + u1(IND).^2),k)';
end
% if (u1 == MAXV || u1 == MINV)
%     K1 = -I1;
% else if (u1 < MAXV && u1 > MINV)
%     K1 = -I1 - M * r1 / R .* exp(-i .* k1 .* u1) ./ sqrt(1 + u1^2);
%     end
% end
% Steady contribution
%K10 = -1-(x - nu * tan(lattice.lambda(s)))/R;
%K1 = K1 - K10;
%
T1 = cos(lattice.gamma(r) - lattice.gamma(s)); 
Q1 = ( K1.* exp(-1i  * (x - nu * tan(lattice.lambda(s)))* k ) ) .* meshgrid(T1,k)';
%
end
