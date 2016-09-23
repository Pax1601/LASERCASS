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

function [IX,IY,IZ,IXZ] = scinert(B,W,F,T,E1,E2,LFIN,LTAIL,WB,WW,WF,WT, ...
    WE,WFUEL,WP,LCG,HCG,IXIN,IYIN,IZIN,IXZIN,AINCI,WGT,LFUEL)

% decide if it is a coarse approximation
if WW <= 0.0

    if IXIN <= 0.0
        IX = 0.0984*(W(4)^2)*WGT;    % approx. Ix
    else
        IX = IXIN;                    % default to user input
    end

    if IYIN <= 0.0
        IY = 0.0289*WGT*((B(3)+B(4))^2);% approx. Iy
    else
        IY = IYIN;                    % default to user input
    end

    if IZIN <= 0.0
        IZ = 0.0256*WGT*( ((B(3)+B(4))^2) + 4.0*W(4)^2 );% approx. Iz
    else
        IZ = IZIN;                    % default to user input
    end
    
else

%     fprintf('\tRefined approximation (w.r.t datum - reference wing apex):\n');

    % moment of inertia of wing plus fuel
    D1 = W(1);
    D2 = W(4)*tan(W(3)/57.296);
    D3 = D2 + W(2);
    
    if (D2-D1) <= 0.0

        C1 = D2;

        if (D1-D3) <= 0.0

            C2 = D1;
            C3 = D3;

        else

            C2 = D3;
            C3 = D1;

        end

    else

        C1 = D1;
        C2 = D2;
        C3 = D3;

    end

    RO = 2.0*WW/(-C1+C2+C3);   % sort of linear density (?) [Kg/m]
    WX = 0.1667*RO*(-C1^2 + C2^2 + C3^2 + C2*C3); % Kgm

    % Here change of notation to clarify the code: renamed D1 with I_T
    I_T = 0.08333*RO*(C1^3 + C2^3 + C3^3 + C2*C3^2 + C2^2*C3); % Kgm^2    
    IYW = 0.703*(I_T - (WX^2)/WW) + WW*(WX/WW - LCG)^2;   
    IXW = 0.125*WW*(W(4)^2)*(W(1)+3.0*W(2))/(W(1)+W(2));
    
    fprintf('\t - Wing:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXW, IYW, IXW+IYW);
    
    % Same as previous: D1 renamed with CF (correction factor); here is
    % unclear the unit of CF, isn't it supposed to be adimensional?

    % Original formula
    %    CF = 1.0 + (LFUEL^2)*WFUEL/WW;

    % Instead, putting CF in this way it's more likely a fuel mass
    % correction to structural mass (fuel distributed like structure)
    CF = 1.0 + (WFUEL/WW);

    IXW = CF*IXW;
    IYW = CF*IYW;
    IZW = IXW + IYW;

    fprintf('\t - Wing plus fuel:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXW, IYW, IZW);
    
    % moment of inertia of body
    D1 = 0.5*(B(5) + B(6));
    D2 = 0.5*(B(3)-B(4)) - LCG;
    D3 = B(3) - B(1);
    R = (D3^2 + 0.25*D1^2)/D1;

    % Modified here to avoid "complex" inertia: the real meaning of this
    % line remain unclear, better knowledge of Mitchell's code is needed...

    % Original formula
    % E = ((B(4)-B(2))^2-0.25*D1*D1)/(B(4)-B(2))^0.5;

    % Now E is adimensional as required by asin()
    E = sqrt( (B(4)-B(2))^2-0.25*D1*D1 )/(B(4)-B(2));
    SBW = 1.5708*D1*(0.5*D1 + (B(4)-B(2))*asin(E)/E + 2.0*(B(1)+B(2))) + ...
        4.4429*R*( 1.4142*D3 -(R-0.5*D1)*log((1.4142*R+D3)/(1.4142*R-D3)) );
    IYB = 0.0235*WB*SBW*( 1.5*D1/(B(3)+B(4))+(B(3)+B(4))/D1 ) + WB*D2^2;
    IXB = 0.15*WB*( (0.318*SBW/(B(3)+B(4)))^2 ) + WB*HCG^2;
    IZB = IYB;
    
    fprintf('\t - Fuselage:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXB, IYB, IZB);
    
    % moment of inertia of tail
    IYT = WT*LTAIL^2;
    IXT = 0.15*(WT*T(5)^2)*(T(2)+3.0*T(3))/(T(2)+T(3)) + WT*((F(6)-HCG)^2);
    IZT = IYT + IXT;

    fprintf('\t - Tail:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXT, IYT, IZT);
    
    % moment of inertia of fin
    D1 = 0.333*F(5)*(F(2)+2.0*F(3))/(F(2)+F(3));
    IXF = 0.04*WF*(F(5)^2)*(1.0+2.0*F(2)*F(3)/((F(2)+F(3))^2)) + WF*D1^2;
    IZF = WF*LFIN^2;
    IYF = IXF + IZF;

    fprintf('\t - Fin:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXF, IYF, IZF);
    
    % moment of inertia of engines
    IYE = (0.5*WE)*( ((E1(2)-LCG)^2)+((E2(2)-LCG)^2) );
    IXE = (0.5*WE)*( E1(1)^2 + E2(1)^2 );
    IZE = IXE + IYE;
    
    fprintf('\t - Engines:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXE, IYE, IZE);
    
    % moment of inertia of payload
    D1 = 0.5*(B(1) + B(2) + B(3) + B(4));
    D2 = 0.5*D1 - 0.5*(B(2)+B(4)) - LCG;
    IZP = WP*(0.08333*D1^2 + D2^2);
    IYP = IZP;
    IXP = 0.;
    
    fprintf('\t - Payload:\n\tIxx = %12.8g\n\tIyy = %12.8g\n\tIzz = %12.8g\n',...
        IXP, IYP, IZP);
    
    if IXIN <= 0.0
        IX = IXW + IXB + IXF + IXT + IXE;         % more accurate Ix
    else
        IX = IXIN;                                % default to user input
    end

    if IYIN <= 0.0
        IY = IYW + IYB + IYF + IYT + IYE + IYP;   % more accurate Iy
    else
        IY = IYIN;                                % default to user input
    end

    if IZIN <= 0.0
        IZ = IZW + IZB + IZF + IZT + IZE + IZP;   % more accurate Iz
    else
        IZ = IZIN;                                % default to user input
    end

end

if IXZIN <= 0.0
    IXZ = (IZ-IX)*sin(AINCI/57.3);     % compute Ixz regardless of accuracy
end
