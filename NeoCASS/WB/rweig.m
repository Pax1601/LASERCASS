%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080101      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function       Script called by weight_xml main function
%
%
%   DESCRIPTION: Now compute the centre of gravity locations for various
%                loading scenarios
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%         
%                
%    REFERENCES:
%
%**************************************************************************


% MEW centre of gravity
sumweig = 0.0;

sumwemx = 0.0;
%-------------------------------------
sumwemy = 0.0; % MISSING in the code!!
%-------------------------------------
sumwemz = 0.0;

for i=1:25

    if i ~= 17 && i ~= 18 && i ~= 19 && i ~= 20 && i ~= 22 && i ~= 23 && i ~= 24 && i ~= 25
        
        % this calculations neglects wing fuel, centre tank fuel, auxiliary tank fuel
        % individual passenger collective weight and collective baggage weight
        sumweig = sumweig + PLOTCGS(i, 4, DRWSELN);% weight accumulator
        
        % weight-moment accumulator for longitudinal axis
        sumwemx = sumwemx + PLOTCGS(i, 1, DRWSELN)*PLOTCGS(i, 4, DRWSELN);
        
        % weight-moment accumulator for vertical axis
        sumwemz = sumwemz + PLOTCGS(i, 3, DRWSELN)*PLOTCGS(i, 4, DRWSELN);

        % weight-moment accumulator for lateral axis
        sumwemy = sumwemy + PLOTCGS(i, 2, DRWSELN)*PLOTCGS(i, 4, DRWSELN);
        
    end
    
end


% COECOGX(DRWSELN) = 100*( sumwemx/sumweig - ...
%                         (REFWAPX(DRWSELN)*fuselgt(DRWSELN) + ...
%                          REFWYBR(DRWSELN)*wingspn(DRWSELN)/2*tan(kdrang*REFWLSW(DRWSELN))) )/REFWMAC(DRWSELN);
%              
% COECOGZ(DRWSELN) = 100*sumwemz/( sumweig*fusevma(DRWSELN) );

% Aircraft MEW CoG
COECOGX(DRWSELN) = (sumwemx/sumweig)/fuselgt(DRWSELN);
COECOGY(DRWSELN) = 0.0;
COECOGZ(DRWSELN) = (sumwemz/sumweig)/fusevma(DRWSELN);

% Output
PLOTCGS(29, 1, DRWSELN) = sumwemx/sumweig;
PLOTCGS(29, 3, DRWSELN) = sumwemz/sumweig;

fprintf('\n\tCoG for MEW configuration:\n');
fprintf('\tX-position [m]: %10.2f\n', PLOTCGS(29, 1, DRWSELN));
fprintf('\tY-position [m]: %10.2f\n', PLOTCGS(29, 2, DRWSELN));
fprintf('\tZ-position [m]: %10.2f\n', PLOTCGS(29, 3, DRWSELN));

%==========================================================================

% maximum payload centre of gravity - MTOW
% build the fueling schedule

tankts1 = FMPYWEI(DRWSELN) - PLOTCGS(18, 4, DRWSELN);
tankts2 = tankts1 - PLOTCGS(19, 4, DRWSELN);
tankts3 = tankts2 - PLOTCGS(20, 4, DRWSELN);
tankwe1 = 0.0;
tankwe2 = 0.0;
tankwe3 = 0.0;

if tankts1 > 0.0001
    
    % fill the integral wing tanks to max capacity
    tankwe1 = PLOTCGS(18, 4, DRWSELN);
    
    if tankts2 > 0.0001
        
        % fill the centre tank to max capacity
        tankwe2 = PLOTCGS(19, 4, DRWSELN);
        
    else
        
        tankwe2 = abs(tankts1);
        
    end
    
    if tankts3 > 0.0001
        
        % fill the auxiliary tank to max capacity
        tankwe3 = PLOTCGS(20, 4, DRWSELN);
        
    else
        
        tankwe3 = abs(tankts2);
        
    end
    
else
    
    % partially filled integral wing tanks
    tankwe1 = FMPYWEI(DRWSELN);
    
end

sumweig = sumweig + tankwe1 + tankwe2 + tankwe3;   % weight accumulator

% weight-moment accumulator for longitudinal axis
sumwemx = sumwemx + PLOTCGS(18, 1, DRWSELN)*tankwe1 + PLOTCGS(19, 1, DRWSELN)*tankwe2 + ...
                                                      PLOTCGS(20, 1, DRWSELN)*tankwe3;
      
% weight-moment accumulator for vertical axis
sumwemz = sumwemz + PLOTCGS(18, 3, DRWSELN)*tankwe1 + PLOTCGS(19, 3, DRWSELN)*tankwe2 + ...
                                                      PLOTCGS(20, 3, DRWSELN)*tankwe3;
      
% address the impact of crew, operating items and payload
% weight accumulator
sumweig = sumweig + PLOTCGS(17, 4, DRWSELN) + PLOTCGS(22, 4, DRWSELN) + ...
                    PLOTCGS(23, 4, DRWSELN) + PLOTCGS(24, 4, DRWSELN) + PLOTCGS(25, 4, DRWSELN);
      
% weight-moment accumulator for longitudinal axis
sumwemx = sumwemx + PLOTCGS(17, 1, DRWSELN)*PLOTCGS(17, 4, DRWSELN) + ...
                    PLOTCGS(22, 1, DRWSELN)*PLOTCGS(22, 4, DRWSELN) + ...
                    PLOTCGS(23, 1, DRWSELN)*PLOTCGS(23, 4, DRWSELN) + ...
                    PLOTCGS(24, 1, DRWSELN)*PLOTCGS(24, 4, DRWSELN) + ...
                    PLOTCGS(25, 1, DRWSELN)*PLOTCGS(25, 4, DRWSELN);
                
% weight-moment accumulator for vertical axis
sumwemz = sumwemz + PLOTCGS(17, 3, DRWSELN)*PLOTCGS(17, 4, DRWSELN) + ...
                    PLOTCGS(22, 3, DRWSELN)*PLOTCGS(22, 4, DRWSELN) + ...
                    PLOTCGS(23, 3, DRWSELN)*PLOTCGS(23, 4, DRWSELN) + ...
                    PLOTCGS(24, 3, DRWSELN)*PLOTCGS(24, 4, DRWSELN) + ...
                    PLOTCGS(25, 3, DRWSELN)*PLOTCGS(25, 4, DRWSELN);
                
% CMPCOGX(DRWSELN) = 100*( sumwemx/sumweig - ...
%                         (REFWAPX(DRWSELN)*fuselgt(DRWSELN) + ...
%                          REFWYBR(DRWSELN)*wingspn(DRWSELN)/2*tan(kdrang*REFWLSW(DRWSELN))) )/REFWMAC(DRWSELN);
%                      
% CMPCOGZ(DRWSELN) = 100*sumwemz/( sumweig*fusevma(DRWSELN) );

% Aircraft MTOW CoG
CMPCOGX(DRWSELN) = (sumwemx/sumweig)/fuselgt(DRWSELN);
CMPCOGY(DRWSELN) = 0.0;     
CMPCOGZ(DRWSELN) = (sumwemz/sumweig)/fusevma(DRWSELN);

% Output
PLOTCGS(27, 1, DRWSELN) = sumwemx/sumweig;
PLOTCGS(27, 3, DRWSELN) = sumwemz/sumweig;

% SR 18/07/2010 Printout of CoG position for MTOW configuration

fprintf('\n\tCoG for MTOW configuration:\n');
fprintf('\tX-position [m]: %10.2f\n', PLOTCGS(27, 1, DRWSELN));
fprintf('\tY-position [m]: %10.2f\n', PLOTCGS(27, 2, DRWSELN));
fprintf('\tZ-position [m]: %10.2f\n', PLOTCGS(27, 3, DRWSELN));

% LR 20/09/2012 Print out MAC position and value

[MAC, YMAC, XLEMAC, XACMAC] = RecoverMACInfo(aircraft);
fprintf('\n\tMean Aerodynamic Chord data:\n');
fprintf('\tMAC value  [m]: %10.2f\n', MAC);
fprintf('\tMAC LE X   [m]: %10.2f\n', XLEMAC);
fprintf('\tMAC AC X   [m]: %10.2f\n', XACMAC);

% Print out the x-position of CoG w.r.t. MAC leading edge 
fprintf('\n\t(XCG-XLEMAC)/MAC [%%]:\n');
fprintf('\tMEW  [%%]: %10.2f\n', abs(PLOTCGS(29, 1, DRWSELN)-XLEMAC)/MAC*100);
fprintf('\tMTOW [%%]: %10.2f\n', abs(PLOTCGS(27, 1, DRWSELN)-XLEMAC)/MAC*100);

% END OF FUNCTION BODY
%-------------------------------------------------------------------------------------------------------------------------------------------------------------
