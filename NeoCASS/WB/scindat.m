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

function [LB,LW,LF,LT,LE1,LE2,IXI,IYI,IZI,IXZI,WB,WW,WF,WTP,WE,WFUEL,WP,LFUEL] ...
         = scindat(z, IXXINER, IYYINER, IZZINER, IXZINER, ...
                   kdrang, fuselgt, aftslgt, fusevma, forelgt, fusehma,...
                   RAMPWEI, WINGWEI, VTALWEI, HTALWEI, PYLNWEI, POWPWEI,...
                   NACEWEI, DMFWWEI, PLOTCGS,...
                   WSPNMTX, wingspn, REFWLSW, wingplc, REFWARE, REFWTAP, REFWTHB, wingdih,...
                   REFWAPX, engeylc, NACELGT, nacemdi,...
                   vtalapx, vtalspn, VTAWLSW, vtalver, VTAWARE, VTAWTAP, VTALTHB, VSPNMTX,...
                   htallsw, htalapx, htalver, htalspn, HTAWLSW, HTAWARE, HTAWTAP, HSPNMTX, PMFWWEI)
               
if z < 1
      error('scindat:no_aircraft', 'At least one airplane must be select!!');
else   
    
    %======================================================================
    % Copied here from qgeotry.m: needed for REFHAPX, REFVCAP, REFVCRC
    % definitions.

    % locate reference vertical tail down to the fuselage reference plane
    REFVCAP(z) = (vtalapx(z)*fuselgt(z)- ...
                  fusevma(z)*abs(vtalver(z))*tan(kdrang*VTAWLSW(z)))/fuselgt(z);
    % reference vertical tail root chord at fuselage reference plane
    REFVCRC(z) = qxcdcop(-fusevma(z)*abs(vtalver(z)),VSPNMTX, ...
                         2*VTAWARE(z)/(vtalspn(z)*(1+VTAWTAP(z))),VTAWTAP,z);
    
    % compute the moment arm to h-tail 0.25MAC and the volume coefficient
    xibad = HSPNMTX(1)*tan(kdrang*htallsw(1,z));% inboard sweep adjustment
    xmbad = HSPNMTX(2)*tan(kdrang*htallsw(2,z));% midboard sweep adjustment
    xobad = HSPNMTX(3)*tan(kdrang*htallsw(3,z));% outboard sweep adjustment  
    % locate the reference horizontal tail relative apex
    REFHAPX(z) = (htalapx(z)*fuselgt(z)+xibad+xmbad+xobad- ...
                  0.5*htalspn(z)*tan(kdrang*HTAWLSW(z)))/fuselgt(z);
    %======================================================================
    
    % central fuselage body length aft of reference wing apex
    LB(1) = fuselgt(z)*(1-REFWAPX(z))-aftslgt(z);
    % central fuselage body length forward of reference wing apex        
    LB(2) = fuselgt(z)*REFWAPX(z)-forelgt(z);
    % fuselage body length aft of reference wing apex         
    LB(3) = fuselgt(z)*(1-REFWAPX(z));
    % fuselage body length forward of reference wing apex         
    LB(4) = fuselgt(z)*REFWAPX(z);
    LB(5) = fusehma(z);       % fuselage cross-section horizontal diameter
    LB(6) = fusevma(z);       % fuselage cross-section vertical diameter
    % reference wing apex height from fuselage reference plane
    LB(7) = fusevma(z)*abs(0.5-wingplc(z));
    LB(8) = 0.0;              % main landing gear distance aft of reference wing apex         
    LB(9) = 0.0;              % body incidence on ground
    % reference wing root chord
    LW(1) = 2*REFWARE(z)/( wingspn(z)*(1 + REFWTAP(z)) );
    LW(2) = LW(1)*REFWTAP(z); % reference wing tip chord
    LW(3) = REFWLSW(z);       % reference wing leading edge sweep
    LW(4) = wingspn(z)/2;     % reference wing semi-span
    LW(5) = REFWTHB(z);       % reference wing mean t/c 
    LW(6) = 0.0;              % reference wing incidence angle
    % reference wing dihedral angle
    LW(7) = atan(2*(WSPNMTX(1)*tan(kdrang*wingdih(1,z))+ ...
                    WSPNMTX(2)*tan(kdrang*wingdih(2,z))+ ...
                    WSPNMTX(3)*tan(kdrang*wingdih(3,z)))/wingspn(z))/kdrang; 
    LW(8) = 0.3*LW(1);        % aileron hinge line extended to reference wing root chord
    LW(9) = 0.3*LW(2);        % aileron hinge line extended to reference wing tip chord
    LW(10) = 0.69*LW(4);      % semi-span to inboard aileron
    LW(11) = 0.98*LW(4);      % semi-span to outboard aileron
    % length from reference wing apex to reference vertical tail root chord at FRP
    LF(1) = fuselgt(z)*(REFVCAP(z)-REFWAPX(z));
    LF(2) = REFVCRC(z);       % reference vertical tail root chord at FRP
    LF(3) = 2*VTAWARE(z)/(vtalspn(z)*(1+ ...
            VTAWTAP(z)))*VTAWTAP(z);    % ref. vertical tail tip chord
    LF(4) = VTAWLSW(z);       % reference vertical tail leading edge sweep
    % reference vertical tail span extended down to fuselage reference plane
    LF(5) = vtalspn(z)+fusevma(z)*abs(vtalver(z));
    % reference horizontal tail height from fuselage reference plane
    LF(6) = fusevma(z)*abs(htalver(z));
    LF(7) = VTALTHB(z);       % reference vertical tail mean t/c
    % approximate fuselage top height at reference vertical tail Q.chd
    LF(8) = fusevma(z)*abs(vtalver(z));
    LF(9) = 0.0;              % rudder hinge line extended to reference vertical root chord 
    LF(10) = 0.0;             % rudder hinge line extended to reference vertical tip chord
    LF(11) = 0.0;             % height to lower rudder
    LF(12) = 0.0;             % height to upper rudder
    % length from reference wing apex to reference horizontal tail apex
    LT(1) = fuselgt(z)*(REFHAPX(z)-REFWAPX(z));
    % reference horizontal tail root chord
    LT(2) = 2*HTAWARE(z)/(htalspn(z)*(1+HTAWTAP(z)));
    LT(3) = 2*LT(2)*HTAWTAP(z); % reference horizontal tail tip chord
    LT(4) = HTAWLSW(z);         % reference horizontal tail leading edge sweep
    LT(5) = htalspn(z)/2;       % reference horizontal tail semi-span
    % the LE1s
    LE1(1) = engeylc(1,z)*wingspn(z)/2;  %fuse CL-primary powerplant 
    LE1(2) = 0.0;             % wing apex-primary engine c.g. (plan)
    LE1(3) = 0.0;             % fuse CL-primary engine c.g.(side)
    LE1(4) = 0.0;             % thrust parameter
    LE1(5) = 0.0;             % thrust parameter
    LE1(6) = 0.0;             % thrust parameter
    LE1(7) = NACELGT(1,z);    % primary nacelle length
    LE1(8) = nacemdi(1,z);    % primary nacelle maximum diameter
    % the LE2s
    LE2(1) = engeylc(2,z)*wingspn(z)/2;   %fuse CL-secondary powerplant 
    LE2(2) = 0.0;             % wing apex-secondary engine c.g. (plan)
    LE2(3) = 0.0;             % fuse CL-secondary engine c.g.(side)
    LE2(4) = 0.0;             % thrust parameter
    LE2(5) = 0.0;             % thrust parameter
    LE2(6) = 0.0;             % thrust parameter
    LE2(7) = NACELGT(2,z);    % secondary nacelle length
    LE2(8) = nacemdi(2,z);    % secondary nacelle maximum diameter
    
%     factor = 1E3;
    factor = 1;
    
    IXI = IXXINER(z)*factor;
    IYI = IYYINER(z)*factor;
    IZI = IZZINER(z)*factor;
    IXZI = IXZINER(z)*factor;
    
    WW = WINGWEI(z);
    WF = VTALWEI(z);
    WTP = HTALWEI(z);
    WE = PYLNWEI(z) + POWPWEI(z) + NACEWEI(z);
    WFUEL = DMFWWEI(z);    
    WP = PMFWWEI(z);

    % Fuel CG from nose apex
    CGF_N = sum(PLOTCGS(18:20, 1, z).*PLOTCGS(18:20, 4, z))/sum(PLOTCGS(18:20, 4, z)); 
    if ~isnan(CGF_N)
        % length to fuel tank cg from datum
        LFUEL = CGF_N - REFWAPX(z)*fuselgt(z);
    else
        LFUEL = REFWAPX(z)*fuselgt(z);
    end
    
    % equate the body weight as residual after deducting the above constituents
    WB = RAMPWEI(z) - WW - WF - WTP - WE - WFUEL - WP;
    
end


%==========================================================================
% Auxiliary functions

% Computes the local chord length at given span station
%          compute chord on actual planform geometry
%
function chord = qxcdcop(spanin, spanmtrx, rcrd, tapmtrx, z)

	if spanin == spanmtrx(1) || spanin == spanmtrx(1)+spanmtrx(2)
   		correct = -1;   % correction to heavyside result
	else
   		correct = 0;    % no correction to heavyside required
    end
    
	% identify the taper ratio for given wing segment
	taprs = tapmtrx(correct + 1 + qxheavy(spanin, spanmtrx(1), 1) + qxheavy(spanin, spanmtrx(1)+spanmtrx(2), 1), z);
	% identify the local span length for given wing segment           
	spanc = spanmtrx(correct + 1 + qxheavy(spanin, spanmtrx(1), 1) + qxheavy(spanin, spanmtrx(1)+spanmtrx(2), 1));
	% additional corrections before interpolation is executed 
    
	if spanin > spanmtrx(1) && spanin < spanmtrx(1)+spanmtrx(2)

   		spanin = spanin-spanmtrx(1);    % correct input span to local datum
   		taprs = taprs/tapmtrx(1,z);     % correct identified taper to local datum
   		rcrd = rcrd*tapmtrx(1,z);       % correct for local chord datum   

	elseif spanin >= spanmtrx(1)+spanmtrx(2)   

   		spanin = spanin-spanmtrx(1)-spanmtrx(2);    % correct input span to local datum
   		taprs = taprs/tapmtrx(2,z);     % correct identified taper to local datum
   		rcrd = rcrd*tapmtrx(2,z);       % correct for local chord datum   

	end
	
	chord = rcrd*(1-(1-taprs)*spanin/spanc);        % computed chord result

%--------------------------------------------------------------------------
% A heavyside step function to switch "on" or "off"
%
function comp1 = qxheavy(input, limit, type)
	
if type < 1
    comp1 = 0.5 + 0.5*tanh(110*(input - limit));
else
    comp1 = round(0.5+0.5*tanh(110*(input- limit)));
end