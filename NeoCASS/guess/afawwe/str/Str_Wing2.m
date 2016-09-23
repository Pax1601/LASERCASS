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
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Structural analysis modulus applied to wing2 structural box.
%
% Weight estimation is computed as a:
%   - vector station-by-station for the wing structural box;
%   - a single value (scalar) for the carrythrough structure;
% The outpout contains structural box and carrythorough structure weights
% as 2 separated quantities.
%
% Called by:    AFaWWE.m
%
% Calls:
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str, optim] = Str_Wing2(pdcylin, aircraft, geo, loads, str, max_loads, optim)


%--------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------

str.wing2.dW        = [];    % optimum web spacing                           [m] , vector
str.wing2.tCbar     = [];    % equivalent isotropic thickness of the covers  [m] , vector
str.wing2.tWbar     = [];    % equivalent isotropic thickness of the webs    [m] , vector
str.wing2.tgC       = [];    % cover gage thicknesses                        [m] , vector
str.wing2.tgW       = [];    % web gage thicknesses                          [m] , vector
str.wing2.tC        = [];    % maximum cover thickness                       [m] , vector
str.wing2.tW        = [];    % maximum web thickness                         [m] , vector
str.wing2.nrW       = [];    % number of webs
str.wing2.WBEND     = [];    % weight of bending material                    [kg], vector
str.wing2.WSHEAR    = [];    % weight of shear material                      [kg], vector
str.wing2.WBOX      = [];    % ideal weight of 1 wing2 box structure          [kg], vector
str.wing2.WBENDC    = [];    % weight of bending material                    [kg], scalar
str.wing2.WSHEARC   = [];    % weight of shear material                      [kg], scalar
str.wing2.WTORSIONC = [];    % weight of torsion material                    [kg], scalar
str.wing2.WC        = [];    % weight of carrythrough-structure              [kg], scalar
loads.wing2.FS      = [];    % resulting shear force                         [N]  , vector
loads.wing2.M       = [];    % resulting bending moment                      [N*m], vector

% KS = 1.25;
% loads.wing2.FS = KS*max_loads.wing2.Tz;
% loads.wing2.M  = KS*max_loads.wing2.Mx;

% Check if loads and geo data are compatible and refer to CT values as well
nload = length(loads.wing2.M);
ngeo  = length(geo.wing2.tbs);
if ngeo > nload
  offset = ngeo-nload +1;
  Zs     = geo.wing2.Zs(offset:end);
  tbs    = geo.wing2.tbs(offset:end);
else
  Zs  = geo.wing2.Zs;
  tbs = geo.wing2.tbs;
end

% Size following the chosen structural concept
if pdcylin.wing2.kcon <= 6
    
    % Carrythrough element number
    nct = pdcylin.stick.nwing2_carryth;
 
    % Optimal frame spacing
    str.wing2.dW = tbs .*( (1-2*geo.wing2.ec)/((1-geo.wing2.ec)*sqrt(2*geo.wing2.epw)) .*...
        (loads.wing2.M./(Zs.*(tbs*0.5).^2*pdcylin.wing2.esw)) .^...
        ((2*geo.wing2.ec-3)/(2*geo.wing2.ec)) *...
        geo.wing2.epc ^(3/(2*geo.wing2.ec)) ).^(2*geo.wing2.ec/(4*geo.wing2.ec-3));
%   avoid spikes at tip node
    str.wing2.dW(end) = str.wing2.dW(end-1);
    Ind = find(str.wing2.dW ~= 0);
    str.wing2.tCbar = zeros(geo.wing2.leny, 1);
    
    % Cover thickness
    str.wing2.tCbar(Ind) = str.wing2.dW(Ind) .*( loads.wing2.M(Ind)./(Zs(Ind).*tbs(Ind)*0.5*pdcylin.wing2.esw*...
        geo.wing2.epc.*str.wing2.dW(Ind)) ) .^(1/geo.wing2.ec);
    
    % Web thickness
    str.wing2.tWbar = tbs .*sqrt( (loads.wing2.M./(Zs.*(tbs*0.5).^2*pdcylin.wing2.esw)) .^...
        (2-1/geo.wing2.ec) .*...
        (geo.wing2.epc.*str.wing2.dW./(tbs*0.5)) .^(1/geo.wing2.ec).*(2/geo.wing2.epw) );
    
    % Gauge thicknesses
    str.wing2.tgC = (pdcylin.wing2.tmgw/geo.wing2.Kgc).*ones(geo.wing2.leny, 1);
    str.wing2.tgW = (pdcylin.wing2.tmgw/geo.wing2.Kgw).*ones(geo.wing2.leny, 1);
    
    % Maximum web thickness
    str.wing2.tW  = max([str.wing2.tWbar'; str.wing2.tgW'])';
    
    % Number of webs
    str.wing2.nrW = ceil(Zs(Ind)./str.wing2.dW(Ind)) + 1;
    
    % Sizing due to bending
    Jd   = 2*1/12.*tbs.^3 + 2.*Zs.*(tbs./2).^2 ;
    fos  = 1.0;
    smax = pdcylin.wing2.fcsw;
    str.wing2.tbC = fos.*loads.wing2.M./Jd .*tbs./2 ./smax;
    
    % Maximum cover thickness
    str.wing2.tC = max([str.wing2.tCbar'; str.wing2.tgC'])';
%     str.wing2.tC = max([str.wing2.tCbar'; str.wing2.tgC'; str.wing2.tbC'])';
    
    % Section areas (BAR number + 1) [m2]
    Aw = str.wing.nrW.*geo.wing.tbs.*str.wing.tW;       % Web area
    Ac = 2.*(geo.wing.Zs + geo.wing.tbs).*str.wing.tC;  % Cover area
    
    % Mass (BAR number) [kg]
    m2 = pdcylin.wing.dsw.*diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(Ac+Aw);
    
    %--------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE wing2 STRUCTURAL BOX
    %--------------------------------------------------------------------------
    
    % Bending material linear density (BAR number + 1) [kg/m]
    WBENDp = pdcylin.wing2.dsw *Zs .*tbs *geo.wing2.ep .*...
        (abs(loads.wing2.M)./(Zs .*tbs.^2 *pdcylin.wing2.esw)).^geo.wing2.e;
    WBENDp2 = pdcylin.wing2.dsw*abs(loads.wing2.M)./(0.8*tbs*smax);
    WBENDp3 = pdcylin.wing2.dsw*str.wing2.tbC.*(Zs)*2;  
    
    % Bending material mass (BAR number) [kg]
    str.wing2.WBEND  = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
    str.wing2.WBEND2 = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WBENDp2);
    str.wing2.WBEND3 = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WBENDp3);
    
    % Shear material linear density (BAR number + 1) [kg/m]
    WSHEARp = pdcylin.wing2.dsw *abs(loads.wing2.FS) /pdcylin.wing2.fcsw;
     
    % Shear material mass (BAR number) [kg]
    str.wing2.WSHEAR  = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
    
    % Load-carrying weight for semi-wing  
    m1 = str.wing2.WBEND + str.wing2.WSHEAR;
    
    % Resize web thickness
    A1  = (WBENDp + WSHEARp)/pdcylin.wing2.dsw;             % ideal bending and shear areas
    A2c = 2.*(Zs + tbs).*str.wing2.tC;  % cover area
    A2w = str.wing2.nrW.*tbs.*str.wing2.tW;       % web area
    tst = A1 > (A2c+A2w);
    %
    str.wing2.tW(tst) = (A1(tst) - A2c(tst))./(str.wing2.nrW(tst).*tbs(tst));
    
    % Total structural mass (BAR number) [kg]
    str.wing2.WBOX = max([m1'; m2'])';
    
    %--------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE CARRYTHROUGH STRUCTURE
    %--------------------------------------------------------------------------   
    Torque_bend = 0;
    Torque_shea = 0;
    for i = 1:length(geo.wing2.index)-1
        Torque_bend = Torque_bend + (loads.wing2.M(geo.wing2.index(i)) - loads.wing2.M(geo.wing2.index(i+1)))*...
            cos(geo.wing2.CAERO1.sweepC2(i)*pi/180);
        Torque_shea = Torque_shea + (loads.wing2.M(geo.wing2.index(i)) - loads.wing2.M(geo.wing2.index(i+1)))*...
            sin(geo.wing2.CAERO1.sweepC2(i)*pi/180);
    end
    
    % Carry-through bending mass [kg]
    str.wing2.WBENDC    = pdcylin.wing2.dsw *geo.wing2.ep *geo.wing2.CSR *geo.wing2.tcs *(geo.fus.R*2) *...
        ( Torque_bend /(geo.wing2.tcs^2 *geo.wing2.CSR *pdcylin.wing2.esw) )^geo.wing2.e;
    
    % Carry-through shear mass [kg]
    str.wing2.WSHEARC   = pdcylin.wing2.dsw *loads.wing2.FS(1) /pdcylin.wing2.fcsw *(geo.fus.R*2);
    
    % Carry-through torsion mass [kg]
    str.wing2.WTORSIONC = pdcylin.wing2.dsw *Torque_shea *(geo.wing2.tcs + geo.wing2.CSR) *(geo.fus.R*2) /...
        ( geo.wing2.tcs *geo.wing2.CSR *pdcylin.wing2.fcsw );
    
    % Total carry-through mass [kg]
    str.wing2.WC        = str.wing2.WBENDC + str.wing2.WSHEARC + str.wing2.WTORSIONC;
    
else
    
    switch pdcylin.wing2.kcon
        
        case {9}
            % four design variables, twbs (shear ), Aeq
            % (bending), number of stringer (Nstr) and ttorq (torque)
            % Nstr is fized to four
            DEF_SPITCH = 0.16;
            DEF_RPITCH = 0.55;
            %
            try
                pdcylin.wing2.spitch;
            catch
                pdcylin.wing2.spitch = DEF_SPITCH;
                fprintf('\n\nWARNING: stringer pitch not defined in material_property.vtail.spitch');
            end
            try
                pdcylin.wing2.rpitch;
            catch
                pdcylin.wing2.rpitch = DEF_RPITCH;
                fprintf('\n\nWARNING: rib pitch not defined in material_property.vtail.rpitch');
            end
%           check data
            if (geo.wing2.y(end)<pdcylin.wing2.rpitch)
              fprintf('\n\t### Warning: rib pitch exceeds wing2 span.');
            end
            if (min(Zs)<pdcylin.wing2.spitch)
              fprintf('\n\t### Warning: stringer pitch exceeds minimum wing2 chord.');
              fprintf('\n\t             Two stringers will be used.');
            end
            %
            fprintf('\n\tOptimization parameters:\n\tstringer pitch: %f\n\trib pitch: %f\n',...
                pdcylin.vtail.spitch,pdcylin.vtail.rpitch);
            %
            Nsec = length(loads.vtail.M);
            str.wing2.Nstr = round(Zs/pdcylin.vtail.spitch)+1;
            index = find(str.wing2.Nstr<2);
            if (~isempty(index))
              str.wing2.Nstr(index) = 2;
            end
            str.wing2.ttorq = zeros(Nsec,1);
            str.wing2.twbs  = zeros(Nsec,1);
            str.wing2.Aeq   = zeros(Nsec,1);
            
            if (niter==1)
                XBK = 1.0e-3.*ones(3,1);
                XBK (3) = 1.0e-4;
            else
                XBK(1) = optim.wing2.ttorq(1);
                XBK(2) = optim.wing2.twbs(1);
                XBK(3) = optim.wing2.Aeq(1);
            end
            str.wing2.ttorq(1) = XBK(1);
            str.wing2.twbs(1) = XBK(2);
            str.wing2.Aeq(1) = XBK(3);
            %
            NITN = 5; KTRIAL = 10; PERT_AMPL = 10.0;
            str.wing2.WBOX_IT = zeros(NITN,1);
            %-------------------------------------------------------------------------------
            %           Size first section
            i=1;
            [SOL, flag, fun, CUNS] = optim_Zst_sk_panel(Zs(i), tbs(i), ...
                str.wing2.Nstr(i), pdcylin.wing2.rpitch, pdcylin.wing2.esw, pdcylin.wing2.msl, ...
                0.0, loads.wing2.FS(i), loads.wing2.M(i), loads.wing2.Mt(i),...
                [], XBK);
            SOL = XBK;
            if (~isempty(CUNS))
                fprintf('\n\n\t\t### Warning: optimization failed for section %d. ', i);
                fprintf('\n\t\t    Problem will be relaxed. ');
            end
            K=1;
            while ( (~isempty(CUNS)) && (K<=KTRIAL) )
                fprintf('\n\t\tTrial: %d.', K);
                CEVAL = CUNS;
                [SOL, flag, fun, CUNS] = optim_Zst_sk_panel(Zs(i), tbs(i), ...
                    str.wing2.Nstr(i), pdcylin.wing2.rpitch, pdcylin.wing2.esw, pdcylin.wing2.msl, ...
                    0.0, loads.wing2.FS(i), loads.wing2.M(i), loads.wing2.Mt(i),...
                    CEVAL, SOL);
                [SOL, flag, fun, CUNS] = optim_Zst_sk_panel(Zs(i), tbs(i), ...
                    str.wing2.Nstr(i), pdcylin.wing2.rpitch, pdcylin.wing2.esw, pdcylin.wing2.msl, ...
                    0.0, loads.wing2.FS(i), loads.wing2.M(i), loads.wing2.Mt(i),...
                    [], SOL);
                K = K + 1;
                if (isempty(CUNS))
                    fprintf(' Feasible.');
                end
            end
            %
            if ( (~isempty(CUNS)) && (K==11) )
                XBK = PERT_AMPL .* XBK;
                fprintf('\n\t\t### Warning: unable to find an initial feasible solution.');
            else
                XBK = SOL;
            end
            %
            for NIT = 2:NITN
                %
                [SOL, flag, fun, CUNS] = optim_Zst_sk_panel(Zs(i), tbs(i), ...
                    str.wing2.Nstr(i), pdcylin.wing2.rpitch, pdcylin.wing2.esw, pdcylin.wing2.msl, ...
                    0.0, loads.wing2.FS(i), loads.wing2.M(i), loads.wing2.Mt(i),...
                    [], XBK);
                if (flag>0)
                    str.wing2.ttorq(i) = SOL(1);
                    str.wing2.twbs(i)  = SOL(2);
                    str.wing2.Aeq(i)   = SOL(3);
                else
                    str.wing2.ttorq(i) = PERT_AMPL*XBK(1);
                    str.wing2.twbs(i)  = PERT_AMPL*XBK(2);
                    str.wing2.Aeq(i)   = PERT_AMPL*XBK(3);
                    fprintf('\n\t\tNegative flag for section %d: %g', i, flag);
                end
                XBK(1) = str.wing2.ttorq(i);
                XBK(2)  = str.wing2.twbs(i);
                XBK(3)   = str.wing2.Aeq(i);
                str.wing2.sec_whisto(i, NIT) = fun;
            end
            if (~isempty(CUNS))
                error('Solution not feasible. Try to change spitch and/or rpitch.');
            end
            %
            for i=2:Nsec
                for NIT = 1:NITN
                    [SOL, flag, fun, CUNS] = optim_Zst_sk_panel(Zs(i), tbs(i), ...
                        str.wing2.Nstr(i), pdcylin.wing2.rpitch, pdcylin.wing2.esw, pdcylin.wing2.msl, ...
                        0.0, loads.wing2.FS(i), loads.wing2.M(i), loads.wing2.Mt(i),...
                        [], XBK);
                    if (flag>0)
                        str.wing2.ttorq(i) = SOL(1);
                        str.wing2.twbs(i)  = SOL(2);
                        str.wing2.Aeq(i)   = SOL(3);
                    else
                        str.wing2.ttorq(i) = PERT_AMPL*XBK(1);
                        str.wing2.twbs(i)  = PERT_AMPL*XBK(2);
                        str.wing2.Aeq(i)   = PERT_AMPL*XBK(3);
                        fprintf('\n\t\tNegative flag for section %d: %g', i, flag);
                    end
                    XBK(1) = str.wing2.ttorq(i);
                    XBK(2)  = str.wing2.twbs(i);
                    XBK(3)   = str.wing2.Aeq(i);
                    str.wing2.sec_whisto(i, NIT) = fun;
                end
                if (~isempty(CUNS))
                    error('Solution not feasible. Try to change spitch and/or rpitch.');
                end
            end
            %
            optim.wing2.ttorq(:,niter) = str.wing2.ttorq;
            optim.wing2.twbs(:,niter) = str.wing2.twbs;
            optim.wing2.Aeq(:,niter) = str.wing2.Aeq;
            %
            % Carrythrough element number
            nct = pdcylin.stick.nwing2_carryth;
            
            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.wing2.Aeq.*str.wing2.Nstr + str.wing2.ttorq.*Zs)*2*pdcylin.wing2.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.wing2.WBEND = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.wing2.twbs.*tbs*2*pdcylin.wing2.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.wing2.WSHEAR  = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.wing2.ttorq.*Zs*2*pdcylin.wing2.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.wing2.WTORQUE  = diff(geo.wing2.y_nodes_thick(nct+1:end)).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.wing2.WBOX = str.wing2.WSHEAR + str.wing2.WBEND;
            
            % First section linear densities (note that geo.htail.Zs(1) is the chord
            % along carry-thourgh axis and not projected along cantilever
            % part axis, used instead of Zs(1))
            D = 2*geo.fus.R;
            WBENDpC   = (str.wing2.Aeq(1).*str.wing2.Nstr(1) + str.wing2.ttorq(1).*Zs(1))*2*pdcylin.wing2.dsw;
            WSHEARpC  = str.wing2.twbs(1).*tbs(1)*2*pdcylin.wing2.dsw;
            WTORQUEpC = str.wing2.ttorq(1).*Zs(1)*2*pdcylin.wing2.dsw + WSHEARpC;
            
            str.wing2.WBENDC    = WBENDpC*D;
            str.wing2.WSHEARC   = WSHEARpC*D;
            str.wing2.WTORSIONC = WTORQUEpC*D;
            str.wing2.WC        = str.wing2.WBENDC + str.wing2.WSHEARC;
            
        otherwise
            error('Wrong value of kcon for wing2.');
            
    end
    
end