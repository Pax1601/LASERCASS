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
%   Author: <andreadr@kth.se>
%
% Structural analysis modulus applied to HT structural box.
%
% Weight estimation is computed as a:
%   - vector station-by-station for the wing structural box;
%   - a single value (scalar) for the carrythrough structure;
% The outpout contains structural box and carrythorough structure weights
% as 2 separated quantities.
%
%
% Called by:    AFaWWE.m
%
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str, optim] = Str_Htail(niter, pdcylin, aircraft, geo, loads, str, optim)
%----------------------------------------------------------------------------------------
% Initialize structure: STR.HTAIL
%----------------------------------------------------------------------------------------
str.htail.WBEND     = [];    % weight of bending material                    [kg], vector
str.htail.WSHEAR    = [];    % weight of shear material                      [kg], vector
str.htail.WBOX      = [];    % ideal weight of 1 wing box structure          [kg], vector
str.htail.WBENDC    = [];    % weight of bending material                    [kg], scalar
str.htail.WSHEARC   = [];    % weight of shear material                      [kg], scalar
str.htail.WTORSIONC = [];    % weight of torsion material                    [kg], scalar
str.htail.WC        = [];    % weight of carrythrough-structure              [kg], scalar% Check if loads and geo data are compatible and refer to CT values as well
nload = length(loads.htail.M);
ngeo  = length(geo.htail.tbs);
if ngeo > nload
    offset = ngeo-nload +1;
    Zs     = geo.htail.Zs(offset:end);
    tbs    = geo.htail.tbs(offset:end);
else
    Zs  = geo.htail.Zs;
    tbs = geo.htail.tbs;
end
index = geo.htail.index(2:end)-(geo.htail.index(2)-1);
Nsec  = length(loads.htail.M);

% Check on carrythrough existence and recover element number
% When an aircraft has tailbooms or T-Tail twc=0
if geo.htail.twc
    nct = pdcylin.stick.nhtail_carryth;
else
    nct = 0;
end

% Size following the chosen structural concept
if pdcylin.htail.kcon <= 6
        
    str.htail.dW        = [];    % optimum web spacing                           [m] , vector
    str.htail.tCbar     = [];    % equivalent isotropic thickness of the covers  [m] , vector
    str.htail.tWbar     = [];    % equivalent isotropic thickness of the webs    [m] , vector
    str.htail.tgC       = [];    % cover gage thicknesses                        [m] , vector
    str.htail.tgW       = [];    % web gage thicknesses                          [m] , vector
    str.htail.tC        = [];    % maximum cover thickness                       [m] , vector
    str.htail.tW        = [];    % maximum web thickness                         [m] , vector
    str.htail.nrW       = [];    % number of webs

    % Optimal frame spacing
    str.htail.dW = tbs .*( (1-2*geo.htail.ec)/((1-geo.htail.ec)*sqrt(2*geo.htail.epw)) .*...
        (abs(loads.htail.M)./(Zs.*tbs.^2*pdcylin.htail.esw)) .^...
        ((2*geo.htail.ec-3)/(2*geo.htail.ec)) *...
        geo.htail.epc ^(3/(2*geo.htail.ec)) ).^(2*geo.htail.ec/(4*geo.htail.ec-3));
%   avoid spikes at tip node
    str.htail.dW(end) = str.htail.dW(end-1);
    % Skin thickness
    Ind  = find(str.htail.dW ~= 0);
    Ind1 = find(str.htail.dW == 0);
    str.htail.tCbar = zeros(Nsec, 1);
    str.htail.tCbar(Ind) = str.htail.dW(Ind) .*( abs(loads.htail.M(Ind))./(Zs(Ind).*tbs(Ind)*pdcylin.htail.esw*...
        geo.htail.epc.*str.htail.dW(Ind)) ) .^(1/geo.htail.ec);
    
    % Frame thickness
    str.htail.tWbar = tbs .*sqrt( (abs(loads.htail.M)./(Zs.*tbs.^2*pdcylin.htail.esw)) .^...
        (2-1/geo.htail.ec) .*...
        (geo.htail.epc.*str.htail.dW./tbs) .^(1/geo.htail.ec).*(2/geo.htail.epw) );
    
    % Gauge limits
    str.htail.tgC = (pdcylin.htail.tmgw/geo.htail.Kgc).*ones(Nsec, 1);
    str.htail.tgW = (pdcylin.htail.tmgw/geo.htail.Kgw).*ones(Nsec, 1);
    
    if ~isempty(Ind1)
        str.htail.tWbar(Ind1) = 0.0;
        str.htail.dW(Ind1)    = Zs(Ind1)+eps;
    end
    str.htail.tW  = max( [str.htail.tWbar'; str.htail.tgW'] )';
    str.htail.nrW = ceil(Zs./str.htail.dW) + 1;
    
    % Sizing due to bending
    smax = pdcylin.htail.fcsw;
    Jw   = str.htail.nrW.*(1./12.*str.htail.tW.*tbs.^3)./(0.25 .* Zs .* tbs.^2);
    J1   = abs(loads.htail.M)./(smax.*tbs);
    str.htail.tbC = J1 - Jw;
    str.htail.tC  = max( [str.htail.tCbar'; str.htail.tgC'; str.htail.tbC'] )';
    
    % Section areas (BAR number + 1) [m2]
    Aw = str.htail.nrW.*tbs.*str.htail.tW;       % Web area
    Ac = 2.*(Zs + tbs).*str.htail.tC;  % Cover area
    
    % Mass (BAR number) [kg]
    m2 = pdcylin.htail.dsw.*diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(Ac+Aw);
    
    %--------------------------------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE STRUCTURAL BOX
    %--------------------------------------------------------------------------------------------------
    
    % Bending material linear density (BAR number + 1) [kg/m]
    WBENDp = pdcylin.htail.dsw.*Zs .*tbs *geo.htail.ep .*...
        (abs(loads.htail.M) ./(Zs .*tbs.^2 *pdcylin.htail.esw)).^geo.htail.e;
    
    % Bending material mass (BAR number) [kg]
    str.htail.WBEND  = diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
    
    % Shear material linear density (BAR number + 1)
    WSHEARp = pdcylin.htail.dsw.*abs(loads.htail.FS) /pdcylin.htail.fcsw;
    
    % Shear material mass (BAR number) [kg]
    str.htail.WSHEAR = diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
    
    % Load-carrying weight for vertical tail [kg]
    m1 = str.htail.WBEND + str.htail.WSHEAR;
    
    % Resize web thickness
    A1  = (WBENDp + WSHEARp)/pdcylin.htail.dsw;             % ideal bending and shear areas
    A2c = 2.*(Zs + tbs).*str.htail.tC;  % cover area
    A2w = str.htail.nrW.*tbs.*str.htail.tW;       % web area
    tst = A1 > (A2c+A2w);
    %
    str.htail.tW(tst) = (A1(tst) - A2c(tst))./(str.htail.nrW(tst).*tbs(tst));
    
    % Total structural mass (BAR number) [kg]
    str.htail.WBOX = max([m1'; m2'])';
    
    % Check on carry-through existence
    if geo.htail.twc
        %--------------------------------------------------------------------------
        % TOTAL IDEAL WEIGHT OF THE CARRYTHROUGH STRUCTURE
        %--------------------------------------------------------------------------
        Torque_bend = 0;
        Torque_shea = 0;
        for i = 1:length(index)-1
            Torque_bend = Torque_bend + (loads.htail.M(index(i)) - loads.htail.M(index(i+1)))*...
                cos(geo.htail.CAERO1.sweepC2(i)*pi/180);
            Torque_shea = Torque_shea + (loads.htail.M(index(i)) - loads.htail.M(index(i+1)))*...
                sin(geo.htail.CAERO1.sweepC2(i)*pi/180);
        end
        
        % Carry-through bending mass [kg]
        str.htail.WBENDC    = pdcylin.htail.dsw *geo.htail.ep *geo.htail.CSR *geo.htail.tcs *(geo.fus.R*2) *...
            ( abs(Torque_bend) /((geo.htail.tcs*0.5)^2 *geo.htail.CSR *pdcylin.htail.esw) )^geo.htail.e;
        
        % Carry-through shear mass [kg]
        str.htail.WSHEARC   = pdcylin.htail.dsw *abs(loads.htail.FS(1)) /pdcylin.htail.fcsw *(geo.fus.R*2);
        
        % Carry-through torsion mass [kg]
        str.htail.WTORSIONC = pdcylin.htail.dsw * abs(Torque_shea) *((geo.htail.tcs*0.5) + geo.htail.CSR) *(geo.fus.R*2) /...
            ( geo.htail.tcs *geo.htail.CSR *pdcylin.htail.fcsw );
        
        % Total carry-through mass [kg]
        str.htail.WC        = str.htail.WBENDC + str.htail.WSHEARC + str.htail.WTORSIONC;
        
    else
        str.htail.WC = 0;
    end
    
else
    
    switch pdcylin.htail.kcon
        
        case {9}
          pdcylin.htail = check_input_params(pdcylin.htail, geo.htail);

            fprintf('\n\tOptimization parameters:\n\t- stringer pitch: %f.\n\t- rib pitch: %f.',...
                         pdcylin.htail.spitch,pdcylin.htail.rpitch);
%
            if (niter==1)
              optim.htail.skin = [];
              optim.htail.web = [];
            end
%
            outps = 'bk_optim_Zst_sk_panel_h.mat';
            [str.htail.skin, str.htail.web] = run_optim_Zst_sk_panel_9(niter, optim.htail.skin, optim.htail.web, Zs, tbs, pdcylin.htail.esw, pdcylin.htail.msl, loads.htail.N,...
                                              loads.htail.FS, loads.htail.M, loads.htail.Mt, pdcylin.htail.rpitch, pdcylin.htail.spitch, outps);
%
            optim.htail.skin.tskin(:,niter) =  str.htail.skin.tskin;
            optim.htail.skin.Astr(:,niter)  =  str.htail.skin.Astr;     
            optim.htail.web.tw(:,niter)     =  str.htail.web.tw;       
            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.htail.skin.Astr.*str.htail.skin.Nstr + str.htail.skin.tskin.*Zs)*2*pdcylin.htail.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.htail.WBEND = diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.htail.web.tw.*tbs*2*pdcylin.htail.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.htail.WSHEAR  = diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.htail.skin.tskin.*Zs*2*pdcylin.htail.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.htail.WTORQUE  = diff(geo.htail.y_nodes_thick(nct+1:end)).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.htail.WBOX = str.htail.WSHEAR + str.htail.WBEND;
            
            if geo.htail.twc
                % First section linear densities (note that Zs(1) is the chord
                % along carry-thourgh axis and not projected along cantilever
                % part axis, used instead of Zs(1))
                WBENDpC   = (str.htail.skin.Astr(1).*str.htail.skin.Nstr(1) + str.htail.skin.tskin(1).*Zs(1))*2*pdcylin.htail.dsw;
                WSHEARpC  = str.htail.web.tw(1).*tbs(1)*2*pdcylin.htail.dsw;
                WTORQUEpC = str.htail.skin.tskin(1).*Zs(1)*2*pdcylin.htail.dsw + WSHEARpC;
                % Carry-through structural masses (nct elements)
                str.htail.WBENDC    = WBENDpC*geo.htail.twc;
                str.htail.WSHEARC   = WSHEARpC*geo.htail.twc;
                str.htail.WTORSIONC = WTORQUEpC*geo.htail.twc;
                str.htail.WC        = str.htail.WBENDC + str.htail.WSHEARC;
            else
                str.htail.WC = 0;
            end

            
        otherwise
            error('Unknown value of kcon for htail.');
            
    end
end
