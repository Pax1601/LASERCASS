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
% Structural analysis modulus applied to canard structural box.
%
% Weight estimation is computed as a:
%   - vector station-by-station for the canard structural box;
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
function [str, optim] = Str_Canr(niter, pdcylin, aircraft, geo, loads, str, optim)

%----------------------------------------------------------------------------------------
% Initialize structure: STR.canard
%----------------------------------------------------------------------------------------

str.canard.WBEND     = [];    % weight of bending material                    [kg], vector
str.canard.WSHEAR    = [];    % weight of shear material                      [kg], vector
str.canard.WBOX      = [];    % ideal weight of 1 wing box structure          [kg], vector
str.canard.WBENDC    = [];    % weight of bending material                    [kg], scalar
str.canard.WSHEARC   = [];    % weight of shear material                      [kg], scalar
str.canard.WTORSIONC = [];    % weight of torsion material                    [kg], scalar
str.canard.WC = [];           % weight of carrythrough-structure              [kg], scalar%
% Check if loads and geo data are compatible and refer to CT values as well
% 
nload = length(loads.canard.M);
ngeo  = length(geo.canard.tbs);
if ngeo > nload
  offset = ngeo-nload +1;
  Zs     = geo.canard.Zs(offset:end);
  tbs    = geo.canard.tbs(offset:end);
else
  Zs  = geo.canard.Zs;
  tbs = geo.canard.tbs;
end
index = geo.canard.index(2:end)-(geo.canard.index(2)-1);
Nsec = length(loads.canard.M);
nct = pdcylin.stick.ncanard_carryth;
% Size following the chosen structural concept
if pdcylin.canard.kcon <= 6

    str.canard.dW        = [];    % optimum web spacing                           [m] , vector
    str.canard.tCbar     = [];    % equivalent isotropic thickness of the covers  [m] , vector
    str.canard.tWbar     = [];    % equivalent isotropic thickness of the webs    [m] , vector
    str.canard.tgC       = [];    % cover gage thicknesses                        [m] , vector
    str.canard.tgW       = [];    % web gage thicknesses                          [m] , vector
    str.canard.tC        = [];    % maximum cover thickness                       [m] , vector
    str.canard.tW        = [];    % maximum web thickness                         [m] , vector
    str.canard.nrW       = [];    % number of webs
    % Optimal frame spacing
    str.canard.dW = tbs .*( (1-2*geo.canard.ec)/((1-geo.canard.ec)*sqrt(2*geo.canard.epw)) .*...
        (loads.canard.M./(Zs.*(tbs*0.5).^2*pdcylin.canard.esw)) .^...
        ((2*geo.canard.ec-3)/(2*geo.canard.ec)) *...
        geo.canard.epc ^(3/(2*geo.canard.ec)) ).^(2*geo.canard.ec/(4*geo.canard.ec-3));
%   avoid spikes at tip node
    str.canard.dW(end) = str.canard.dW(end-1);
    Ind = find(str.canard.dW ~= 0);
    str.canard.tCbar = zeros(Nsec, 1);
    
    % Cover thickness
    str.canard.tCbar(Ind) = str.canard.dW(Ind) .*( loads.canard.M(Ind)./(Zs(Ind).*tbs(Ind)*0.5*pdcylin.canard.esw*...
        geo.canard.epc.*str.canard.dW(Ind)) ) .^(1/geo.canard.ec);
    
    % Web thickness
    str.canard.tWbar = tbs .*sqrt( (loads.canard.M./(Zs.*(tbs*0.5).^2*pdcylin.canard.esw)) .^...
        (2-1/geo.canard.ec) .*...
        (geo.canard.epc.*str.canard.dW./(tbs*0.5)) .^(1/geo.canard.ec).*(2/geo.canard.epw) );
    
    % Gauge thicknesses
    str.canard.tgC = (pdcylin.canard.tmgw/geo.canard.Kgc).*ones(Nsec, 1);
    str.canard.tgW = (pdcylin.canard.tmgw/geo.canard.Kgw).*ones(Nsec, 1);
    
    % Maximum web thickness
    str.canard.tW  = max([str.canard.tWbar'; str.canard.tgW'])'; %str.canard.tWbar;%
    
    % Number of webs
    str.canard.nrW = ceil(Zs(Ind)./str.canard.dW(Ind)) + 1;
    
    % Sizing due to bending
    Jd   = 2*1/12.*tbs.^3 + 2.*Zs.*(tbs./2).^2 ;%+ str.canard.nrW.*1./12.*str.canard.tW.*tbs.^3;  2*1/12.*tbs.^3
    fos  = 1.0;
    smax = pdcylin.canard.fcsw;
    str.canard.tbC = fos.*loads.canard.M./Jd .*tbs./2 ./smax;
    % str.canard.tbC = loads.canard.M./(0.8*tbs*smax.*Zs);
    
    % Maximum cover thickness
    str.canard.tC  = max([str.canard.tCbar'; str.canard.tgC'])';
    % str.canard.tC = max([str.canard.tbC'; str.canard.tgC'])';
    % str.canard.tC = max([str.canard.tCbar'; str.canard.tgC'; str.canard.tbC'])';
    
    % Section areas (BAR number + 1) [m2]
    Aw = str.canard.nrW.*tbs.*str.canard.tW;       % Web area
    Ac = 2.*(Zs + tbs).*str.canard.tC;  % Cover area
    
    % Mass (BAR number) [kg]
    m2 = pdcylin.canard.dsw.*diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(Ac+Aw);
    
    %--------------------------------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE STRUCTURAL BOX
    %--------------------------------------------------------------------------------------------------
    
    % Bending material linear density (BAR number + 1) [kg/m]
    WBENDp = pdcylin.canard.dsw *Zs .*tbs *geo.canard.ep .*...
        (abs(loads.canard.M)./(Zs .*tbs.^2 *pdcylin.canard.esw)).^geo.canard.e;
    
    % Bending material mass (BAR number) [kg]
    str.canard.WBEND  = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
   
    % Shear material linear density (BAR number + 1) [kg/m]
    WSHEARp = pdcylin.canard.dsw *abs(loads.canard.FS) /pdcylin.canard.fcsw;
     
    % Shear material mass (BAR number) [kg]
    str.canard.WSHEAR  = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
    
    % Load-carrying weight for semi-canard    
    m1 = str.canard.WBEND + str.canard.WSHEAR;
    
    % Resize web thickness
    A1  = (WBENDp + WSHEARp)/pdcylin.canard.dsw;               % ideal bending and shear areas
    A2c = 2.*(Zs + tbs).*str.canard.tC;  % cover area
    A2w = str.canard.nrW.*tbs.*str.canard.tW;       % web area
    tst = A1 > (A2c+A2w);
    %
    str.canard.tW(tst) = (A1(tst) - A2c(tst))./(str.canard.nrW(tst).*tbs(tst));
    
    % Total structural mass (BAR number) [kg]
    str.canard.WBOX = max([m1'; m2'])';
    
    %--------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE CARRYTHROUGH STRUCTURE
    %--------------------------------------------------------------------------    
    Torque_bend = 0;
    Torque_shea = 0;
    for i = 1:length(index)-1
        Torque_bend = Torque_bend + (loads.canard.M(index(i)) - loads.canard.M(index(i+1)))*...
            cos(geo.canard.CAERO1.sweepC2(i)*pi/180);
        Torque_shea = Torque_shea + (loads.canard.M(index(i)) - loads.canard.M(index(i+1)))*...
            sin(geo.canard.CAERO1.sweepC2(i)*pi/180);
    end
    
    % Carry-through bending mass [kg]
    str.canard.WBENDC    = pdcylin.canard.dsw *geo.canard.ep *geo.canard.CSR *geo.canard.tcs *(geo.fus.R*2) *...
        ( abs(Torque_bend) /((geo.canard.tcs*0.5)^2 *geo.canard.CSR *pdcylin.canard.esw) )^geo.canard.e;
    
    % Carry-through shear mass [kg]
    str.canard.WSHEARC   = pdcylin.canard.dsw *abs(loads.canard.FS(1)) /pdcylin.canard.fcsw *(geo.fus.R*2);
    
    % Carry-through torsion mass [kg]
    str.canard.WTORSIONC = pdcylin.canard.dsw * abs(Torque_shea) *((geo.canard.tcs*0.5) + geo.canard.CSR) *(geo.fus.R*2) /...
        ( geo.canard.tcs *geo.canard.CSR *pdcylin.canard.fcsw );
    
    % Total carry-through mass [kg]
    str.canard.WC        = str.canard.WBENDC + str.canard.WSHEARC + str.canard.WTORSIONC;

else
    switch pdcylin.canard.kcon

        case {8}
          Lstrut = geo.canard.bS; % find strut length for global instability constraints
          %  save('optim_canard.mat')
          if (niter==1)
            optim.canard.skin = [];
          end
          outps = 'bk_optim_strut_c.mat';
%
          tbs = tbs./2;
          [str.canard.skin] = run_optim_strut_8(niter, optim.canard.skin, tbs, pdcylin.canard.esw, pdcylin.canard.msl, loads.canard.N, loads.canard.FS, loads.canard.M, loads.canard.Mt, Lstrut, outps);
%
          optim.canard.skin.tskin(:,niter) =  str.canard.skin.tskin;
          % Bending material linear density (BAR number + 1, only cantilever part)
          WBENDp = (pi *pdcylin.canard.dsw) .* (tbs.^2 - (tbs - str.canard.skin.tskin).^2);
          % Bending material mass (BAR number, only cantilever part)
          str.canard.WBEND = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
          % Shear material linear density (BAR number + 1, only cantilever part)
          WSHEARp = WBENDp;
          % Shear material mass (BAR number, only cantilever part)
          str.canard.WSHEAR  = str.canard.WBEND;
          % Torsion material linear density (BAR number + 1, only cantilever part)
          WTORQUEp = WBENDp;
          % Torsion material mass (BAR number, only cantilever part)
          str.canard.WTORQUE  = str.canard.WBEND;
          % Total structural mass (BAR number, only cantilever part)
          str.canard.WBOX = str.canard.WBEND;

          if geo.canard.twc
              % First section linear densities (note that Zs(1) is the chord
              % along carry-thourgh axis and not projected along cantilever
              % part axis, used instead of Zs(1))
              WBENDpC   = WBENDp(1);
              WSHEARpC  = WSHEARp(1);
              WTORQUEpC = WTORQUEp(1);
              % Carry-through structural masses (nct elements)
              str.canard.WBENDC    = WBENDpC*geo.canard.twc;
              str.canard.WSHEARC   = WSHEARpC*geo.canard.twc;
              str.canard.WTORSIONC = WTORQUEpC*geo.canard.twc;
              str.canard.WC        = str.canard.WBENDC;
          else
              str.canard.WC = 0;
          end
%
        case {9}

          pdcylin.canard = check_input_params(pdcylin.canard, geo.canard);

            fprintf('\n\tOptimization parameters:\n\t- stringer pitch: %f.\n\t- rib pitch: %f.',...
                         pdcylin.canard.spitch,pdcylin.canard.rpitch);
%
%            save('optim_wing.mat')
            if (niter==1)
              optim.canard.skin = [];
              optim.canard.web = [];
            end
%
            outps = 'bk_optim_Zst_sk_panel_c.mat';
            [str.canard.skin, str.canard.web] = run_optim_Zst_sk_panel_9(niter, optim.canard.skin, optim.canard.web, Zs, tbs, pdcylin.canard.esw, pdcylin.canard.msl, loads.canard.N,...
                                              loads.canard.FS, loads.canard.M, loads.canard.Mt, pdcylin.canard.rpitch, pdcylin.canard.spitch, outps);
%
            optim.canard.skin.tskin(:,niter) =  str.canard.skin.tskin;
            optim.canard.skin.Astr(:,niter)  =  str.canard.skin.Astr;     
            optim.canard.web.tw(:,niter)     =  str.canard.web.tw;       
            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.canard.skin.Astr.*str.canard.skin.Nstr + str.canard.skin.tskin.*Zs)*2*pdcylin.canard.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.canard.WBEND = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.canard.web.tw.*tbs*2*pdcylin.canard.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.canard.WSHEAR  = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.canard.skin.tskin.*Zs*2*pdcylin.canard.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.canard.WTORQUE  = diff(geo.canard.y_nodes_thick(nct+1:end)).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.canard.WBOX = str.canard.WSHEAR + str.canard.WBEND;
            
            if geo.canard.twc
                % First section linear densities (note that Zs(1) is the chord
                % along carry-thourgh axis and not projected along cantilever
                % part axis, used instead of Zs(1))
                WBENDpC   = (str.canard.skin.Astr(1).*str.canard.skin.Nstr(1) + str.canard.skin.tskin(1).*Zs(1))*2*pdcylin.canard.dsw;
                WSHEARpC  = str.canard.web.tw(1).*tbs(1)*2*pdcylin.canard.dsw;
                WTORQUEpC = str.canard.skin.tskin(1).*Zs(1)*2*pdcylin.canard.dsw + WSHEARpC;
                % Carry-through structural masses (nct elements)
                str.canard.WBENDC    = WBENDpC*geo.canard.twc;
                str.canard.WSHEARC   = WSHEARpC*geo.canard.twc;
                str.canard.WTORSIONC = WTORQUEpC*geo.canard.twc;
                str.canard.WC        = str.canard.WBENDC + str.canard.WSHEARC;
            else
                str.canard.WC = 0;
            end

        otherwise
            error('Wrong value of kcon for canard.');
   
    end
end
    
