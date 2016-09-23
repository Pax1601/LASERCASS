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
% Structural analysis modulus applied to wing structural box.
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
function [str, optim] = Str_Wing(niter, pdcylin, aircraft, geo, loads, str, optim)

%--------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------
%
%
str.wing.WBEND     = [];    % weight of bending material                    [kg], vector
str.wing.WSHEAR    = [];    % weight of shear material                      [kg], vector
str.wing.WBOX      = [];    % ideal weight of 1 wing box structure          [kg], vector
str.wing.WBENDC    = [];    % weight of bending material                    [kg], scalar
str.wing.WSHEARC   = [];    % weight of shear material                      [kg], scalar
str.wing.WTORSIONC = [];    % weight of torsion material                    [kg], scalar
str.wing.WC        = [];    % weight of carrythrough-structure              [kg], scalar
%
% Check if loads and geo data are compatible and refer to CT values as well
nload = length(loads.wing.M);
ngeo  = length(geo.wing.tbs);
if ngeo > nload
  offset = ngeo-nload +1;
  Zs     = geo.wing.Zs(offset:end);
  tbs    = geo.wing.tbs(offset:end);
else
  Zs  = geo.wing.Zs;
  tbs = geo.wing.tbs;
end
index = geo.wing.index(2:end)-(geo.wing.index(2)-1);
Nsec = length(loads.wing.M);
% Size following the chosen structural concept
if pdcylin.wing.kcon <= 6
%
    str.wing.dW        = [];    % optimum web spacing                           [m] , vector
    str.wing.tCbar     = [];    % equivalent isotropic thickness of the covers  [m] , vector
    str.wing.tWbar     = [];    % equivalent isotropic thickness of the webs    [m] , vector
    str.wing.tgC       = [];    % cover gage thicknesses                        [m] , vector
    str.wing.tgW       = [];    % web gage thicknesses                          [m] , vector
    str.wing.tC        = [];    % maximum cover thickness                       [m] , vector
    str.wing.tW        = [];    % maximum web thickness                         [m] , vector
    str.wing.nrW       = [];    % number of webs
    % Carrythrough element number
    nct = pdcylin.stick.nwing_carryth;
 
    % Optimal frame spacing
    str.wing.dW = tbs .*( (1-2*geo.wing.ec)/((1-geo.wing.ec)*sqrt(2*geo.wing.epw)) .*...
        (loads.wing.M./(Zs.*(tbs*0.5).^2*pdcylin.wing.esw)) .^...
        ((2*geo.wing.ec-3)/(2*geo.wing.ec)) *...
        geo.wing.epc ^(3/(2*geo.wing.ec)) ).^(2*geo.wing.ec/(4*geo.wing.ec-3));
%   avoid spikes at tip node
    str.wing.dW(end) = str.wing.dW(end-1);
%
    Ind = find(str.wing.dW ~= 0);
    str.wing.tCbar = zeros(Nsec, 1);
    
    % Cover thickness
    str.wing.tCbar(Ind) = str.wing.dW(Ind) .*( loads.wing.M(Ind)./(Zs(Ind).*tbs(Ind)*0.5*pdcylin.wing.esw*...
        geo.wing.epc.*str.wing.dW(Ind)) ) .^(1/geo.wing.ec);
    
    % Web thickness
    str.wing.tWbar = tbs .*sqrt( (loads.wing.M./(Zs.*(tbs*0.5).^2*pdcylin.wing.esw)) .^...
        (2-1/geo.wing.ec) .*...
        (geo.wing.epc.*str.wing.dW./(tbs*0.5)) .^(1/geo.wing.ec).*(2/geo.wing.epw) );
    
    % Gauge thicknesses
    str.wing.tgC = (pdcylin.wing.tmgw/geo.wing.Kgc).*ones(Nsec, 1);
    str.wing.tgW = (pdcylin.wing.tmgw/geo.wing.Kgw).*ones(Nsec, 1);
    
    % Maximum web thickness
    str.wing.tW  = max([str.wing.tWbar'; str.wing.tgW'])'; %str.wing.tWbar;%
    
    % Number of webs
    str.wing.nrW = ceil(Zs(Ind)./str.wing.dW(Ind)) + 1;
    
    % Sizing due to bending
    Jd   = 2*1/12.*tbs.^3 + 2.*Zs.*(tbs./2).^2 ;%+ str.wing.nrW.*1./12.*str.wing.tW.*tbs.^3;  2*1/12.*tbs.^3
    fos  = 1.0;
    smax = pdcylin.wing.fcsw;
    str.wing.tbC = fos.*loads.wing.M./Jd .*tbs./2 ./smax;
    % str.wing.tbC = loads.wing.M./(0.8*tbs*smax.*Zs);
    
    % Maximum cover thickness
    str.wing.tC  = max([str.wing.tCbar'; str.wing.tgC'])';
    % str.wing.tC = max([str.wing.tbC'; str.wing.tgC'])';
    % str.wing.tC = max([str.wing.tCbar'; str.wing.tgC'; str.wing.tbC'])';
    
    % Section areas (BAR number + 1) [m2]
    Aw = str.wing.nrW.*tbs.*str.wing.tW;       % Web area
    Ac = 2.*(Zs + tbs).*str.wing.tC;  % Cover area
    
    % Mass (BAR number) [kg]
    m2 = pdcylin.wing.dsw.*diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(Ac+Aw);
    
    %--------------------------------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE STRUCTURAL BOX
    %--------------------------------------------------------------------------------------------------
    
    % Bending material linear density (BAR number + 1) [kg/m]
    WBENDp = pdcylin.wing.dsw *Zs .*tbs *geo.wing.ep .*...
        (abs(loads.wing.M)./(Zs .*tbs.^2 *pdcylin.wing.esw)).^geo.wing.e;
    WBENDp2 = pdcylin.wing.dsw*abs(loads.wing.M)./(0.8*tbs*smax);
    WBENDp3 = pdcylin.wing.dsw*str.wing.tbC.*(Zs)*2;  
    
    % Bending material mass (BAR number) [kg]
    str.wing.WBEND  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
    str.wing.WBEND2 = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WBENDp2);
    str.wing.WBEND3 = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WBENDp3);
    
    % Shear material linear density (BAR number + 1) [kg/m]
    WSHEARp = pdcylin.wing.dsw *abs(loads.wing.FS) /pdcylin.wing.fcsw;
     
    % Shear material mass (BAR number) [kg]
    str.wing.WSHEAR  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
    
    % Load-carrying weight for semi-wing    
    m1 = str.wing.WBEND + str.wing.WSHEAR;
    
    % Resize web thickness
    A1  = (WBENDp + WSHEARp)/pdcylin.wing.dsw;           % ideal bending and shear areas
    A2c = 2.*(Zs + tbs).*str.wing.tC;  % cover area
    A2w = str.wing.nrW.*tbs.*str.wing.tW;       % web area
    tst = A1 > (A2c+A2w);
    %
    str.wing.tW(tst) = (A1(tst) - A2c(tst))./(str.wing.nrW(tst).*tbs(tst));
    
    % Total structural mass (BAR number) [kg]
    str.wing.WBOX = max([m1'; m2'])';
    
    %--------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE CARRYTHROUGH STRUCTURE
    %--------------------------------------------------------------------------    
    Torque_bend = 0;
    Torque_shea = 0;
    for i = 1:length(index)-1
        Torque_bend = Torque_bend + (loads.wing.M(index(i)) - loads.wing.M(index(i+1)))*...
            cos(geo.wing.CAERO1.sweepC2(i)*pi/180);
        Torque_shea = Torque_shea + (loads.wing.M(index(i)) - loads.wing.M(index(i+1)))*...
            sin(geo.wing.CAERO1.sweepC2(i)*pi/180);
    end
    
    % Carry-through bending mass [kg]
    str.wing.WBENDC    = pdcylin.wing.dsw *geo.wing.ep *geo.wing.CSR *geo.wing.tcs *(geo.fus.R*2) *...
        ( abs(Torque_bend) /((geo.wing.tcs*0.5)^2 *geo.wing.CSR *pdcylin.wing.esw) )^geo.wing.e;
    
    % Carry-through shear mass [kg]
    str.wing.WSHEARC   = pdcylin.wing.dsw *abs(loads.wing.FS(1)) /pdcylin.wing.fcsw *(geo.fus.R*2);
    
    % Carry-through torsion mass [kg]
    str.wing.WTORSIONC = pdcylin.wing.dsw * abs(Torque_shea) *((geo.wing.tcs*0.5) + geo.wing.CSR) *(geo.fus.R*2) /...
        ( geo.wing.tcs *geo.wing.CSR *pdcylin.wing.fcsw );
    
    % Total carry-through mass [kg]
    str.wing.WC        = str.wing.WBENDC + str.wing.WSHEARC + str.wing.WTORSIONC;
    
else
    
    switch pdcylin.wing.kcon
        
        case {9}

          pdcylin.wing = check_input_params(pdcylin.wing, geo.wing);

            fprintf('\n\tOptimization parameters:\n\t- stringer pitch: %f.\n\t- rib pitch: %f.',...
                         pdcylin.wing.spitch,pdcylin.wing.rpitch);
%
%            save('optim_wing.mat')
            if (niter==1)
              optim.wing.skin = [];
              optim.wing.web = [];
            end
%
            outps = 'bk_optim_Zst_sk_panel_w.mat';
            [str.wing.skin, str.wing.web] = run_optim_Zst_sk_panel_9(niter, optim.wing.skin, optim.wing.web, Zs, tbs, pdcylin.wing.esw, pdcylin.wing.msl, loads.wing.N,...
                                              loads.wing.FS, loads.wing.M, loads.wing.Mt, pdcylin.wing.rpitch, pdcylin.wing.spitch, outps);
%
            optim.wing.skin.tskin(:,niter) =  str.wing.skin.tskin;
            optim.wing.skin.Astr(:,niter)  =  str.wing.skin.Astr;     
            optim.wing.web.tw(:,niter)     =  str.wing.web.tw;       
            % Carrythrough element number
            nct = pdcylin.stick.nwing_carryth;

            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.wing.skin.Astr.*str.wing.skin.Nstr + str.wing.skin.tskin.*Zs)*2*pdcylin.wing.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.wing.WBEND = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.wing.web.tw.*tbs*2*pdcylin.wing.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.wing.WSHEAR  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.wing.skin.tskin.*Zs*2*pdcylin.wing.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.wing.WTORQUE  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.wing.WBOX = str.wing.WSHEAR + str.wing.WBEND;
            
            % First section linear densities (note that Zs(1) is the chord
            % along carry-thourgh axis and not projected along cantilever
            % part axis, used instead of Zs(1))
            D = 2*geo.fus.R; 
            WBENDpC   = (str.wing.skin.Astr(1).*str.wing.skin.Nstr(1) + str.wing.skin.tskin(1).*Zs(1))*2*pdcylin.wing.dsw;
            WSHEARpC  = str.wing.web.tw(1).*tbs(1)*2*pdcylin.wing.dsw;
            WTORQUEpC = str.wing.skin.tskin(1).*Zs(1)*2*pdcylin.wing.dsw + WSHEARpC;
            
            str.wing.WBENDC    = WBENDpC*D;
            str.wing.WSHEARC   = WSHEARpC*D;
            str.wing.WTORSIONC = WTORQUEpC*D;
            str.wing.WC        = str.wing.WBENDC + str.wing.WSHEARC;


        case {10}

          pdcylin.wing = check_input_params(pdcylin.wing, geo.wing);
%
            fprintf('\n\tOptimization parameters:\n\t- stringer pitch: %f.\n\t- rib pitch: %f.\n\t- spars: %d.',...
                         pdcylin.wing.spitch,pdcylin.wing.rpitch,pdcylin.wing.nspar);
%
            if (niter==1)
              optim.wing.web = [];
              optim.wing.skin = [];
            end
%
            outpc = 'bk_optim_Tcap_spar_w.mat'
            str.wing.web  = run_optim_Tcap_spar_10(niter, optim.wing.web, Zs, tbs, pdcylin.wing.nspar, pdcylin.wing.esw, pdcylin.wing.msl, loads.wing.FS, loads.wing.M, zeros(Nsec,1), zeros(Nsec,1), zeros(Nsec,1), outpc);
            outps = 'bk_optim_Zst_sk_panel_w.mat'
            str.wing.skin = run_optim_Zst_sk_panel_10(niter, optim.wing.skin, Zs, tbs, pdcylin.wing.nspar, pdcylin.wing.esw, pdcylin.wing.msl, loads.wing.FS, loads.wing.M, loads.wing.Mt, pdcylin.wing.rpitch, pdcylin.wing.spitch, str.wing.web.Acap, str.wing.web.tw, outps);
            %str.wing.web  = run_optim_Tcap_spar_10(niter, optim.wing.web, Zs, tbs, pdcylin.wing.nspar, pdcylin.wing.esw, pdcylin.wing.msl, loads.wing.FS, loads.wing.M, str.wing.skin.Nstr, str.wing.skin.Astr, str.wing.skin.tskin, outpc);
%
            optim.wing.web.Acap(:,niter)   =  str.wing.web.Acap;     
            optim.wing.web.tw(:,niter)     =  str.wing.web.tw;       
            optim.wing.web.B1_cap(:,niter) =  str.wing.web.B1_cap;   
            optim.wing.web.T1_cap(:,niter) =  str.wing.web.T1_cap;   
            optim.wing.web.B2_cap(:,niter) =  str.wing.web.B2_cap;   
            optim.wing.web.T2_cap(:,niter) =  str.wing.web.T2_cap;   
            optim.wing.web.Bu_stf(:,niter) =  str.wing.web.Bu_stf;   
            optim.wing.web.tu_stf(:,niter) =  str.wing.web.tu_stf;   
            optim.wing.web.du_stf(:,niter) =  str.wing.web.du_stf;   
            optim.wing.skin.tskin(:,niter) =  str.wing.skin.tskin;
            optim.wing.skin.Astr(:,niter)  =  str.wing.skin.Astr;     
            optim.wing.skin.trib(:,niter)  =  str.wing.skin.trib;
            optim.wing.skin.D1_rib(:,niter)=  str.wing.skin.D1_rib;
            optim.wing.skin.D2_rib(:,niter)=  str.wing.skin.D2_rib;
            optim.wing.skin.RP(:,niter)    =  str.wing.skin.RP;
            % Carrythrough element number
            nct = pdcylin.stick.nwing_carryth;

            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.wing.skin.Astr.*str.wing.skin.Nstr + str.wing.skin.tskin.*Zs + pdcylin.wing.nspar.*str.wing.web.Acap)*2*pdcylin.wing.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.wing.WBEND = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.wing.web.tw.*tbs*2*pdcylin.wing.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.wing.WSHEAR  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.wing.skin.tskin.*Zs*2*pdcylin.wing.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.wing.WTORQUE  = diff(geo.wing.y_nodes_thick(nct+1:end)).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.wing.WBOX = str.wing.WSHEAR + str.wing.WBEND;
            
            % First section linear densities (note that Zs(1) is the chord
            % along carry-thourgh axis and not projected along cantilever
            % part axis, used instead of Zs(1))
            D = 2*geo.fus.R; 
            WBENDpC   = (str.wing.skin.Astr(1).*str.wing.skin.Nstr(1) + str.wing.skin.tskin(1).*Zs(1) + pdcylin.wing.nspar*str.wing.web.Acap(1))*2*pdcylin.wing.dsw;
            WSHEARpC  = str.wing.web.tw(1).*tbs(1)*pdcylin.wing.nspar*pdcylin.wing.dsw;
            WTORQUEpC = str.wing.skin.tskin(1).*Zs(1)*2*pdcylin.wing.dsw + WSHEARpC;
            
            str.wing.WBENDC    = WBENDpC*D;
            str.wing.WSHEARC   = WSHEARpC*D;
            str.wing.WTORSIONC = WTORQUEpC*D;
            str.wing.WC        = str.wing.WBENDC + str.wing.WSHEARC;

        otherwise
          error('Wrong value of kcon for wing.');

    end
            
end