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
% Structural analysis modulus applied to vertical tail. Aerodynamic exposed
% vertical tail is estimated only.
%
% Called by:    AFaWWE.m
%
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080404      1.0     A. Da Ronch      Creation
%*******************************************************************************
function [str, optim] = Str_Vtail(niter, pdcylin, aircraft, geo, loads, str, optim)

%--------------------------------------------------------------------------------------------------
% Initialize structure
%--------------------------------------------------------------------------------------------------

str.vtail.WBEND     = [];    % weight of bending material                    [kg], vector
str.vtail.WSHEAR    = [];    % weight of shear material                      [kg], vector
str.vtail.WBOX      = [];    % ideal weight of 1 wing box structure          [kg], vector
str.vtail.WBENDC    = [];    % weight of bending material                    [kg], scalar
str.vtail.WSHEARC   = [];    % weight of shear material                      [kg], scalar
str.vtail.WTORSIONC = [];    % weight of torsion material                    [kg], scalar
str.vtail.WC        = [];    % weight of carrythrough-structure              [kg], scalar
%
% Size following the chosen structural concept
if pdcylin.vtail.kcon <=6

    str.vtail.dW        = [];    % optimum web spacing                           [m] , vector
    str.vtail.tCbar     = [];    % equivalent isotropic thickness of the covers  [m] , vector
    str.vtail.tWbar     = [];    % equivalent isotropic thickness of the webs    [m] , vector
    str.vtail.tgC       = [];    % cover gage thicknesses                        [m] , vector
    str.vtail.tgW       = [];    % web gage thicknesses                          [m] , vector
    str.vtail.tC        = [];    % maximum cover thickness                       [m] , vector
    str.vtail.tW        = [];    % maximum web thickness                         [m] , vector
    str.vtail.nrW       = [];    % number of webs

    % Optimal frame spacing
    str.vtail.dW = geo.vtail.tbs .*( (1-2*geo.vtail.ec)/((1-geo.vtail.ec)*sqrt(2*geo.vtail.epw)) .*...
        (abs(loads.vtail.M)./(geo.vtail.Zs.*geo.vtail.tbs.^2*pdcylin.vtail.esw)) .^...
        ((2*geo.vtail.ec-3)/(2*geo.vtail.ec)) *...
        geo.vtail.epc ^(3/(2*geo.vtail.ec)) ).^(2*geo.vtail.ec/(4*geo.vtail.ec-3));
%   avoid spikes at tip node
    str.vtail.dW(end) = str.vtail.dW(end-1);
    % Skin thickness
    Ind  = find(str.vtail.dW ~= 0);
    Ind1 = find(str.vtail.dW == 0);
    str.vtail.tCbar = zeros(geo.vtail.leny, 1);
    str.vtail.tCbar(Ind) = str.vtail.dW(Ind) .*( abs(loads.vtail.M(Ind))./(geo.vtail.Zs(Ind).*geo.vtail.tbs(Ind)*pdcylin.vtail.esw*...
        geo.vtail.epc.*str.vtail.dW(Ind)) ) .^(1/geo.vtail.ec);
    
    % Frame thickness
    str.vtail.tWbar = geo.vtail.tbs .*sqrt( (abs(loads.vtail.M)./(geo.vtail.Zs.*geo.vtail.tbs.^2*pdcylin.vtail.esw)) .^...
        (2-1/geo.vtail.ec) .*...
        (geo.vtail.epc.*str.vtail.dW./geo.vtail.tbs) .^(1/geo.vtail.ec).*(2/geo.vtail.epw) );
    
    % Gauge limits
    str.vtail.tgC = (pdcylin.vtail.tmgw/geo.vtail.Kgc).*ones(geo.vtail.leny, 1);
    str.vtail.tgW = (pdcylin.vtail.tmgw/geo.vtail.Kgw).*ones(geo.vtail.leny, 1);
    
    if ~isempty(Ind1)
        str.vtail.tWbar(Ind1) = 0.0;
        str.vtail.dW(Ind1)    = geo.vtail.Zs(Ind1)+eps;
    end
    str.vtail.tW  = max( [str.vtail.tWbar'; str.vtail.tgW'] )';
    str.vtail.nrW = ceil(geo.vtail.Zs./str.vtail.dW) + 1;
    
    % Sizing due to bending
    smax = pdcylin.vtail.fcsw;
    Jw   = str.vtail.nrW.*(1./12.*str.vtail.tW.*geo.vtail.tbs.^3)./(0.25 .* geo.vtail.Zs .* geo.vtail.tbs.^2);
    J1   = abs(loads.vtail.M)./(smax.*geo.vtail.tbs);
    str.vtail.tbC = J1 - Jw;
    str.vtail.tC  = max( [str.vtail.tCbar'; str.vtail.tgC'; str.vtail.tbC'] )';
    
    % Section areas (BAR number + 1) [m2]
    Aw = str.vtail.nrW.*geo.vtail.tbs.*str.vtail.tW;       % Web area
    Ac = 2.*(geo.vtail.Zs + geo.vtail.tbs).*str.vtail.tC;  % Cover area
    
    % Mass (BAR number) [kg]
    %     m2  = pdcylin.vtail.dsw.*geo.vtail.dy.*(Ac+Aw);
    m2 = pdcylin.vtail.dsw.*diff(geo.vtail.y_nodes_thick).*meancouple(Ac+Aw);
    
    %--------------------------------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE STRUCTURAL BOX
    %--------------------------------------------------------------------------------------------------
    
    % Bending material linear density (BAR number + 1) [kg/m]
    WBENDp = pdcylin.vtail.dsw.*geo.vtail.Zs .*geo.vtail.tbs *geo.vtail.ep .*...
        (abs(loads.vtail.M) ./(geo.vtail.Zs .*geo.vtail.tbs.^2 *pdcylin.vtail.esw)).^geo.vtail.e;
    
    % Bending material mass (BAR number) [kg]
    str.vtail.WBEND  = diff(geo.vtail.y_nodes_thick).*meancouple(WBENDp);
    
    % Shear material linear density (BAR number + 1)
    WSHEARp = pdcylin.vtail.dsw.*abs(loads.vtail.FS) /pdcylin.vtail.fcsw;
    
    % Shear material mass (BAR number) [kg]
    str.vtail.WSHEAR = diff(geo.vtail.y_nodes_thick).*meancouple(WSHEARp);
    
    % Load-carrying weight for vertical tail [kg]
    m1 = str.vtail.WBEND + str.vtail.WSHEAR;
    
    % Resize web thickness
    A1  = (WBENDp + WSHEARp)/pdcylin.vtail.dsw;             % ideal bending and shear areas
    A2c = 2.*(geo.vtail.Zs + geo.vtail.tbs).*str.vtail.tC;  % cover area
    A2w = str.vtail.nrW.*geo.vtail.tbs.*str.vtail.tW;       % web area
    tst = A1 > (A2c+A2w);
    %
    str.vtail.tW(tst) = (A1(tst) - A2c(tst))./(str.vtail.nrW(tst).*geo.vtail.tbs(tst));
    
    %     for i = 1:geo.vtail.leny
    %         if m1(i) > m2(i)
    %             str.vtail.tW(i) = (m1(i) - m2c(i) .*str.vtail.tC(i))./m2w(i);
    %         end
    %     end
    
    % Total structural mass (BAR number) [kg]
    str.vtail.WBOX = max([m1'; m2'])';
    
    %--------------------------------------------------------------------------
    % TOTAL IDEAL WEIGHT OF THE CARRYTHROUGH STRUCTURE
    %--------------------------------------------------------------------------
    % Vertical tail doesn't have a carry-through
    str.vtail.WBENDC    = 0;
    str.vtail.WSHEARC   = 0;
    str.vtail.WTORSIONC = 0;
    str.vtail.WC        = str.vtail.WBENDC + str.vtail.WSHEARC;
    
else
    
    switch pdcylin.vtail.kcon
        
        case {9}

          pdcylin.vtail = check_input_params(pdcylin.vtail, geo.vtail);

            fprintf('\n\tOptimization parameters:\n\t- stringer pitch: %f.\n\t- rib pitch: %f.',...
                         pdcylin.vtail.spitch,pdcylin.vtail.rpitch);
%
%            save('optim_wing.mat')
            if (niter==1)
              optim.vtail.skin = [];
              optim.vtail.web = [];
            end
%
            outps = 'bk_optim_Zst_sk_panel_v.mat';
            [str.vtail.skin, str.vtail.web] = run_optim_Zst_sk_panel_9(niter, optim.vtail.skin, optim.vtail.web, geo.vtail.Zs, geo.vtail.tbs, pdcylin.vtail.esw, pdcylin.vtail.msl, loads.vtail.N,...
                                              loads.vtail.FS, loads.vtail.M, loads.vtail.Mt, pdcylin.vtail.rpitch, pdcylin.vtail.spitch, outps);
%
            optim.vtail.skin.tskin(:,niter) =  str.vtail.skin.tskin;
            optim.vtail.skin.Astr(:,niter)  =  str.vtail.skin.Astr;     
            optim.vtail.web.tw(:,niter)     =  str.vtail.web.tw;       

            % Bending material linear density (BAR number + 1, only cantilever part)
            WBENDp = (str.vtail.skin.Astr.*str.vtail.skin.Nstr + str.vtail.skin.tskin.*geo.vtail.Zs)*2*pdcylin.vtail.dsw;
            
            % Bending material mass (BAR number, only cantilever part)
            str.vtail.WBEND = diff(geo.vtail.y_nodes_thick).*meancouple(WBENDp);
            
            % Shear material linear density (BAR number + 1, only cantilever part)
            WSHEARp = str.vtail.web.tw.*geo.vtail.tbs*2*pdcylin.vtail.dsw;
            
            % Shear material mass (BAR number, only cantilever part)
            str.vtail.WSHEAR  = diff(geo.vtail.y_nodes_thick).*meancouple(WSHEARp);
            
            % Torsion material linear density (BAR number + 1, only cantilever part)
            WTORQUEp = str.vtail.skin.tskin.*geo.vtail.Zs*2*pdcylin.vtail.dsw + WSHEARp;
            
            % Torsion material mass (BAR number, only cantilever part)
            str.vtail.WTORQUE  = diff(geo.vtail.y_nodes_thick).*meancouple(WTORQUEp);
            
            % Total structural mass (BAR number, only cantilever part)
            str.vtail.WBOX = str.vtail.WSHEAR + str.vtail.WBEND;
            
            % Vertical tail doesn't have a carry-through
            str.vtail.WBENDC    = 0;
            str.vtail.WSHEARC   = 0;
            str.vtail.WTORSIONC = 0;
            str.vtail.WC        = str.vtail.WBENDC + str.vtail.WSHEARC;

        otherwise
            error('Wrong value of kcon for vtail.');
            
    end
end