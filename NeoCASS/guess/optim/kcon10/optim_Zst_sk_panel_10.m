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
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     251111      1.0     L. Cavagna       Creation
%                         
%*******************************************************************************
%
% function guess
%
%   DESCRIPTION: Run GUESS Module from structural sizing to stick model creation
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                C              real       wing box chord
%                H              real       wing box height
%                n_str          int        number of stringers
%                rib_pitch      real       rib pitch
%                E              real       Young modulus
%                Tx             real       horizontal force
%                Ty             real       vertical force
%                Mx             real       bending moment
%                Mz             real       torsional moment
%                CEVAL          array      index of constraints to be used
%                X0             array 3x1  initial solution for the optimization
%
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                SOL            array 3x1  solution
%                flag           integer    exit flag
%                fun            real       OBJ value
%                CUNS           array      constraints not satisfied
%
%    REFERENCES:
%
%*******************************************************************************

function [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_10(C, H, NSPAR, Acap, tw, n_strs, rib_pitch, E, smax,...
                                                        Tx, Ty, Mx, Mz, CEVAL, X0)

% INPUT
PRINT = 0;
%-------------------------------------------------------------------------------
% number of stringers
data.param.NS = n_strs;
% Young modulus
data.param.E = E;
% max tensile stress
data.param.smax = smax;
%-------------------------------------------------------------------------------
% LOADS
data.load.Tx = Tx;
data.load.Ty = Ty;
data.load.Mx = Mx;
data.load.Mz = Mz;
%-------------------------------------------------------------------------------
% GEOMETRY
data.geo.c = C;
data.geo.h = H;
data.geo.Acap = Acap;
data.geo.tw = tw;
%-------------------------------------------------------------------------------
% INTERNAL PARAMETERS for Z-stringer
%
CUNS = [];
data.param.Nspar = NSPAR;
data.param.KAs = 0.36;
data.param.ds  = 0.12;
data.param.tws_ts_ratio_max  = 1.4;
data.param.tws_ts_ratio_min  = 0.4;
data.param.Cindex = CEVAL;
%if (abs(Ty)>eps)
%
% VARIABLES
% ts, tw, As
%
% LIMITS

%  XL = 0.001 .* ones(3,1); XL(3) = 2.5e-5;
%  XU(1) = 1.1; XU(2) = 1.1; XU(3)= 1.1;
  XL = 0.001 .* ones(6,1); 
  XL(2) = 2.5e-5;
  XL(3) = H/240;
  XL(4) = 0.3;
  XL(5) = 0.38;
  XL(6) = min(H/2, rib_pitch/2);
  XL(6) = rib_pitch/2;
  XU(1) = 1.1; XU(2)= 1.1;
  XU(3) = H/5;
  XU(4) = 0.75;
  XU(5) = 0.66;
  XU(6) = rib_pitch*2;
  
  warning off
  TolCon = 0.001;
% OPTIONS =optimset('Algorithm','interior-point','MaxFunEvals',25000,'TolCon', 0.005, 'LargeScale', 'off', 'Display', 'off','tolfun',1e-4);
  OPTIONS =optimset('Algorithm','sqp','GradObj', 'on','Display', 'off','FunValCheck','on',...
                    'Diagnostics','off', 'MaxFunEvals',500000,'TolCon', TolCon, 'LargeScale', 'on', 'tolfun',1e-3);
  try
    [SOL, fun, flag, output] = fmincon(@(X)Zst_sk_mass_10(X,data), X0, ...
                               [],[],[],[], ... 
                               XL, XU, @(X)cstr_Zst_sk_panel_10(X,data), OPTIONS);
  %
    if (flag<0)
      if PRINT
        for (k=1:length(SOL))
          fprintf('\n\t\tVariable n. %d: %g.', k, SOL(k));
        end
      end
      [CIN, CEQ] = cstr_Zst_sk_panel_10(SOL, data);
      if PRINT
        for (k=1:length(CIN))
          fprintf('\n\t\tConstraint n. %d: %g.', k, CIN(k));
        end
      end
      cindex = find(CIN>TolCon);
      if (~isempty(cindex))
        if (isempty(CEVAL))
          CEVAL = [1:length(CIN)];
        end
        CUNS = CEVAL(cindex);
      else
        CUNS = [];
      end
    end
  catch
    flag = -1;
    fun = Inf;
    CUNS = [];
    SOL = X0;
  end
  warning on
