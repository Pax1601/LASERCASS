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

function [SOL, flag, fun, CUNS] = optim_Tcap_spar_10(C, H, Nstr, Astr, tskin, E, smax, Ty, Mx, CEVAL, X0)

% INPUT
%-------------------------------------------------------------------------------
%
PRINT = 0;
X0 = X0./0.0254;
%
E =10.7e+6;
smax = 74000;
sult = 82000;
Astr = Astr /0.0254/0.0254;
Mx = Mx *8.8;
Ty = Ty/0.45/9.81;
H = H/0.0254;
C = C/0.0254;
tskin = tskin /0.0254;
%
% Initialize value
%
% Young modulus
data.param.E = E;
% max tensile stress
data.param.smax = smax;
data.param.sult = sult;
%-------------------------------------------------------------------------------
% LOADS
data.load.Ty = Ty;
data.load.Mx = Mx;
%-------------------------------------------------------------------------------
% GEOMETRY
data.geo.h = H;
data.geo.C = C;
data.geo.Astr = Astr;
data.geo.Nstr = Nstr;
data.geo.tskin = tskin;
%-------------------------------------------------------------------------------
% INTERNAL PARAMETERS for Z-stringer
%
CUNS = [];
data.param.Cindex = CEVAL;

  XU(1) = H/35;
  XU(2) = 20;
  XU(3) = 1;
  XU(4) = 20;
  XU(5) = 1;
  XU(6) = 20;
  XU(7) = 1;
  XU(8) = H;

  XL(1) = H/1500;
  XL(2) = 0.5;
  XL(3) = 0.04;
  XL(4) = 0.5;
  XL(5) = 0.04;
  XL(6) = 0.5;
  XL(7) = 0.04;
  XL(8) = 0.2*H;

  warning off
  TolCon = 0.001;
%notify-detailed
  OPTIONS =optimset('Algorithm','interior-point','Diagnostics','off','GradObj', 'on','Display', 'off',...
                    'MaxFunEvals',500000,'TolCon', TolCon, 'LargeScale', 'on', 'tolfun',1e-2, 'FunValCheck','off');
  try
    [SOL, fun, flag, output] = fmincon(@(X)spar_mass_10(X,data), X0, ...
                               [],[],[],[], ... 
                               XL, XU, @(X)cstr_Tcap_spar_10(X,data), OPTIONS);
    if (flag<0)
      if PRINT
        for (k=1:length(SOL))
          fprintf('\n\t\tVariable n. %d: %g.', k, SOL(k).*0.0254);
        end
      end
      [CIN, CEQ] = cstr_Tcap_spar_10(SOL, data);
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
    SOL = SOL.*0.0254;
  catch
    flag = -1;
    fun = Inf;
    CUNS = [];
    SOL = X0.*0.0254;
  end
warning on
%
