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
% DESVAR for TCR fuselage optimization
%
function [X, X0, XL, XU, CSTR_INEQ, IN_VALUE, CSTR_EQ, EQ_VALUE, CT_IN, CT_EQ] = desvar()
% define constraint design variables (equalities) 
CSTR_EQ = {}; EQ_VALUE = []; CT_EQ = []; CT_IN = [];
%
% define design variables 
%-----------------------------------------------------------------------------------------------------
% BEAM isotropic thickness
X={};
%
%-----------------------------------------------------------------------------------------------------
% HALFC is an array with half wing-box chord
% It is used to set the maximum web spacing and avoid the optimizer sets 0 webs within each section.
% At least 1 web in the semi-chord MUST exist in order to have a realistic section 
% For litting surfaces (wing,tail empennages, canard...), chord values are stores in beam_model.PBar.Section(X).data(4)
%
HALFC = [4.84747619047619
   4.17685714285714
   3.40938095238095
   3.21566666666667
   3.51242857142857
   3.20185714285714
   2.89128571428571
   2.58066666666667
   1.98385714285714
   1.78719047619048
   1.59052380952381
   1.39385714285714
   1.19723809523810
   1.00057142857143
   0.80390476190476
   0.60723809523810
   3.69342857142857
   2.19185714285714
   2.06952380952381
   1.94723809523810
   1.80900000000000
   1.66252380952381
   1.51600000000000
   1.36947619047619
   1.60200000000000
   1.35914285714286
   1.06704761904762
   0.96861904761905
   0.80590476190476
   0.57900000000000];
%
% CMIN is simply HALFC divided by 20, in order to set a maximum number of 20 webs in the semichord
%
CMIN =[0.10179700000000
   0.08771400000000
   0.07159700000000
   0.06752900000000
   0.07376100000000
   0.06723900000000
   0.06071700000000
   0.05419400000000
   0.04166100000000
   0.03753100000000
   0.03340100000000
   0.02927100000000
   0.02514200000000
   0.02101200000000
   0.01688200000000
   0.01275200000000
   0.07756200000000
   0.04602900000000
   0.04346000000000
   0.04089200000000
   0.03798900000000
   0.03491300000000
   0.03183600000000
   0.02875900000000
   0.03364200000000
   0.02854200000000
   0.02240800000000
   0.02034100000000
   0.01692400000000
   0.01215900000000];
%-----------------------------------------------------------------------------------------------------
% Define DESVAR for each component
%
% WING
%
windex = [12:27];
offset = 0;
cont = 0;
for k=1:length(windex)
  X{k} = ['beam_model.PBar.Section(',num2str(windex(k)),').data(1)']; % web thick
end
offset = length(X);
for k=1:length(windex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(windex(k)),').data(2)']; % skin thick
end
offset = length(X);
for k=1:length(windex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(windex(k)),').data(3)']; % web spacing
  cont=cont+1;
  CIND(cont) = k+offset;
end
%
% Vertical tail (variables are the same as wing)
%
vindex = [28:35];
offset = length(X);
for k=1:length(vindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(vindex(k)),').data(1)'];
end
offset = length(X);
for k=1:length(vindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(vindex(k)),').data(2)'];
end
offset = length(X);
for k=1:length(vindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(vindex(k)),').data(3)'];
  cont=cont+1;
  CIND(cont) = k+offset;
end
%
% Horizontal tail (variables are the same as wing)
%
hindex = [36:41];
offset = length(X);
for k=1:length(hindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(hindex(k)),').data(1)'];
end
offset = length(X);
for k=1:length(hindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(hindex(k)),').data(2)'];
end
offset = length(X);
for k=1:length(hindex)
  X{k+offset} = ['beam_model.PBar.Section(',num2str(hindex(k)),').data(3)'];
  cont=cont+1;
  CIND(cont) = k+offset;
end
CIND
%
% Fuselage
%
offset = length(X);
for k=1:11
  X{k+offset} = ['beam_model.PBar.Section(',num2str(k),').data(1)']; % skin thick
  cont=cont+1;
end
offset = length(X);
for k=1:11
  X{k+offset} = ['beam_model.PBar.Section(',num2str(k),').data(2)']; % frame area
  cont=cont+1;
end
offset = length(X);
for k=1:11
  X{k+offset} = ['beam_model.PBar.Section(',num2str(k),').data(3)']; % frame spacing
  cont=cont+1;
end
%-----------------------------------------------------------------------------------------------------
% Set DESVAR limits
% 
% INITIAL SOLUTION (nondimensional values required)
X0 = ones(1,length(X)); % UNITARY = use starting solution with coeff 1.0
%
% LOWER LIMIT (dimensional/real values required)
XL = 0.0005*ones(1,length(X));
% UPPER LIMIT (dimensional/real values required)
XU = 4*ones(1,length(X));
% overwrite values with correct fuselage frame spacing (20 cm)
XL(end-10:end) = 0.2*ones(1,11);
% set maximum allowed web spacing
XU(CIND) = HALFC; XL(CIND) = CMIN;
%
% define constraint variables
%
% Skin buckling 
CSTR_INEQ = {};
offset = length(CSTR_INEQ);
w=[1:63];
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{1}.Bar.CSM.SM_SBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{1}.Bar.CSM.SM_PBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{1}.Bar.CSM.SM_Norm(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{2}.Bar.CSM.SM_SBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{2}.Bar.CSM.SM_PBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{2}.Bar.CSM.SM_Norm(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{3}.Bar.CSM.SM_SBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{3}.Bar.CSM.SM_PBuck(',num2str(w(k)),')'];
end
offset = length(CSTR_INEQ);
for k=1:length(w)
  CSTR_INEQ{k+offset} = ['-beam_model.Optim.Res.Free_trim{3}.Bar.CSM.SM_Norm(',num2str(w(k)),')'];
end
% Aerodynamic derivatives constraints
l=length(CSTR_INEQ);
CSTR_INEQ{l+1} = '-beam_model.Optim.Res.Free_trim{1}.Aero.DStab_Der.Control.dcml_dDelta(3)/beam_model.Optim.Res.Free_trim{1}.Aero.RStab_Der.Control.dcml_dDelta(3)';
CSTR_INEQ{l+2} = '-beam_model.Optim.Res.Free_trim{1}.Aero.DStab_Der.Control.dcs_dDelta(5)/beam_model.Optim.Res.Free_trim{1}.Aero.RStab_Der.Control.dcs_dDelta(5)';
%
IN_VALUE = zeros(1,length(CSTR_INEQ));
% set ratio for constraints on aero derivatives (BE CAREFUL WITH SIGN!)
% remember: constraint inequalities is violated when leads to a POSITIVE value
IN_VALUE(end) = -0.75;
IN_VALUE(end-1) = -0.75;
%
