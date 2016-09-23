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
%     080308      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% directly pick constrain equations
function [IN_CSTR, EQ_CSTR] = get_dcstr_value(beam_model, in_index, eq_index)
%
n_in = beam_model.Optim.Cstr.n_in;
n_eq = beam_model.Optim.Cstr.n_eq;
%
IN_CSTR = zeros(n_in,1);
EQ_CSTR = zeros(n_eq,1);
%
for k=1:length(in_index)
  command = ['IN_CSTR(',num2str(in_index(k)),')=', beam_model.Optim.Cstr.In.Name{in_index(k)},';'];
  eval(command);
end
%
for k=1:length(eq_index)
  command = ['EQ_CSTR(',num2str(eq_index(k)),')=', beam_model.Optim.Cstr.Eq.Name{eq_index(k)},';'];
  eval(command);
end
%
end
