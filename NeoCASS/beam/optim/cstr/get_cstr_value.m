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
% look for missing constraint equations
function [IN_CSTR, EQ_CSTR, SET_IN, SET_EQ] = get_cstr_value(beam_model, IN_CSTR, EQ_CSTR, in_index, eq_index)
%
n_in = beam_model.Optim.Cstr.n_in;
n_eq = beam_model.Optim.Cstr.n_eq;
fid = beam_model.Param.FID;
fprintf(fid, '\nConstraint evaluation:');
EVALUATED = false;
%
SET_IN = [];
SET_EQ = [];
%
for k=1:length(in_index)
  try
    command = ['IN_CSTR(',num2str(in_index(k)),')=', beam_model.Optim.Cstr.In.Name{in_index(k)},';'];
    eval(command);
    SET_IN = [SET_IN, in_index(k)];
    fprintf(fid, '\n - Constraint n. %d evaluated: %f.', SET_IN(end), IN_CSTR(SET_IN(end)));    
    EVALUATED = true;
  end
end
%
for k=1:length(eq_index)
  try
    command = ['EQ_CSTR(',num2str(eq_index(k)),')=', beam_model.Optim.Cstr.Eq.Name{eq_index(k)},';'];
    eval(command);
    SET_EQ = [SET_EQ, eq_index(k)];
    fprintf(fid, '\n - Constraint n. %d evaluated: %f.', SET_EQ(end), IN_CSTR(SET_EQ(end)));    
    EVALUATED = true;
  end
end
%
if (EVALUATED==false)
  fprintf(fid, ' none.');
end

end
