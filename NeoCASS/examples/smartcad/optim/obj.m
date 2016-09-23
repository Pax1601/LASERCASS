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

function [OBJ_VAL, SET] = obj(beam_model)

SET = 0;
OBJ_VAL = 0;
  try % MUST ALWAYS BE PRESENT in order to discover if the OBJ can be evaluated
    OBJ_VAL = beam_model.WB.MCG(1,1)/2.238603921671120e+05; % OBJ is nondimensional, dividing actual weight by the initial weight 
    SET = 1; % if the OBJ can be evaluated this variable is set to 1, meaning the OBJ has been evaluated!
  catch
    disp('Unable to evaluate OBJ.');
  end
end 
