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

function nodal_aero_loads = aero_loads(beam_model)



nodal_aero_loads=[];
nodal_aero_loads.fus=[];
nodal_aero_loads.wing=[];
nodal_aero_loads.vtail=[];
nodal_aero_loads.htail=[];
nodal_aero_loads.wing2=[];

n_trim=length(beam_model.Aero.Trim.Select);

for i=1:n_trim
    solve_free_lin_trim_guess(i)
%     guess_model.nodal_loads=[guess_model.nodal_loads beam_model.Res.Guess.Aero_Nforce];
    nodal_aero_loads.fus=[nodal_aero_loads.fus beam_model.Res.Guess.Aero_Nforce(1:12,:)];
    nodal_aero_loads.wing=[nodal_aero_loads.wing beam_model.Res.Guess.Aero_Nforce(13:133,:)];
    nodal_aero_loads.vtail=[nodal_aero_loads.vtail beam_model.Res.Guess.Aero_Nforce(255:285,:)];
    nodal_aero_loads.htail=[nodal_aero_loads.htail beam_model.Res.Guess.Aero_Nforce(286:346,:)];
    nodal_aero_loads.wing2=[nodal_aero_loads.wing2 beam_model.Res.Guess.Aero_Nforce(408:528,:)];
end

