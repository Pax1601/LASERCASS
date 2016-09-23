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
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% Luca Cavagna
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%*******************************************************************************
% This function determines new WB struct for updated CONM in GUESS STD
function [beam_model] = update_mass_std(aircraft, beam_model, NEWPOS, M)
%
v = NEWPOS - beam_model.Node.Coord(beam_model.Param.GRDPNT,:);
beam_model.ConM.Offset = v;
%
for k=1:3
  beam_model.ConM.M(k,k,1) = M;
end
S = zeros(3,3);
S(2,1) =  M * v(3); S(3,1) = -M * v(2); S(1,2) = -S(2,1);
S(3,2) =  M * v(1); S(1,3) = -S(3,1);   S(2,3) = -S(3,2);
beam_model.ConM.M(4:6,1:3) = S; beam_model.ConM.M(1:3,4:6) = S';
Jin = diag(diag(aircraft.weight_balance.Imat));
beam_model.ConM.M(4:6, 4:6) = Jin - M * crossm(v) * crossm(v);
%
NODE = beam_model.Node;
[beam_model.WB.CG, beam_model.WB.MCG, beam_model.WB.MRP] = ...
wb_set_conm_mass(beam_model.Info.nconm, NODE.Index, NODE.Coord, NODE.R, beam_model.Param.GRDPNT, beam_model.ConM);
% get principal axes
[beam_model.WB.MCG_pa, beam_model.WB.R_pa] = wb_principal_axis(beam_model.WB.MCG);