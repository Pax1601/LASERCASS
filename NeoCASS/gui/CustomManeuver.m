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
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090312      1.3     A. De Gaspari    Creation
%     090824      1.3.7   A. De Gaspari    Added maneuver label
%     100216      1.4     A. De Gaspari    Force user to insert a positive
%                                          mach number
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function state_cell = CustomManeuver(nstates, labels)
%
function state_cell = CustomManeuver(nstates, Tlabels, state_cell)
%
labels = Tlabels(:,1);

if nargin < 3
    id = num2cell(1:nstates);
    nlab = length(labels(3:end));
    st = num2cell(zeros(nstates, nlab));
    lab = repmat({'Custom'},nstates,1);
    state_cell = [id' lab st];
end

state_cell = tableGUI('FigName','Custom Maneuvers', 'array',state_cell,...
                      'checks','Yes', 'NumRows',nstates, 'ColNames',labels');

if ~isempty(state_cell) && any(ismember(str2double(state_cell(:,4)), 0))
    hwd = warndlg('All Mach numbers must be positive.', 'NeoCASS - Custom Maneuver Warning');
    uiwait(hwd);
    state_cell = CustomManeuver(nstates, Tlabels, state_cell);
end

