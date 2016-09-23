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
%     090827      1.3.7   A. De Gaspari    Modification
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function writeTRIM2file(fid, state)
%
%   DESCRIPTION: Appends multiple TRIM cards to SMARTCAD file for NeoCASS trim
%   analysis or GUESS/NeoCASS trim interface
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    SMARTCAD file pointer
%                maneuver       real       2-D cell array containing flight
%                condition and flight parameters
%        OUTPUT: []
%
% Called by:    export_model_smartcad.m
%               export_solver_cards.m
% 
% Calls:        BULKdataTRIM.m
%
%
%*******************************************************************************
%
function writeTRIM2file(fid, state)
%
fprintf(fid, '\n$\n$ Flight condition parameters');

ns = size(state, 1);

global enabled_gui;
if enabled_gui,
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    Tl = gui_param.solver.aeros.TRIM_LABELS;
else
    Tl = TRIMlabels;
end

dum(6:length(Tl)) = true;

for i = 1:ns,

    lab = Tl(strcmp(state(i,:),''), 2); 
    strlab = [repmat('\n$  ',length(lab),1), char(lab)];
    
    cst = str2double(state(i,:));
    inlab = Tl(dum & cst ~= 0 & ~isnan(cst), 2);
    chlab = [char(inlab), repmat(',', length(inlab), 1)];
    strinlab = reshape(chlab',1,numel(chlab));

    fprintf(fid, '\n$');
    fprintf(fid, '\nTRIM= %d, %s', str2double(state{i, 1}), state{i, 2});
    fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
    fprintf(fid, '\n$');
    fprintf(fid, '\n$  ID: %d', str2double(state{i, 1}));
    fprintf(fid, '\n$');
    fprintf(fid, '\n$ %s determine:', upper(state{i, 2}));
    fprintf(fid, strlab');
    if ~isempty(strinlab),
        fprintf(fid, '\n$ Input: ');
        fprintf(fid, strinlab(regexpi(strinlab,'\S'))');
        fprintf(fid, '.');
    end
    fprintf(fid, '\n$');

    BULKdataTRIM(fid, state(i, :));

end

fprintf(fid, '\n$\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n\n');



% fprintf('\n\t- TRIM card;\n');


end


