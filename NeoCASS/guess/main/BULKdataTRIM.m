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
%
%      <degaspari@aero.polimi.it>
%
%*******************************************************************************
%
% function [] = BULKdataTRIM(fid, maneuver)
%
%   DESCRIPTION: Appends TRIM card to SMARTCAD file for NeoCASS trim
%   analysis or GUESS/NeoCASS trim interface
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    SMARTCAD file pointer
%                maneuver       real       1-D cell array with flight parameters                                      
%        OUTPUT: []
%
% Called by:    writeTRIM2file.m
% 
% Calls:        str2_8ch.m
%
%
%*******************************************************************************
%
function BULKdataTRIM(fid, maneuver)
%
%
global enabled_gui;
if enabled_gui,
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    lab = gui_param.solver.aeros.TRIM_LABELS;
else
    lab = TRIMlabels;
end
labels = lab(:, 1);

% gust check
Igust = ismember(labels, 'VGUST');
if str2double(maneuver{Igust}) == 0.0
    labels(Igust) = [];
    maneuver(Igust) = [];
end

% landing check
Iland = ismember(labels, {'VSINK', 'STROKE', 'LNDGEFF'});
if all(str2double(maneuver(Iland)) == 0);
    labels(Iland) = [];
    maneuver(Iland) = [];
end

fprintf(fid, '\nTRIM    %s%s%s%s', str2_8ch(maneuver{1}), str2_8ch(maneuver{3}),...
                                   str2_8ch(maneuver{4}), str2_8ch(maneuver{5}));
                               
count = 5;

for i = 6:length(labels),

    if ~isempty(maneuver{i})
        
        if isequal(count,9),
            fprintf(fid, '\n        ');
            count=1;
        end

        fprintf(fid, [str2_8ch(labels{i}), str2_8ch(maneuver{i})] );
        
        count= count + 2;
    end

end







% if isempty(CTR_W2)==0
% 
%     fprintf(fid, 'can2r   ');
% 
%     [CTR_W2str] = cnvt2_8chs(CTR_W2);
%     fprintf(fid, '%s', CTR_W2str);
%     count=count+2;
%     if isequal(count,9)==1
%        fprintf(fid, '\n        ');
%        count=1;
%     end
% end






%
% Includendo le prossime 4 schede nelle struct labels{} e maneuver{}
% ricordarsi di modificare la funzione ManeuverSelect in GUI.
%
% if isempty(CLIMB)==0
% 
    if isequal(count,9),
       fprintf(fid, '\n        ');
       count=1;
    end

    fprintf(fid, 'CLIMB   ');

    [CLIMBstr] = cnvt2_8chs(0);
    fprintf(fid, '%s', CLIMBstr);
    count=count+2;
% end





% if isempty(BANK)==0
    if isequal(count,9),
        fprintf(fid, '\n        ');
        count=1;
    end

    fprintf(fid, 'BANK    ');

    [BANKstr] = cnvt2_8chs(0);
    fprintf(fid, '%s', BANKstr);
    count=count+2;
% end


% if isempty(HEAD)==0
    if isequal(count,9),
       fprintf(fid, '\n        ');
       count=1;
    end

    fprintf(fid, 'HEAD    ');

    [HEADstr] = cnvt2_8chs(0);
    fprintf(fid, '%s', HEADstr);
    count=count+2;
% end

% 
% if isempty(THRUST)==0
    if isequal(count,9),
       fprintf(fid, '\n        ');
       %count=1;
    end

    fprintf(fid, 'THRUST  ');

    [THRUSTstr] = cnvt2_8chs(0);
    fprintf(fid, '%s', THRUSTstr);
    %count=count+2;
% end





% fprintf(fid, '\n');




end



