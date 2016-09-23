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

function set_max_table_row(hOb, ed)
%

% Show current settings
handles = guidata(hOb);
old_num = handles.wb.num_vis_rows;
fprintf('\t- Current maximum row number: %2d\n', old_num);

% Open dialog box
screen_size = get(0, 'ScreenSize'); 
fig_dim = [screen_size(3)/2, screen_size(4)/2, 140, 60];
hfig = figure('MenuBar', 'none', 'Name', 'Set row number', 'NumberTitle', 'off',...
    'Position', fig_dim, 'IntegerHandle', 'off');

% Set text box and edit box
uicontrol(hfig, 'Style', 'Text', 'Background', [.8 .8 .8], 'String',...
    'Rows: ','Position', [0, fig_dim(4)/2-10, 60, 20]);
uicontrol(hfig, 'Style', 'Edit', 'Background', [1 1 1], 'String',...
    old_num,'Position', [65, fig_dim(4)/2-10, 50, 20], 'Callback', @edit1_Set_Callback);


    function edit1_Set_Callback(hObject, eventdata) % Rows
        
        new_num = str2double(get(hObject, 'String'));
        
        if new_num ~= old_num
           
           handles.wb.num_vis_rows = new_num;
           fprintf('\t- Maximum row number set to %2d\n', new_num);
           guidata(hOb, handles);
           close(hfig);
           
        end
        
    end


end