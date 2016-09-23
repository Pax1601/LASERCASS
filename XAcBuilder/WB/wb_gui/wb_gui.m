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

function varargout = wb_gui(varargin)
%**************************************************************************
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
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090303      2.1     L.Riccobene      Creation
%
%**************************************************************************
%
% function       wb_gui
%
%
%   DESCRIPTION:  WB_GUI M-file for wb_gui.fig, Weight&Balance 2.1 (will
%                 become obsolete once aircraft and parameter xml will be
%                 merged)
%
%         INPUT: NAME           TYPE         DESCRIPTION
%
%
%         OUTPUT: NAME          TYPE         DESCRIPTION
%
%
%    REFERENCES:
%
%**************************************************************************

%**************************************************************************
%
%         Optional input (label/value pair):
%
%               'names'         cell array   stores parameter names;
%
%               'overwrite'     char         {'yes'|'no'} allow struct
%                                            merge with/without overwriting
%                                            existing values.
%
%**************************************************************************

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name', mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @wb_gui_OpeningFcn, ...
    'gui_OutputFcn',  @wb_gui_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);

if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before wb_gui is made visible.
function wb_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.

% Choose default command line output for wb_gui
handles.output = hObject;

% Initialize weight and balance field
handles.wb = [];
handles.wb.input_filename = [];
handles.wb.param_struct = [];
handles.wb.input = [];
handles.wb.output = [];
handles.wb.param = [];
handles.wb.num_vis_rows = 20;

% Set W&B version
handles.wb.version = wb_version;
set(handles.figure1, 'Name', handles.wb.version);

% Check multiple GUI on computer or/and old wb version
wbguidir = which('wb_gui', '-ALL');
if length(wbguidir) > 1
    warning('wb_gui:MultipleGUI',...
        'More than one W&B GUI found in Matlab path: to avoid conflicts leave only one.');
else
    wbguidir = cell2mat(wbguidir);
    wbdir = wbguidir(1:end-16);
    wb_struct_init_dir = which('wb_struct_init', '-ALL');
    l = length(wb_struct_init_dir);
    if l > 1

        list = cell(l, 1);
        for k = 1:l,
            list{k} = wb_struct_init_dir{k}(1:end-17);
        end

        test = ~ismember(list, wbdir);
        found = list(test);
        warning('wb_gui:MultipleFile',...
            '(%2d) directory with W&B files found: potential conflicts may arise, check them.', sum(test));
        fprintf(' - Directories found:\n');

        for k = 1:length(found),
            fprintf('  - %s\n', found{k});
        end

    end
end

% Set default flag value (don't overwrite parameter values)
flag = false;

% Check if user supplied optional entries
entries = varargin;
names = [];

if rem(length(entries), 2)
    % Optional entries are in pairs
    error('Optional entries must be in pairs label/value');
else
    for i = 1:2:length(entries),
        switch lower(entries{i})

            case 'names'

                names = varargin{i+1};

            case 'overwrite'

                if strcmpi(varargin{i+1}, 'yes')
                    flag = true;
                else
                    flag = false;
                end

            otherwise

                warning('Property %s not yet supported', entries{i});

        end
    end
end

% Save overwrite flag
handles.wb.flag = flag;

% Mandatory parameters (default)
if isempty(names)

    handles.wb.param.names = {'Thrust_to_weight_ratio',...
        'installation_type',...
        'gross_volume',...
        'Baggage_combined_length',...
        'Baggage_apex_per_fuselgt',...
        'Cabin_length_to_aft_cab',...
        'Cabin_max_internal_height',...
        'Cabin_max_internal_width',...
        'Cabin_floor_width',...
        'Cabin_volume',...
        'Cabin_attendant_number',...
        'Flight_crew_number',...
        'Passenger_accomodation',...
        'Seats_abreast_in_fuselage',...
        'Seat_pitch',...
        'Maximum_cabin_altitude',...
        'Max_pressure_differential',...
        'Maximum_fuel_in_wings',...
        'Maximum_fuel_in_auxiliary',...
        'Maximum_fuel_in_central_wingbox ',...
        'MFW_decrement_to_MTOW',...
        'VD_Flight_envelope_dive',...
        'VMO_Flight_envelope',...
        'Landing_gear',...
        'main_landing_gear_x_cg',...
        'main_landing_gear_z_cg',...
        'Aux_Landing_gear',...
        'aux_landing_gear_x_cg',...
        'aux_landing_gear_z_cg',...
        'box_ea_loc_root',...
        'box_ea_loc_kink1',...
        'box_ea_loc_kink2',...
        'box_ea_loc_tip',...
        'box_semispan_root',...
        'box_semispan_kink1',...
        'box_semispan_kink2',...
        'box_semispan_tip',...
        'Centre_tank_portion_used',...
        'Fuel_tank_auxiliary_x_cg',...
        'Fuel_tank_auxiliary_z_cg',...
        'IXXINER',...
        'IYYINER',...
        'IZZINER',...
        'IXYINER',...
        'IYZINER',...
        'IXZINER'};
    %         'Design_classification',...
    %         'Aft_fuse_bladder_length',...  % useful in aux tank volume estimate
    %         'box2_ea_loc_root',...
    %         'box2_ea_loc_kink1',...
    %         'box2_ea_loc_kink2',...
    %         'box2_ea_loc_tip',...
    %         'box2_semispan_root',...
    %         'box2_semispan_kink1',...
    %         'box2_semispan_kink2',...
    %         'box2_semispan_tip',...
    %         'Fuel_tank_wing_x_cg',...
    %         'Fuel_tank_wing_z_cg',...
    %         'Fuel_centre_plus_confor_x_cg',...
    %         'Fuel_centre_plus_confor_z_cg'};
else
    % user supplied mandatory parameters
    handles.wb.param.names = names;
end

% Part names
handles.wb.param.part_names = {'WING1'...
    'WING2'...
    'HT'...
    'VT'...
    'FUSELAGE'...
    'LANDING GEAR'...
    'POWERPLANT1'...
    'POWERPLANT2'...
    'AUX LANDING GEAR'...
    'VT2'...
    'CANARD'...
    'TAILBOOMS'...
    '############'...
    '############'...
    '############'...
    '############'...
    'SYSTEMS'...
    'WING TANKS'...
    'CENTRE FUEL TANKS'...
    'AUXILIARY TANKS'...
    'INTERIOR'...
    'PILOTS'...
    'CREW'...
    'PASSENGERS'...
    'BAGGAGE'...
    '############'...
    'CG_at_MTOW_wrt_nose'...
    '############'...
    'CG_at_MEW_wrt_nose'...
    '############'};

handles.wb.show_handles = zeros(1, 3);

% Set 'Option' menu
handles.uimenu1 = uimenu('Parent', handles.figure1, 'Label', 'Option');
uimenu('Parent', handles.uimenu1, 'Label', 'Table max rows', 'Callback', @set_max_table_row);
h_export = uimenu('Parent', handles.uimenu1, 'Label', 'Export *.xls', 'Callback', @export_excel_callback);
if isunix
    set(h_export, 'Enable', 'Off');
end
uimenu('Parent', handles.uimenu1, 'Label', 'Show state', 'Callback', @show_state_callback);
handles.hl = uimenu('Parent', handles.uimenu1,...
    'Label', 'Exit', 'Callback', 'close', 'Separator', 'on');


% Update handles structure
guidata(hObject, handles);

% Disable all pushbutton other than 'Load'
for n = [2:4 9 10],
    name = ['pushbutton', num2str(n)];
    set(handles.(name), 'Enable', 'Off');
end

% Disable all checkbox
for n = 1:3,
    name = ['checkbox', num2str(n)];
    set(handles.(name), 'Enable', 'Off');
end


% --- Outputs from this function are returned to the command line.
function varargout = wb_gui_OutputFcn(hObject, eventdata, handles)
%
varargout{1} = handles.output;


function export_excel_callback(hObject, eventdata)
%

handles = guidata(hObject);
if ~isempty(handles.wb.output)

    export_file_name = [regexp(handles.wb.input_filename, '(\w+)|\.', 'match', 'once'), '_excel.xls'];
    cog = handles.wb.output.weight_balance.COG(:,:,1);
    Imat = handles.wb.output.weight_balance.Imat;
    res = export_COG(export_file_name, handles.wb.param.part_names, cog, Imat);

else
    fprintf('\t- No data to process.\n');
end


function show_state_callback(hObject, eventdata)
%

handles = guidata(hObject);
wb_state = handles.wb;
fprintf(' - WB GUI state:\n');
disp(wb_state);
fprintf(' - Accessing W&B workspace (to exit press F5 or digit "return").\n');
keyboard
fprintf(' - Exiting GUI workspace.\n');


%--------------------------------------------------------------------------
% Load/Save
%--------------------------------------------------------------------------
%

% Load XML file button
function pushbutton1_Callback(hObject, eventdata, handles)
%

[file, pathname, filterindex] = uigetfile('*.xml', 'Select a file: ');

% If the file exist, load it
if filterindex

    % Convert XML in struct (XML toolbox needed)
    fname = [pathname, file];

    % Always add all fields needed by WB (useful when a short list of
    % parameters is loaded)
    add_param = wb_struct_init();
    input = mergestruct2(neocass_xmlwrapper(fname), add_param, handles.wb.flag);
    handles.wb.input = input;

    handles.wb.input_filename = file;

    % Update struct
    guidata(hObject, handles);

    if strcmp(get(handles.pushbutton9, 'Enable'), 'off')
        set(handles.pushbutton9, 'Enable', 'On');
    end

    fprintf('%s loaded...\n', fname);

else
    fprintf('No file specified.\n');
    return
end


% Load additional XML
function pushbutton9_Callback(hObject, eventdata, handles)
%

[file, pathname, filterindex] = uigetfile('*.xml', 'Select a file: ');

% If the file exist, load it
if filterindex

    % Convert XML in struct (XML toolbox needed)
    fname = [pathname, file];

    % Add/overwrite parameters to input struct
    handles.wb.input = mergestruct2(handles.wb.input, neocass_xmlwrapper(fname), handles.wb.flag);
    handles.wb.param_struct = neocass_xmlwrapper(fname);
    
    fprintf('%s loaded...\n', fname);

    % Extract fields value and initialize values cell array
    names = handles.wb.param.names;
    values = cell(length(names), 1);
    for i = 1:length(names),
        values{i} = extract_field_value(handles.wb.input, names{i});
        if isempty(values{i})
            values{i} = 0.;
        end
        % Avoid possible conflict in tableGUI which accepts only
        % double/char
        if ~isfloat(values{i})
            values{i} = double(values{i});
        end    
    end

    % Check parameter consistency
    if isempty(values{1})
        error('No correspondence found with the parameters specified!');
    else
        handles.wb.param.values = values;
    end

    % Initialize temporary variable
    handles.wb.param.values_tmp = values;

    % Update struct
    guidata(hObject, handles);

    % Toggle enable on, if the case
    if strcmp( get(handles.pushbutton2, 'Enable'), 'off')
        for n = [3:4 10],
            name = ['pushbutton', num2str(n)];
            set(handles.(name), 'Enable', 'On');
        end

    end

else
    fprintf('No file specified.\n');
    return
end


% Save XML pushbutton
function pushbutton10_Callback(hObject, eventdata, handles)
%

% Set default output file name
out_name = handles.wb.input_filename;
% Get last '.' index
I = regexpi(out_name, '\.(\w*)$') - 1;
out_fname = out_name(1:I);
out_name = [out_fname, '_WB.xml'];

% Save results
neocass_xmlunwrapper(out_name, handles.wb.output);
fprintf('Calculation saved in %s.\n', out_name);

% Save param to xml
out_name_param = [out_fname, '_WB_param.xml'];
params = extract_struct(handles.wb.output, handles.wb.param.names);
neocass_xmlunwrapper(out_name_param, params);
fprintf('Parameters saved in %s.\n', out_name_param);

guidata(hObject, handles);


%
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Modify parameters and update calculations
%--------------------------------------------------------------------------
%

% Update calculation button
function pushbutton2_Callback(hObject, eventdata, handles)
%

if isempty(handles.wb.output)
    % Enable all checkbox the first time update button is pressed
    for n = 1:3,
        name = ['checkbox', num2str(n)];
        set(handles.(name), 'Enable', 'On');
    end
end

% Update calculation
handles.wb.output = weight_xml(handles.wb.input);

% Extract fields values and update temporary values vector
names = handles.wb.param.names;
values = cell(length(names), 1);
for i = 1:length(names),
    values{i} = extract_field_value(handles.wb.output, names{i});
    if isempty(values{i})
        values{i} = 0.;
    end
    % Avoid possible conflict in tableGUI which accepts only
    % double/char
    if ~isfloat(values{i})
        values{i} = double(values{i});
    end
end

handles.wb.param.values_tmp = values;

% Update handles
guidata(hObject, handles);

% Update tables if any open
if get(handles.checkbox1, 'Value')
    cog = handles.wb.output.weight_balance.COG(:,:,1);
    update_table(handles.wb.show_handles(1), cog);
end

if get(handles.checkbox2, 'Value')
    Imat = handles.wb.output.weight_balance.Imat;
    update_table(handles.wb.show_handles(2), Imat);
end

if get(handles.checkbox3, 'Value')
    cog = handles.wb.output.weight_balance.COG(:,:,1);
    % Recover plot handles
    hf = handles.wb.show_handles(3);
    plot_handles = get(get(hf, 'Children'), 'Children');
    plot_handles = flipud(plot_handles{2});
    update_graph(plot_handles, cog);
end


% Modify parameter button
function pushbutton3_Callback(hObject, eventdata, handles)
%

% Load values, update and save them in handles
names  = handles.wb.param.names;
values = handles.wb.param.values_tmp;

% Set parameter values by graphical interface
upd_values = tableGUI_mod('array', values, 'NumRows', numel(names), 'NumCol', 2, 'ColNames',...
    {'Value'}, 'RowNames', names, 'RowNamesWidth', 250, 'MAX_ROWS', handles.wb.num_vis_rows);

if isempty(upd_values)
    % Cancel button pressed
    upd_values = values;
else

    % Convert update values from char to double
    for i = 1:length(upd_values),
        upd_values{i} = str2num(upd_values{i}); 
    end

    % Compare cell arrays and find what has been changed
    change_index = cell2mat(values) ~= cell2mat(upd_values);

    chd_names = names(change_index);
    chd_upd_values = upd_values(change_index);

    % Update parameters
    for i = 1:length(chd_upd_values),
        handles.wb.input = assign_field_value(handles.wb.input, chd_names{i}, chd_upd_values{i});
    end

end

% Update values
handles.wb.param.values_tmp = upd_values;


% The first time the GUI is called force the user to modify parameters and
% THEN to update
n = 2;
name = ['pushbutton', num2str(n)];
if strcmp(get(handles.(name), 'Enable'), 'off')
    set(handles.(name), 'Enable', 'On');
end

% Update variable
guidata(hObject, handles);


% Reset all parameter button
function pushbutton4_Callback(hObject, eventdata, handles)
%

handles.wb.param.values_tmp = handles.wb.param.values;
names = handles.wb.param.names;

% Update parameters
for i = 1:length(handles.wb.param.values),
    handles.wb.input = assign_field_value(handles.wb.input, names{i}, handles.wb.param.values{i});
end

fprintf('Parameters reset to default values\n');
guidata(hObject, handles);

%
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Show results
%--------------------------------------------------------------------------

% WB resume checkbox
function checkbox1_Callback(hObject, eventdata, handles)
% WB resume
%

status = get(hObject,'Value');
if status

    cog    = handles.wb.output.weight_balance.COG(:,:,1);
    parts  = handles.wb.param.part_names;
    partID = 1:numel(parts); 
    handles.wb.show_handles(1) = tableGUI_mod('array', [partID' cog], 'Numrows', numel(parts), 'NumCol', 5, 'ColNames',...
        {'#' 'X [m]' 'Y [m]' 'Z [m]' 'M [Kg]'}, 'RowNames', parts,...
        'RowNamesWidth', 250, 'MAX_ROWS', numel(parts),...
        'modal', '', 'Figname', 'WB resume - PLOTCGS matrix');

    guidata(hObject, handles);

else

    try
        close(handles.wb.show_handles(1));
        %     catch
        %         warning('Table window closed - use checkbox instead!');
    end

end


% Inertia matrix checkbox
function checkbox2_Callback(hObject, eventdata, handles)
% Inertia matrix
%

status = get(hObject,'Value');
if status

    Imat = handles.wb.output.weight_balance.Imat;
    handles.wb.show_handles(2) = tableGUI_mod('array', Imat, 'Numrows',3, 'NumCol', 3, 'ColNames',...
        {'' '' ''}, 'RowNames', {'' '' ''}, 'modal', '', 'Figname', 'Inertia matrix',...
        'Colwidth', 100);

    guidata(hObject, handles);

else

    try
        close(handles.wb.show_handles(2));
        %     catch
        %         warning('Table window closed - use checkbox instead!');
    end

end


% Graphic checkbox
function checkbox3_Callback(hObject, eventdata, handles)
% CG positions

status = get(hObject,'Value');
if status

    cog = handles.wb.output.weight_balance.COG(:,:,1);

    % Sample only non zero elements
    index = cog(:, 1) ~= 0;
    index = index(1:26);

    handles.wb.show_handles(3) = figure('IntegerHandle', 'off');

    % Plot parts CogS
    plot(cog(index, 1), cog(index, 3), 'kd', 'markerfacecolor', 'k');
    part_names = handles.wb.param.part_names;
    htext = text(cog(index, 1), cog(index, 3), part_names(index),...
        'Fontname', 'Helvetica', 'Fontsize', 6);

    % Traslate text
    factor = .1;
    for i = 1:length(htext)
        ex = get(htext(i), 'Extent');
        pos = get(htext(i), 'Position');
        pos(1:2) = [pos(1) + ex(3)*factor, pos(2) + ex(4)*factor];
        set(htext(i), 'Position', pos);
    end

    hold on;
    grid on;

    % Plot global CoG
    plot(cog(27, 1), cog(27, 3), 'ro', cog(29, 1), cog(29, 3), 'bo', ...
        'linewidth', 6);

    title('CoGs envelope on simmetry plane (x-z)');
    legend('Parts', 'CoG at MTOW', 'CoG at MEW', 'Location', 'NorthWest');
    xlabel('X [m]');
    ylabel('Z [m]');

    % Set figure extents
    fuselgt = handles.wb.input.Fuselage.Total_fuselage_length;
    x_lim = [0 fuselgt];
    y_lim = [-fuselgt/3 fuselgt/3];
    xlim(x_lim);
    ylim(y_lim);
    guidata(hObject, handles);

else

    try
        close(handles.wb.show_handles(3));
        %     catch
        %         warning('Table window closed - use checkbox instead!');
    end

end

%
%--------------------------------------------------------------------------


%==========================================================================
% Auxiliary function


function update_table(table_handle, var)
%

tmp = get(table_handle, 'UserData');
b = tmp.hEdits;
[row, col] = size(b);

for n = 1:row,
    for m = 1:col,
        set(b(n, m), 'String', num2str(var(n, m), '%6.2f'));
    end
end


function update_graph(plot_handles, var)
%

check = false(length(plot_handles), 1);
for i = 1:length(plot_handles)
    tmp = get(plot_handles(i));
    if isfield(tmp, 'XData')
        check(i) = true;
    end
end
htext = plot_handles(~check);
plot_handles = plot_handles(check);
l = length(plot_handles);
index = var(:, 1) ~= 0;
index = index(1:26);

% Set updated coordinates
x = {var(index, 1), var(27, 1), var(29, 1)};
z = {var(index, 3), var(27, 3), var(29, 3)};
for i = 1:l,
    set(plot_handles(i), 'XData', x{i}, 'YData', z{i});
end

% Update text boxes
factor = 0.1;
var = var(index, :);
for i = 1:length(htext)
    ex = get(htext(i), 'Extent');
    pos = get(htext(i), 'Position');
    pos(1:2) = [var(i, 1) + ex(3)*factor, var(i, 3) + ex(4)*factor];
    set(htext(i), 'Position', pos);
end
