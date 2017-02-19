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

function varargout = NeoCASS(varargin)
% NEOCASS M-file for NeoCASS.fig
%      NEOCASS, by itself, creates a new NEOCASS or raises the existing
%      singleton*.
%
%      H = NEOCASS returns the handle to a new NEOCASS or the handle to
%      the existing singleton*.
%
%      NEOCASS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in NEOCASS.M with the given input arguments.
%
%      NEOCASS('Property','Value',...) creates a new NEOCASS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before NeoCASS_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to NeoCASS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2002-2003 The MathWorks, Inc.

% Edit the above text to modify the response to help NeoCASS

% Last Modified by GUIDE v2.5 26-Nov-2014 17:56:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @NeoCASS_OpeningFcn, ...
    'gui_OutputFcn',  @NeoCASS_OutputFcn, ...
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


% --- Executes just before NeoCASS is made visible.
function NeoCASS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to NeoCASS (see VARARGIN)

% Choose default command line output for NeoCASS
handles.output = hObject;

% UIWAIT makes NeoCASS wait for user response (see UIRESUME)
% uiwait(handles.figure1);

hfig = handles.figure1;

% Check version
status = CheckNeoCASSversionOnServer;
handles.CheckStatus = status;

% Update handles structure
guidata(hObject, handles);

switch status
    
    case 0
        
        % Set main figure properties
        set(hfig, 'CloseRequestFcn', @close_gui);
        set(handles.Main_Settings_popupmenu2str,'Value',1);
        
        set(handles.Main_Settings_edit1,'Value',1);
        %set(handles.Main_Settings_edit1,'Visible','off');
        %set(handles.Main_Settings_text3,'Visible','off');
        set(handles.Main_Settings_edit1,'Enable','off');
        set(handles.Main_Settings_text3,'Enable','off');
        
        % Fix dimensions
        set(handles.figure1, 'units', 'pixel');
        opos = get(handles.figure1, 'position');
        set(handles.figure1, 'position', [opos(1:2) 548 201]);
        
        % Load splash page
        neocass_splash;
        
        % Show NeoCASS version
        fprintf(1,'\n - %s\n',get_neocass_version('NeoCASS'));
        fprintf(1, '\n - Initializing NeoCASS GUI database...');
        
        % Definition of global variables
        global enabled_gui;
        enabled_gui = true;
        
        guipath_init;
        
        init_gui_params('neocass_gui_param.mat');
        neocass_init;
        fprintf(1, 'done. \n');
        
        global enabled_codes;
        enabled_codes = zeros(1,8);
        
    case -1
        
        % Init gui parameters to avoid warning
        init_gui_params('neocass_gui_param.mat');
        
end


% --- Outputs from this function are returned to the command line.
function varargout = NeoCASS_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

switch handles.CheckStatus
    
    case 0
        
        % Switch off push button to open technology file
        set(handles.editguess,'Enable','Off');
        set(handles.openguess,'Enable','Off');
        
        % Switch off checkboxes for enabled solvers
        code_disable(7,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
        code_disable(8,handles.Main_File_checkbox1VLM,handles.Main_Run_pushbutton6vlm);
        code_disable(1,handles.Main_File_checkbox2static,handles.Main_Run_pushbutton2static);
        code_disable(2,handles.Main_File_checkbox3modal,handles.Main_Run_pushbutton3modal);
        code_disable(3,handles.Main_File_checkbox4staer,handles.Main_Run_pushbutton4staer);
        code_disable(4,handles.Main_File_checkbox5flutter,handles.Main_Run_pushbutton5flutter);
        
        % Switch off parameter for non-linear solver
        set(handles.Main_Settings_text2,'Enable','Off');
        set(handles.Main_Settings_text4,'Enable','Off');
        set(handles.Main_Settings_text5,'Enable','Off');
        set(handles.Main_Settings_editniter,'Enable','Off');
        set(handles.Main_Settings_editnstep,'Enable','Off');
        set(handles.Main_Settings_editrestol,'Enable','Off');
        
        % Switch off Start Button for Run solvers
        set(handles.Main_Run_pushbutton6start,'Enable','Off');
        
    case -1
        
        % Quit
        close_gui(hObject);
        
end

% ------------ end of NeoCASS controls initialization ---------------
%
% --- Executes on button press in Main_File_openaircraft.
function Main_File_openaircraft_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_openaircraft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sizeinput_pathname

if ~sizeinput_pathname
    sizeinput_pathname = pwd;
end

[aircraft_filename, aircraft_pathname, aircraft_filterindex] = uigetfile('*.xml', 'Select aircraft geometry file',sizeinput_pathname);

if strcmp(sizeinput_pathname,neocass_path)
    sizeinput_pathname = aircraft_pathname;
end

if (aircraft_filterindex ~= 0)
    
    filename = [aircraft_pathname, aircraft_filename];
    
    fprintf(1, '\n - GUESS aircraft filename: %s. \n', filename);
    
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    gui_param.guess.file.aircraft = filename;
    
    aircraft = neocass_xmlwrapper(filename);
    
    Aaft = pi/2 *( 2*aircraft.Fuselage.a0_aft^2 + aircraft.Fuselage.a1_aft^2 + aircraft.Fuselage.b1_aft^2 );
    Raft = sqrt( Aaft/pi );
    
    MIN = 0.01*aircraft.Wing1.Span*0.5;
    inboard  = aircraft.Wing1.spanwise_kink1 *(aircraft.Wing1.Span*0.5)- Raft;
    midboard = (aircraft.Wing1.spanwise_kink2 - aircraft.Wing1.spanwise_kink1) *(aircraft.Wing1.Span*0.5);
    outboard = (1 - aircraft.Wing1.spanwise_kink2) *(aircraft.Wing1.Span*0.5);
    
    
    
    trim_param = TRIMlabels;
    InotCS = false(1, length(trim_param));
    InotCS(17) = ~(aircraft.Wing1.present && aircraft.Wing1.flap.present && (inboard > MIN));
    InotCS(18) = ~(aircraft.Wing1.present && aircraft.Wing1.flap.present && (midboard > MIN));
    InotCS(19) = ~(aircraft.Wing1.present && aircraft.Wing1.aileron.present && (outboard > MIN));
    InotCS(20) = ~(aircraft.Horizontal_tail.present && aircraft.Horizontal_tail.Elevator.present);
    InotCS(21) = ~(aircraft.Vertical_tail.present && aircraft.Vertical_tail.Rudder.present);
    InotCS(22) = ~(aircraft.Canard.present && aircraft.Canard.Elevator.present);
    trim_param(InotCS',:) = [];
    
    % Trim labels
    gui_param.solver.aeros.TRIM_LABELS = trim_param;
    % State matrix
    gui_param.solver.aeros.STATE_MAT = cell(0,length(trim_param));
    gui_param.solver.aeros.CS = true;
    
    if all(isfield(aircraft,{'user_input', 'experienced_user_input'})),
        gui_param.guess.file.technology = filename;
        set(handles.editguess,'Enable','Off');
        set(handles.openguess,'Enable','Off');
    else
        gui_param.guess.file.technology = '';
        set(handles.editguess,'Enable','On');
        set(handles.openguess,'Enable','On');
    end
    
    save_gui_params('neocass_gui_param.mat', gui_param);
    
end


% --- Executes on button press in Main_File_editaircraft.
function Main_File_editaircraft_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_editaircraft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
filename = gui_param.guess.file.aircraft;

if length(filename)
    command = neocass_xml_editor_path;
    if ~isempty(command)
        [o1, void] = system([command, ' ', filename]);
    else
        fprintf(1, '\n - XML editor not set. \n');
    end
else
    fprintf(1, '\n - No aircraft .xml file given. \n');
end

% --- Executes on button press in Main_File_openstates.
function Main_File_openstates_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_openstates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

%

gui_param.guess.file.state = '';

%

[filename_trim, gui_param.guess.model.Joined_wing, gui_param.guess.model.Strut_wing, gui_param.guess.model.guessstd, gui_param.guess.model.EnvPoints] = guess2trim(gui_param.guess.file.aircraft);

if isempty(filename_trim),
    gui_param.guess.file.trim = '';
    fprintf(1, '\n - Guess in standard mode.\n');
else
    gui_param.guess.file.trim = filename_trim;
    fprintf(1, '\n - GUESS/SMARTCAD trim filename: %s.\n', filename_trim);
end

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes on button press in Main_File_editstates.
function Main_File_editstates_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_editstates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
filename = gui_param.guess.file.state;
if length(filename)
    command = neocass_xml_editor_path;
    if ~isempty(command)
        [o1, void] = system([command, ' ', filename]);
    else
        fprintf(1, '\n - XML editor not set. \n');
    end
else
    fprintf(1, '\n - No state .xml file given. \n');
end



% --- Executes on button press in Main_File_outguess.
function Main_File_outguess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_outguess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[guess_filename, guess_pathname, guess_filterindex] = uigetfile('*.xml', 'Pick a guess XML-file');




% --- Executes on button press in Main_File_editguess.
function Main_File_editguess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_editguess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sizeinput_pathname

if ~sizeinput_pathname
    sizeinput_pathname = neocass_path;
end

[tech_filename, tech_pathname, tech_filterindex] = uigetfile('*.xml', 'Select technology file',sizeinput_pathname);

if strcmp(sizeinput_pathname,neocass_path)
    sizeinput_pathname = tech_pathname;
end

if (tech_filterindex ~= 0)
    
    filename = [tech_pathname, tech_filename];
    
    fprintf(1, '\n - GUESS technology filename: %s. \n', filename);
    
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    
    gui_param.guess.file.technology = filename;
    
    save_gui_params('neocass_gui_param.mat', gui_param);
    
end

% --- Executes on button press in Main_File_settingsguess.
function Main_File_openguess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_settingsguess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
filename = gui_param.guess.file.technology;

if length(filename)
    command = neocass_xml_editor_path;
    if ~isempty(command)
        [o1, void] = system([command, ' ', filename]);
    else
        fprintf(1, '\n - XML editor not set. \n');
    end
else
    fprintf(1, '\n - No technology .xml file given. \n');
end


% --- Executes on button press in Main_File_runguess.
function Main_File_runguess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_runguess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sizeinput_pathname
global guess_pathname

if ~guess_pathname
    guess_pathname = neocass_path;
end

formats = { '*.inc', 'GUESS file (*.inc)';...
    '*.dat', 'GUESS file (*.dat)';...
    '*.*', 'All files (*.*)' };

guess_pathname = sizeinput_pathname;

[stick_filename, stick_pathname, stick_filterindex] = uiputfile(formats, 'Select output filename', guess_pathname);

guess_pathname = stick_pathname;


if (stick_filterindex ~= 0)
    
    stick_filename = uiputext(stick_filename, stick_filterindex, formats);
    
    stick_filename = [stick_pathname, stick_filename];
    
    fprintf(1, '\n - GUESS output stick filename: %s. \n', stick_filename);
    
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    gui_param.guess.file.stick = stick_filename;
    save_gui_params('neocass_gui_param.mat', gui_param);
    
    aircraft_filename = gui_param.guess.file.aircraft;
    % state_filename = gui_param.guess.file.state;
    tech_filename = gui_param.guess.file.technology;
    trim_filename = gui_param.guess.file.trim;
    model = gui_param.guess.model;
    
    if ( ~isempty(aircraft_filename))
        if (~isempty(tech_filename))
            init_guess_model;   % initialize GUESS model struct
            guess_model = guess(aircraft_filename, tech_filename, stick_filename, trim_filename, model);
        else
            fprintf('\n - No technology .xml file given. \n');
        end
    else
        fprintf('\n - No aircraft geometry .xml file given. \n');
    end
    
end


% --- Executes on button press in Main_File_generate.
function Main_File_generate_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_generate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% Input .dat file name where to save analysis model
global smartcad_pathname
global guess_pathname

if ~smartcad_pathname
    smartcad_pathname = neocass_path;
end

formats = { '*.inc', 'Solver input data file (*.inc)';...
    '*.dat', 'Solver input data file (*.dat)';...
    '*.*', 'All files (*.*)' };

smartcad_pathname = guess_pathname;

[neocass_filename, neocass_pathname, neocass_filterindex] = uiputfile(formats, 'Generate Solver input data file', smartcad_pathname);

smartcad_pathname = neocass_pathname;


path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

if (neocass_filterindex ~= 0)
    
    neocass_filename = uiputext(neocass_filename, neocass_filterindex, formats);
    
    filename = [neocass_pathname, neocass_filename];
    fprintf(1, '\n - Solver input data file: %s. \n', filename);
    gui_param.solver.file.SOLVER = filename;
    fp = fopen(filename, 'w');
    export_solver_cards(fp, 'neocass_gui_param.mat');
    fclose(fp);
else
    gui_param.solver.file.SOLVER = '';
end

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes on button press in Main_File_pushbutton_assembly.
function Main_File_pushbutton_assembly_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_pushbutton_assembly (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global smartcad_pathname
global guess_pathname

if ~smartcad_pathname
    smartcad_pathname = neocass_path;
end

formats = { '*.dat', 'SMARTCAD include file (*.dat)';...
    '*.dat', 'SMARTCAD append file (*.dat)';...
    '*.*', 'All files (*.*)'};

if ~strcmp(guess_pathname, neocass_path)
    smartcad_pathname = guess_pathname;
end

[neocass_filename, neocass_pathname, neocass_filterindex] = uiputfile(formats, 'Generate SMARTCAD file', smartcad_pathname);

smartcad_pathname = neocass_pathname;


path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

if (neocass_filterindex ~= 0)
    
    neocass_filename = uiputext(neocass_filename, neocass_filterindex, formats);
    
    filename = [neocass_pathname, neocass_filename];
    
    switch neocass_filterindex
        case {1,3}
            fprintf(1, '\n - SMARTCAD file: %s. \n', filename);
            gui_param.solver.file.FILE = filename;
            text = 'List of files in the SMARTCAD current directory: ';
            
            dlist_inc = dir([neocass_pathname,'*.inc']);
            dlist_dat = dir([neocass_pathname,'*.dat']);
            % I = ~[dlist.isdir]';
            % flist = {dlist(I).name}';
            flist = sort({dlist_inc.name, dlist_dat.name})';
            sfile = select_list(flist, text);
            fp = fopen(filename, 'w');
            fprintf(fp, '$ SMARTCAD cards included by NeoCASS GUI (%s)', datestr(now));
            fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
            for i = 1:length(sfile),
                fprintf(fp, '\n');
                line = ['INCLUDE ', neocass_pathname, flist{sfile(i)}];
                % line = ['INCLUDE ', flist{sfile(i)}];
                fprintf(fp, '%s', line);
            end
            fclose(fp);
            
        case 2
            if isempty(gui_param.solver.file.SOLVER),
                warning('NOfile:NoDriverCards','\n - No solver input data file generated.');
                fprintf(1, '\n - Push GENERATE button to create it.\n');
                
            elseif isempty(gui_param.guess.file.stick),
                warning('NOfile:NoModelCards','\n - No model file generated.');
                fprintf(1, '\n - Push RUN GUESS button to generate it.\n');
                
            else
                fprintf(1, '\n - SMARTCAD file: %s. \n', filename);
                gui_param.solver.file.FILE = filename;
                
                if (isunix)
                    command = ['cp ',gui_param.guess.file.stick,' ', filename];
                else
                    command = ['copy ',gui_param.guess.file.stick,' ', filename];
                end
                system(command);
                fp = fopen(filename, 'a');
                export_solver_cards(fp, 'neocass_gui_param.mat');
                fclose(fp);
                
            end
            
    end
    
else
    gui_param.solver.file.FILE = '';
end

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes on button press in Main_File_openneocass.
function Main_File_openneocass_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_openneocass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global smartcad_pathname

if ~smartcad_pathname
    smartcad_pathname = neocass_path;
end

formats = { '*.dat', 'SMARTCAD assembly file (*.dat)';...
    '*.inc', 'Model card file (*.inc)'};

[neocass_filename, neocass_pathname, neocass_filterindex] = uigetfile(formats, 'Open SMARTCAD file', smartcad_pathname);

smartcad_pathname = neocass_pathname;


% global enabled_codes;
global beam_model;

if (neocass_filterindex ~= 0)
    
    neocass_init;
    filename = [neocass_pathname, neocass_filename];
    
    fprintf(1, 'SMARTCAD filename %s. \n', filename);
    %
    beam_model = load_nastran_model(filename);
    %
    enable_solver(handles);
    
end


% --- Executes on button press in Main_File_analysissettings.
function Main_File_analysissettings_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_analysissettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% Run ANALYSIS_Settings gui to create EIGR TRIM and FLUTTER cards
ANALYSIS_Settings;

% --- Executes on button press in Main_File_referencesettings.
function Main_File_referencesettings_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_referencesettings (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% Run ANALYSIS_Settings gui to create EIGR TRIM and FLUTTER cards
REFERENCE_Settings;


% --- Executes on button press in Main_File_editneocass.
function Main_File_editneocass_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_editneocass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
formats = { '*.dat; *.inc', 'SMARTCAD file (*.dat; *.inc)'};
[neocass_filename, neocass_pathname, neocass_filterindex] = uigetfile(formats, 'Edit SMARTCAD files');

if (neocass_filterindex ~= 0)
    
    filename = ['"', neocass_pathname, neocass_filename, '"'];
    
    command = neocass_text_editor_path;
    if isempty(command)
        command = 'edit';
        command = [command, ' ', filename];
        eval(command);
    else
        command = [command, ' ', filename];
        system(command);
    end
    
else
    fprintf(1, '\n - No SMARTCAD input file given. \n');
end

% --- Executes on button press in Main_File_checkbox1guess.
function Main_File_checkbox1VLM_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkbox1guess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1
global enabled_codes;
enabled_codes(7) = get(hObject,'Value')

if (enabled_codes(7) == 0)
    set(handles.Main_Run_pushbutton6vlm,'Enable','Off');
else
    set(handles.Main_Run_pushbutton6vlm,'Enable','On');
end


% --- Executes on button press in Main_File_checkbox2static.
function Main_File_checkbox2static_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkbox2static (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1

% Check if box for static has been selected or not, then
% activate/deactivates the corresponding solver button

global enabled_codes;
enabled_codes(1) = get(hObject,'Value');
if (enabled_codes(1) == 0)
    set(handles.Main_Run_pushbutton2static,'Enable','Off');
else
    set(handles.Main_Run_pushbutton2static,'Enable','On');
end


% --- Executes on button press in Main_File_checkbox3modal.
function Main_File_checkbox3modal_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkbox3modal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1

% Check if box for modal  has been selected or not
global enabled_codes;
enabled_codes(2) = get(hObject,'Value');

if (enabled_codes(2) == 0)
    set(handles.Main_Run_pushbutton3modal,'Enable','Off');
else
    set(handles.Main_Run_pushbutton3modal,'Enable','On');
end


% --- Executes on button press in Main_File_checkbox4staer.
function Main_File_checkbox4staer_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkbox4staer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1

% Check if box for staer has been selected or not
global enabled_codes;
enabled_codes(3) = get(hObject,'Value');

if (enabled_codes(3) == 0)
    set(handles.Main_Run_pushbutton4staer,'Enable','Off');
else
    set(handles.Main_Run_pushbutton4staer,'Enable','On');
end


% --- Executes on button press in Main_File_checkbox5flutter.
function Main_File_checkbox5flutter_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkbox5flutter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1

% Check if box for flutter has been selected or not
global enabled_codes;
enabled_codes(4) = get(hObject,'Value');

if (enabled_codes(4) == 0)
    set(handles.Main_Run_pushbutton5flutter,'Enable','Off');
else
    set(handles.Main_Run_pushbutton5flutter,'Enable','On');
end

% --- Executes on button press in Main_File_VLM_Callback.
%function Main_File_VLM_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_VLM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Main_File_checkbox1


% --- Executes on button press in Main_File_checkboxVLM.
%function Main_File_checkboxVLM_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_checkboxVLM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% main_Structure_checkbox1

% Check if box for VLM has been selected or not
%global enabled_codes;
%enabled_codes(7) = get(hObject,'Value');
%
%if (enabled_codes(7) == 0)
%    set(handles.Main_Run_pushbutton6vlm,'Enable','Off');
%else
%    set(handles.Main_Run_pushbutton6vlm,'Enable','On');
%end


function Main_Settings_edit1_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Settings_edit1 as text
%        str2double(get(hObject,'String')) returns contents of Main_Settings_edit1 as a double


% --- Executes during object creation, after setting all properties.
function Main_Settings_edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function Main_Settings_editniter_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editniter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Settings_edit1 as text
%        str2double(get(hObject,'String')) returns contents of Main_Settings_edit1 as a double
global beam_model;

beam_model.Param.NITER = floor(abs(str2double(get(hObject,'String'))));
fprintf(1, '\n - Sub-iteration for non-linear solver: %d. \n', beam_model.Param.NITER);


% --- Executes during object creation, after setting all properties.
function Main_Settings_editniter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editniter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function Main_Settings_editnstep_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editnstep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Settings_edit1 as text
%        str2double(get(hObject,'String')) returns contents of Main_Settings_edit1 as a double
global beam_model;

beam_model.Param.NSTEP = floor(abs(str2double(get(hObject,'String'))));

fprintf(1, '\n - Number of aeroelastic/load iterations: %d. \n', beam_model.Param.NSTEP);


% --- Executes during object creation, after setting all properties.
function Main_Settings_editnstep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editnstep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function Main_Settings_editrestol_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editrestol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Settings_edit1 as text
%        str2double(get(hObject,'String')) returns contents of Main_Settings_edit1 as a double

global beam_model;

beam_model.Param.RES_TOL = abs(str2double(get(hObject,'String')));
fprintf(1, '\n - Convergence tolerance for non-linear solver: %g. \n', beam_model.Param.RES_TOL);


% --- Executes during object creation, after setting all properties.
function Main_Settings_editrestol_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editrestol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function Main_Settings_editresfac_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editresfac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Settings_edit1 as text
%        str2double(get(hObject,'String')) returns contents of Main_Settings_edit1 as a double

global beam_model;

beam_model.Param.RES_FAC = abs(str2double(get(hObject,'String')));
fprintf(1, '\n - Number of aeroelastic iterations or load steps: %d. \n', beam_model.Param.RES_FAC);


% --- Executes during object creation, after setting all properties.
function Main_Settings_editresfac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_editresfac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



% --- Executes on selection change in Main_Settings_popupmenu1guess.
function Main_Settings_popupmenu1guess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_popupmenu1guess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Main_Settings_popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Main_Settings_popupmenu1
contents  = get(hObject,'String');
selection = contents{get(hObject,'Value')};
% selection

% --- Executes during object creation, after setting all properties.
function Main_Settings_popupmenu1guess_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_popupmenu1guess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on selection change in Main_Settings_popupmenu2str.
function Main_Settings_popupmenu2str_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_popupmenu2str (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns Main_Settings_popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Main_Settings_popupmenu1

% Selection of structural model to be adopted (3 = equivalent plate)
Structural_model = get(hObject,'Value');

if (Structural_model == 1)
    set(handles.Main_Settings_edit1,'Value',1);
    set(handles.Main_Settings_edit1,'Enable','Off');
    set(handles.Main_Settings_text3,'Enable','Off');
    set(handles.Main_Settings_text2,'Enable','Off');
    set(handles.Main_Settings_text4,'Enable','Off');
    set(handles.Main_Settings_text5,'Enable','Off');
    set(handles.Main_Settings_editniter,'Enable','Off');
    set(handles.Main_Settings_editnstep,'Enable','Off');
    set(handles.Main_Settings_editrestol,'Enable','Off');
end
if (Structural_model == 2)
    set(handles.Main_Settings_edit1,'Value',1);
    set(handles.Main_Settings_edit1,'Enable','Off');
    set(handles.Main_Settings_text3,'Enable','Off');
    set(handles.Main_Settings_text2,'Enable','On');
    set(handles.Main_Settings_text4,'Enable','On');
    set(handles.Main_Settings_text5,'Enable','On');
    set(handles.Main_Settings_editniter,'Enable','On');
    set(handles.Main_Settings_editnstep,'Enable','On');
    set(handles.Main_Settings_editrestol,'Enable','On');
end
if (Structural_model == 3)
    set(handles.Main_Settings_edit1,'Enable','On');
    set(handles.Main_Settings_text3,'Enable','On');
    set(handles.Main_Settings_edit1,'Value',1);
    set(handles.Main_Settings_text2,'Enable','Off');
    set(handles.Main_Settings_text4,'Enable','Off');
    set(handles.Main_Settings_text5,'Enable','Off');
    set(handles.Main_Settings_editniter,'Enable','Off');
    set(handles.Main_Settings_editnstep,'Enable','Off');
    set(handles.Main_Settings_editrestol,'Enable','Off');
end
if (Structural_model == 4)
    set(handles.Main_Settings_edit1,'Value',1);
    set(handles.Main_Settings_edit1,'Enable','Off');
    set(handles.Main_Settings_text3,'Enable','Off');
    set(handles.Main_Settings_text2,'Enable','Off');
    set(handles.Main_Settings_text4,'Enable','Off');
    set(handles.Main_Settings_text5,'Enable','Off');
    set(handles.Main_Settings_editniter,'Enable','Off');
    set(handles.Main_Settings_editnstep,'Enable','On');
    set(handles.Main_Settings_editrestol,'Enable','Off');
end

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.solver.param.MODEL_TYPE = Structural_model;

switch(Structural_model)
    
    case 1
        fprintf(1, '\n - Model type chosen: linear beam model. \n');
    case 2
        fprintf(1, '\n - Model type chosen: non-linear beam model. \n');
    case 3
        fprintf(1, '\n - Model type chosen: linear equivalent plate model. \n');
    case 4
        fprintf(1, '\n - Model type chosen: linear beam with Updated Lagrangian iteration. \n');
end

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function Main_Settings_popupmenu2str_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Settings_popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on button press in Main_Settings_checkboxinter.
function Main_Settings_checkbox1inter_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_checkboxinter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
%
interstatus = get(hObject,'Value');
autostatus = get(handles.Main_Settings_checkbox2auto,'Value');
if interstatus,
    set(handles.Main_Settings_checkbox2auto,'Value',0);
    autostatus = 0;
    set(handles.Main_Run_pushbutton6start,'Enable','Off');
end
interstatus
autostatus

% --- Executes on button press in Main_Settings_checkbox2auto.
function Main_Settings_checkbox2auto_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Settings_checkbox2auto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
%
% If the automatic checkbox is selected all enabled_codes are started
% pressing start button
global enabled_codes;

autostatus = get(hObject,'Value');
interstatus = get(handles.Main_Settings_checkbox1inter,'Value');
if autostatus,
    set(handles.Main_Settings_checkbox1inter,'Value',0);
    interstatus = 0;
    set(handles.Main_Run_pushbutton6start,'Enable','On');
else
    set(handles.Main_Run_pushbutton6start,'Enable','Off');
end

% Start all enabled codes

interstatus
autostatus

% --- Executes on button press in Main_Run_pushbutton1guess.
function Main_Run_pushbutton1guess_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton1guess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp 'RUN GUESS'

% --- Executes on button press in Main_Run_pushbutton2static.
function Main_Run_pushbutton2static_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton2static (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% 101 or 600 solvers
%
global enabled_codes;

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

STR_MODEL = gui_param.solver.param.MODEL_TYPE;

if enabled_codes(1)
    
    switch(STR_MODEL)
        
        case 1
            
            solve_lin_static;
            
        case 2
            
            solve_nlin_static;
            
        case 3
            
            fprintf(1, '\n - Plate model not available yet. \n');
            
        case 4
            
            solve_up_lagr_static;
            
    end
    
end

% --- Executes on button press in Main_Run_pushbutton3modal.
function Main_Run_pushbutton3modal_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton3modal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global enabled_codes;

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

STR_MODEL = gui_param.solver.param.MODEL_TYPE;

if enabled_codes(2)
    
    switch(STR_MODEL)
        
        case {1,2}
            
            solve_eig;
            
        case 3
            
            fprintf(1, '\n - Plate model not available yet. \n');
            
    end
    
end

% --- Executes on button press in Main_Run_pushbutton4staer.
function Main_Run_pushbutton4staer_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton4staer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global enabled_codes;

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

STR_MODEL = gui_param.solver.param.MODEL_TYPE;

if enabled_codes(3)
    
    trimI = ManeuverSelect;
    pause(1);
    if isempty(trimI)
        fprintf(1, 'Warning: an empty trim case array was selected. Please, select no input or a no empty input.\n');
    else
        
        switch(STR_MODEL)
            
            case 1
                
                solve_free_lin_trim(trimI);
                
            case 2
                
                solve_nlin_aerostatic;
                
            case 3
                
                fprintf(1, '\n - Plate model not available yet. \n');
                
            case 4
                
                solve_free_up_lagr_trim(trimI);
                
        end
    end
    
end

% --- Executes on button press in Main_Run_pushbutton5flutter.
function Main_Run_pushbutton5flutter_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton5flutter(see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global enabled_codes;

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

STR_MODEL = gui_param.solver.param.MODEL_TYPE;

if enabled_codes(4)
    
    switch(STR_MODEL)
        
        case {1,2}
            
            solve_linflutt;
            
        case 3
            
            fprintf(1, '\n - Plate model not available yet. \n');
            
    end
    
end

% --- Executes on button press in Main_Run_pushbutton6vlm.
function Main_Run_pushbutton6vlm_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton6vlm(see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global enabled_codes;

enabled_codes;


if enabled_codes(7)
    solve_vlm_rigid;
elseif enabled_codes(8)
    solve_dynder;
end

% --- Executes on button press in Main_Run_pushbutton6start.
function Main_Run_pushbutton6start_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Run_pushbutton6start(see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(1, '\n - Functionality not available yet. \n');

% --- Executes on button press in Main_Results_plotmodel.
function Main_Results_plotmodel_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_plotmodel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global beam_model;

try
    beam_model.Info.ncaero;
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    if (beam_model.Info.ncaero)
        aero_value = [gui_param.plot.WAKE_PLOT gui_param.plot.NORM_PLOT 0];
        plot_beam_model(1, 'aero_param', aero_value);
    else
        plot_beam_model(1);
    end
catch
    fprintf(1, '\n - No beam model loaded. \n');
end

% --- Executes on button press in Main_Results_plotdefo.
function Main_Results_plotdefo_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_plotdefo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global beam_model;

try
    beam_model.Res.SOL;
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    if (beam_model.Info.ncaero)
        aero_value = [gui_param.plot.WAKE_PLOT gui_param.plot.NORM_PLOT gui_param.plot.CONT_PLOT];
        
        plot_beam_defo(2, gui_param.plot.SCALE, 'aero_param', aero_value, 'set', gui_param.plot.SET);
        
    else
        plot_beam_defo(2, gui_param.plot.SCALE, 'set', gui_param.plot.SET);
    end
    
catch
    fprintf(1, '\n - No output results available. \n');
end


% --- Executes on button press in Main_Results_plotmodes.
function Main_Results_plotmodes_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_plotmodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global beam_model;

try
    beam_model.Res.SOL;
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    QUART_FRAMES = floor(gui_param.plot.NFRAMES/4);
    scale1 = [0:QUART_FRAMES];
    scale2 = [QUART_FRAMES-1:-1:1];
    scale3 = [0:-1:-QUART_FRAMES];
    scale4 = [-QUART_FRAMES+1:0];
    scale = (gui_param.plot.SCALE/QUART_FRAMES) .* [scale1, scale2, scale3, scale4];
    
    if (beam_model.Info.ncaero)
        aero_value = [gui_param.plot.WAKE_PLOT gui_param.plot.NORM_PLOT gui_param.plot.CONT_PLOT];
        
        animate_beam_modes(gui_param.plot.SET, scale, 'aero_param', aero_value);
    else
        animate_beam_modes(gui_param.plot.SET, scale);
    end
    
catch
    fprintf(1, '\n - No output results available. \n');
end

% --- Executes on button press in Main_Results_saveall.
function Main_Results_saveall_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_saveall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
global smartcad_pathname

if ~smartcad_pathname
    smartcad_pathname = neocass_path;
end

formats = { '*.mat', 'NeoCASS project file (*.mat)';...
    '*.*', 'All files (*.*)'};

[filename, path, filterindex] = uiputfile(formats, 'Save NeoCASS file', smartcad_pathname);

smartcad_pathname = path;


if (filterindex ~= 0)
    
    neocasspath = neoguiscratchpath(); gui_param = load(fullfile(neocasspath, 'neocass_gui_param.mat'));
    
    global guess_model;
    global beam_model;
    global dlm_model;
    global fl_model;
    
    filename = uiputext(filename, filterindex, formats);
    
    fprintf(1, '\n - Saving NeoCASS database to %s file...', [path,filename]);
    save(fullfile(path, filename), 'gui_param', 'guess_model', 'beam_model', 'dlm_model', 'fl_model');
    fprintf(1, 'done. \n');
    
end

% --- Executes on button press in Main_Results_checkboxpcont.
function Main_Results_checkboxpcont_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_checkboxpcont (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Main_Results_checkboxpcont

value = get(hObject,'Value');

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.plot.CONT_PLOT = value;
save_gui_params('neocass_gui_param.mat', gui_param);

if (value)
    fprintf(1, '\n - Contour plot enabled. \n');
else
    fprintf(1, '\n - Contour plot disabled. \n');
end

% --- Executes on button press in Main_Results_checkboxpnorms.
function Main_Results_checkboxpnorms_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_checkboxpnorms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Main_Results_checkboxpnorms

value = get(hObject,'Value');

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.plot.NORM_PLOT = value;

save_gui_params('neocass_gui_param.mat', gui_param);

if (value)
    fprintf(1, '\n - Normals plot enabled. \n');
else
    fprintf(1, '\n - Normals plot disabled. \n');
end

% --- Executes on button press in Main_Results_checkboxwake.
function Main_Results_checkboxwake_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_checkboxwake (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Main_Results_checkboxwake

value = get(hObject,'Value');

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.plot.WAKE_PLOT = value;

save_gui_params('neocass_gui_param.mat', gui_param);

if (value)
    fprintf(1, '\n - Wake plot enabled. \n');
else
    fprintf(1, '\n - Wake plot disabled. \n');
end



function Main_Results_editselset_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_editselset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Results_editselset as text
%        str2double(get(hObject,'String')) returns contents of Main_Results_editselset as a double

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
value = int32(abs(str2double(get(hObject,'String'))));
if value == 0
    fprintf(1, '\n - Null set index given. Value set to 1 by default. \n');
    gui_param.plot.SET = int32(1);
else
    gui_param.plot.SET = value;
end
fprintf(1, '\n - Output set selected: %d. \n', gui_param.plot.SET);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function Main_Results_editselset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Results_editselset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Main_Results_editsetscale_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_editsetscale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Results_editsetscale as text
%        str2double(get(hObject,'String')) returns contents of Main_Results_editsetscale as a double

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
value = abs(str2double(get(hObject,'String')));
if value == 0
    fprintf(1, '\n - Null scale factor given. Value set to 1.0 by default. \n');
    gui_param.plot.SCALE = 1.0;
else
    gui_param.plot.SCALE = value;
end
fprintf(1, '\n - Scale factor: %g. \n', gui_param.plot.SCALE);
save_gui_params('neocass_gui_param.mat', gui_param);

% --- Executes during object creation, after setting all properties.
function Main_Results_editsetscale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Results_editsetscale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Main_Results_editnframes_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_editnframes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Results_editnframes as text
%        str2double(get(hObject,'String')) returns contents of Main_Results_editnframes as a double

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
value = abs(str2double(get(hObject,'String')));
if value == 0
    fprintf(1, '\n - Null total frames. Value set to 10 by default. \n');
    gui_param.plot.NFRAMES = 10;
else
    gui_param.plot.NFRAMES = value;
end
fprintf(1, '\n - Total frames: %d. \n', gui_param.plot.NFRAMES);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function Main_Results_editnframes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Results_editnframes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Main_Results_pushbuttonplotguessres.
function Main_Results_pushbuttonplotguessres_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_pushbuttonplotguessres (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global guess_model;

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
set = gui_param.plot.SET;

try
    guess_model.geo;
    plot_guess_model(set, 2, guess_model);
catch
    fprintf(1,'\n - GUESS output set not available. \n');
end


% --- Executes on button press in Main_File_pushbuttonloadall.
function Main_File_pushbuttonloadall_Callback(hObject, eventdata, handles)
% hObject    handle to Main_File_pushbuttonloadall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, path, filterindex] = uigetfile('*.mat', 'Select NeoCASS file');

if (filterindex ~= 0)
    
    saveall = load(fullfile(path, filename));
    
    init_gui_params('neocass_gui_param.mat');
    neocass_init;
    
    fprintf(1, '\n - Loading NeoCASS database from %s file...', [path,filename]);
    gui_param = saveall.gui_param;
    save_gui_params('neocass_gui_param.mat', gui_param);
    
    guess_model = saveall.guess_model;
    beam_model = saveall.beam_model;
    dlm_model = saveall.dlm_model;
    fl_model = saveall.fl_model;
    
    enable_solver(handles);
    %
    fprintf(1, 'done. \n');
    
end



function Main_Results_editcol_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_editcol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Results_editcol as text
%        str2double(get(hObject,'String')) returns contents of Main_Results_editcol as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.plot.COL = int32(str2double(get(hObject,'String')));
fprintf(1, '\n - Transfer matrix column: %d \n', gui_param.plot.COL);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function Main_Results_editcol_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Results_editcol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Main_Results_editrow_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_editrow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Main_Results_editrow as text
%        str2double(get(hObject,'String')) returns contents of Main_Results_editrow as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.plot.ROW = int32(str2double(get(hObject,'String')));
fprintf(1, '\n - Transfer matrix row: %d \n', gui_param.plot.ROW);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function Main_Results_editrow_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Main_Results_editrow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Main_Results_pushbuttonplotqhh.
function Main_Results_pushbuttonplotqhh_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_pushbuttonplotqhh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global dlm_model;

try
    dlm_model.data;
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    row = gui_param.plot.ROW;
    col = gui_param.plot.COL;
    set = gui_param.plot.SET;
    
    plot_dlm_qhh(dlm_model, row, col, set, 2);
    
catch
    fprintf(1, '\n - No Aerodynamic transfer matrix available. \n');
    
end

% --- Executes on button press in Main_Results_pushbuttonplotflutter.
function Main_Results_plotflutter_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_pushbuttonplotqhh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fl_model;
global beam_model;

try
    fl_model.Res;
    beam_model.Aero.state.Mach;
    beam_model.Res.Omega;
    path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
    set = gui_param.plot.SET;
    plot_vg_diags(2, fl_model.Res, set, beam_model.Res.Omega./(2*pi), beam_model.Aero.state.Mach, fl_model.param.linestyle);
    
catch
    fprintf(1, '\n - No flutter result available. \n');
    
end

% --- Executes on button press in Main_Results_pushbuttonplotflutter.
function Main_Results_closeall_Callback(hObject, eventdata, handles)
% hObject    handle to Main_Results_pushbuttonplotqhh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fprintf(1,'\n - Cleaning NeoCASS variables...');
clear global beam_model;
clear global guess_model;
clear global fl_model;
clear global dlm_model;
fprintf(1,'done. \n');

fprintf(1,'\n - Cleaning NeoCASS GUI scratch file...');
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

if isunix
    command = ['rm -f ', path, 'neocass_gui_param.mat'];
else
    command = ['del ', path, 'neocass_gui_param.mat'];
end
system(command);
fprintf(1,'done. \n');

close all;



function close_gui(hObject, eventdata)
%
fprintf(1,'\n - Closing NeoCASS GUI...');
global enabled_gui
enabled_gui = false;
delete(hObject);
delneoguifile;
fprintf(1,'done.\n\n');


% LR 27/11/2014
%
% tabpanel Callback copied here and updated to support Matlab version
% higher than 8.3 (new graphic environment)
%
function Main_Callback(hObject, eventdata, handles)


% Recover handles related to 'Main'
tpchandles = guihandles(gcbo);
if verLessThan('matlab', '8.4')
    tpctab  = get(tpchandles.(get(gcbo, 'Tag'))(end-2), 'UserData');
else
    tmptab  = tpchandles.figure1.Children.findobj('type', 'uipanel');
    tmptab2 = findobj(tmptab, 'title', 'Main');
    tpctab  = tmptab2.UserData;
end

% Current tab (old one)
tpchtab = findobj(tpchandles.(tpctab.tag), 'string', tpctab.current);
tpcpos  = get(tpchtab, 'position');
set(tpchtab,'Enable','on','Fontweight','normal','position',[tpcpos(1:3) tpctab.height],'BackGroundColor',tpctab.BackColor,'ForeGroundColor',tpctab.ForeColor);

% New tab (clicked one)
tpcpos = get(gcbo, 'position');
set(gcbo,'Enable','inactive','Fontweight','bold','Position',tpcpos+[0 0 0 tpctab.outbreak],'BackGroundColor',tpctab.CurrBackColor,'ForeGroundColor',tpctab.CurrForeColor);

% Set "hide into the back" option
set(findobj(tpchandles.(tpctab.tag), 'String', 'backhide'), 'position', [tpcpos(1:3) + [1 0 -3] 2]);

tpcvisoff = [];
for tpci = unique(get(tpchtab,'UserData'));
    tpcvisoff = [tpcvisoff getfield(tpchandles,char(tpci))];
end

tpcvison = [];
for tpci = unique(get(gcbo,'UserData'));
    tpcvison = [tpcvison getfield(tpchandles,char(tpci))];
end

set(tpcvisoff, 'Visible', 'off');
set(tpcvison, 'Visible', 'on');

% Refresh figure
drawnow;

% Make new tab the current one
tpctab.current = get(gcbo,'String');

% Refresh user data
ActivePanel = findobj(tpchandles.(tpctab.tag), 'type', 'uipanel');
set(ActivePanel, 'UserData', tpctab);

