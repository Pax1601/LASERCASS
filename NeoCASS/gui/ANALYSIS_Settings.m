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

function varargout = ANALYSIS_Settings(varargin)
% ANALYSIS_SETTINGS M-file for ANALYSIS_Settings.fig
%      ANALYSIS_SETTINGS, by itself, creates a new ANALYSIS_SETTINGS or raises the existing
%      singleton*.
%
%      H = ANALYSIS_SETTINGS returns the handle to a new ANALYSIS_SETTINGS or the handle to
%      the existing singleton*.
%
%      ANALYSIS_SETTINGS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANALYSIS_SETTINGS.M with the given input arguments.
%
%      ANALYSIS_SETTINGS('Property','Value',...) creates a new ANALYSIS_SETTINGS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ANALYSIS_Settings_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ANALYSIS_Settings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2002-2003 The MathWorks, Inc.

% Edit the above text to modify the response to help ANALYSIS_Settings

% Last Modified by GUIDE v2.5 16-Feb-2010 18:28:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ANALYSIS_Settings_OpeningFcn, ...
                   'gui_OutputFcn',  @ANALYSIS_Settings_OutputFcn, ...
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
 

% --- Executes just before ANALYSIS_Settings is made visible.
function ANALYSIS_Settings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ANALYSIS_Settings (see VARARGIN)

% Choose default command line output for ANALYSIS_Settings
handles.output = hObject;

% Update handles structure
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
save_gui_params('neocass_gui_param_backup.mat', gui_param);
% UIWAIT makes ANALYSIS_Settings wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%--------------- Beginning of ANALYSIS_Settings parameters definition
%
% Switch off POPUPMENUID and DOF related to POINT normalization
trim = false; modal = false; Vg_plot = false; Vg_env = false;

if gui_param.solver.STATIC_AERO_AV
    trim = true;
else
    if gui_param.solver.FLUTT_ENV
        Vg_env = true;
    else
        if gui_param.solver.FLUTT_AV
            Vg_plot = true;
        else
            modal = true;
        end
    end
end

analysis_select(hObject, trim, modal, Vg_plot, Vg_env)

if trim
    if gui_param.solver.aeros.CS
        set(handles.setCS,'Enable','Off');
        if str2double(get(handles.nstates,'String')) <= 0
            set(handles.pushbuttonselectstates,'Enable','Off');
        end
    else
        set(handles.pushbuttonselectstates,'Enable','Off');
    end
end

% Recovers number of flight conditions from states.xml files (now set equal to 12)
%path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
%eval(command);
%nstates = gui_param.solver.aeros.MAXSTATES;

init_settings(handles, gui_param)


guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = ANALYSIS_Settings_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function lmodes_Callback(hObject, eventdata, handles)
% hObject    handle to lmodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lmodes as text
%        str2double(get(hObject,'String')) returns contents of lmodes as a double
%
% Input number of modes for analysis (lmodes)

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

lmodes = str2double(get(hObject,'String'));

if rem(lmodes,round(lmodes))~=0 || lmodes<=0,
    warning('Analysis:Settings','LMODES must be an integer > 0.');
    lmodes = abs(round(lmodes));
    set(hObject,'String',num2str(lmodes));
end

gui_param.solver.eig.NROOTS = lmodes;

if lmodes > 0 && ( get(handles.radiovgplot,'Value') || get(handles.radioenvelope,'Value') ),
    set(handles.text15_modalbase,'Enable','On');
    set(handles.pushbutton4_mselect,'Enable','On');
    set(handles.text16_tracking,'Enable','On');
    set(handles.pushbutton5_fmodes,'Enable','On');
else
    set(handles.text15_modalbase,'Enable','Off');
    set(handles.pushbutton4_mselect,'Enable','Off');
    set(handles.text16_tracking,'Enable','Off');
    set(handles.pushbutton5_fmodes,'Enable','Off');
end

fprintf(1, '\n - Number of Modes: %d. \n', gui_param.solver.eig.NROOTS);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function lmodes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lmodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
global beam_model

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

if isfield(beam_model, 'Res') && isfield(beam_model.Res, 'Omega')
    lomega = length(beam_model.Res.Omega);
    set(hObject, 'String', num2str(lomega));
    gui_param.solver.eig.NROOTS = lomega;
    save_gui_params('neocass_gui_param.mat', gui_param);
else
    if ~isempty(gui_param.solver.eig.NROOTS)
        set(hObject, 'String', num2str(gui_param.solver.eig.NROOTS));
    end
end

if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function minf_Callback(hObject, eventdata, handles)
% hObject    handle to minf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of minf as text
%        str2double(get(hObject,'String')) returns contents of minf as a double
%
% Input lower limit for modes to be taken into account (minf)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
minf = str2double(get(hObject,'String'));

gui_param.solver.eig.MINF = abs(minf);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function minf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to minf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function msup_Callback(hObject, eventdata, handles)
% hObject    handle to msup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of msup as text
%        str2double(get(hObject,'String')) returns contents of msup as a double
%
% Input upper limit for modes to be taken into account (msup)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
maxf = str2double(get(hObject,'String'));

gui_param.solver.eig.MAXF = abs(maxf);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function msup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to msup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function popupmenuid_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of popupmenuid as text
%        str2double(get(hObject,'String')) returns contents of popupmenuid as a double
%
% Read Grid POPUPMENUID in case of POINT normalisation
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
ID = str2double(get(hObject,'String'));

gui_param.solver.eig.REF_GRID = abs(ID);
gui_param.solver.eig.NORM_MET = 3;

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function popupmenuid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in popupmenurm.
function popupmenurm_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenurm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenurm contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenurm
%
% Select normalization for modal analysis
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
contents = get(hObject,'String');
norm = contents{get(hObject,'Value')};

switch(norm)

  case '3 POINT'

    fprintf(1, '\n - Normalization method: POINT. \n');
    gui_param.solver.eig.NORM_MET = 3;
    set(handles.popupmenuid,'Enable','On');
    set(handles.popupmenudof,'Enable','On');
    set(handles.text5,'Enable','On');

  case '2 MAX'

    fprintf(1, '\n - Normalization method: MAX. \n');
    gui_param.solver.eig.NORM_MET = 2;
    set(handles.popupmenuid,'Enable','Off');
    set(handles.popupmenudof,'Enable','Off');
    set(handles.text5,'Enable','Off');

  case '1 MASS'

    fprintf(1, '\n - Normalization method: MASS. \n');
    gui_param.solver.eig.NORM_MET = 1;
    set(handles.popupmenuid,'Enable','Off');
    set(handles.popupmenudof,'Enable','Off');
    set(handles.text5,'Enable','Off');

end

save_gui_params('neocass_gui_param.mat', gui_param);



% --- Executes during object creation, after setting all properties.
function popupmenurm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenurm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in popupmenudof.
function popupmenudof_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenudof (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenudof contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenudof
%
% Input DOF in case of POINT normalisation
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
contents = get(hObject,'String');

DOF = str2num(strtok(contents{get(hObject,'Value')}));

gui_param.solver.eig.REF_GRID_C = abs(DOF);
gui_param.solver.eig.NORM_MET = 3;

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function popupmenudof_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenudof (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end





% --- Executes on button press in radiobuttontrim.
function radiobuttontrim_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttontrim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttontrim
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

    gui_param.solver.EIG_AV = false;
    gui_param.solver.FLUTT_AV = false;
    gui_param.solver.STATIC_AERO_AV = true;
    gui_param.solver.FLUTT_ENV = false;
    analysis_select(hObject, true, false, false, false)
    
    if gui_param.solver.aeros.CS
        set(handles.setCS,'Enable','Off');
        if str2double(get(handles.nstates,'String')) <= 0
            set(handles.pushbuttonselectstates,'Enable','Off');
        end
    else
        set(handles.pushbuttonselectstates,'Enable','Off');
    end

save_gui_params('neocass_gui_param.mat', gui_param);
    

function nfreq_Callback(hObject, eventdata, handles)
% hObject    handle to nfreq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nfreq as text
%        str2double(get(hObject,'String')) returns contents of nfreq as a double
%
% Input number of reduced frequencies nfreq (max 12)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
maxfreq = gui_param.solver.flutter.MAXRFREQ;
nfreq = str2double(get(hObject,'String'));

if (nfreq > maxfreq)
    gui_param.solver.flutter.NRFreq = maxfreq;
else
  gui_param.solver.flutter.NRFreq = abs(nfreq);
end

if nfreq > 0 && ( get(handles.radiovgplot,'Value') || get(handles.radioenvelope,'Value') ),
    set(handles.pushbuttoninsertfreq,'Enable','On');
else
    set(handles.pushbuttoninsertfreq,'Enable','Off');
end

fprintf(1, '\n - Number of reduced frequencies: %d. \n', gui_param.solver.flutter.NRFreq);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function nfreq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nfreq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in pushbuttoninsertfreq.
function pushbuttoninsertfreq_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttoninsertfreq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Input reduced frequencies

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
nfreq = gui_param.solver.flutter.NRFreq;

if nfreq > 0

  freq = zeros(1, nfreq);
  freq(1,1) = 0.001;

  klist = {};
  for n= 1:nfreq
    klist{n} = ['k', num2str(n)]; 
  end

  freqs = tableGUI('FigName','Input Reduced Frequencies','array', freq,'ColNames', klist,'NumRows',1);
  values = str2double(freqs); % values contain reduced frequencies values

  index = find(values);
  values = values(index);
  gui_param.solver.flutter.RFreq_list = unique(abs(values));
  gui_param.solver.flutter.NRFreq = length(gui_param.solver.flutter.RFreq_list);

  if gui_param.solver.flutter.NRFreq

    fprintf(1, '\n - Reduced frequencies:');

    for n=1:gui_param.solver.flutter.NRFreq
      fprintf(1, ' %g', gui_param.solver.flutter.RFreq_list(n));
    end
    fprintf(1, '. \n');

  else
    fprintf(1, '\n - No reduced frequency given. \n');
    gui_param.solver.flutter.RFreq_list = [];
    gui_param.solver.flutter.NRFreq = 0;
  end

  save_gui_params('neocass_gui_param.mat', gui_param);

end

% --- Executes on button press in radiovgplot.
function radiovgplot_Callback(hObject, eventdata, handles)
% hObject    handle to radiovgplot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiovgplot
%
% Check if V-g plot is selected, if yes deactivates Flutter Envelope
% Controls

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

    gui_param.solver.EIG_AV = true;
    gui_param.solver.FLUTT_AV = true;
    gui_param.solver.STATIC_AERO_AV = false;
    gui_param.solver.FLUTT_ENV = false;
    analysis_select(hObject, false, false, true, false)

if isnan(str2double(get(handles.lmodes,'String'))) || str2double(get(handles.lmodes,'String')) == 0,
    set(handles.text15_modalbase,'Enable','Off');
    set(handles.pushbutton4_mselect,'Enable','Off');
    set(handles.text16_tracking,'Enable','Off');
    set(handles.pushbutton5_fmodes,'Enable','Off');
end

if isnan(str2double(get(handles.nfreq,'String'))) || str2double(get(handles.nfreq,'String')) <= 0,
    set(handles.pushbuttoninsertfreq,'Enable','Off');
end
    
    set(handles.popupmenurm,'Value',1);
    
save_gui_params('neocass_gui_param.mat', gui_param);


function vmax_Callback(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vmax as text
%        str2double(get(hObject,'String')) returns contents of vmax as a double
%
% Input Vmax for V-g plot
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
vmax = str2double(get(hObject,'String'));
gui_param.solver.flutter.VMAX = abs(vmax);
fprintf(1, '\n - Maximum velocity: %g. \n', gui_param.solver.flutter.VMAX);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function vmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function vstep_Callback(hObject, eventdata, handles)
% hObject    handle to vstep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vstep as text
%        str2double(get(hObject,'String')) returns contents of vstep as a double
%
% Input max number of V step
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
vstep = str2double(get(hObject,'String'));
gui_param.solver.flutter.NV = abs(vstep);
fprintf(1, '\n - Velocity points: %g. \n', gui_param.solver.flutter.NV);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function vstep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vstep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function rho_Callback(hObject, eventdata, handles)
% hObject    handle to rho (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rho as text
%        str2double(get(hObject,'String')) returns contents of rho as a double
%
% Input density for V-g plot
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
rho = str2double(get(hObject,'String'));
gui_param.solver.flutter.RHO = abs(rho);
fprintf(1, '\n - Flight density: %g. \n', gui_param.solver.flutter.RHO);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function rho_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rho (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function mach_Callback(hObject, eventdata, handles)
% hObject    handle to mach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mach as text
%        str2double(get(hObject,'String')) returns contents of mach as a double
%
% Input Mach number for V-g plot
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
get(hObject,'String');
mach = str2double(get(hObject,'String'));

gui_param.solver.flutter.Mach_list = abs(mach);
gui_param.solver.flutter.NMach = length(gui_param.solver.flutter.Mach_list);

fprintf(1, '\n - Mach number: %g. \n', gui_param.solver.flutter.Mach_list(1));

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function mach_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in radioenvelope.
function radioenvelope_Callback(hObject, eventdata, handles)
% hObject    handle to radioenvelope (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radioenvelope
%
% Check if Flutter envelope is selected, if yes deactivates V-g plot
% Controls

path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

    gui_param.solver.EIG_AV = true;
    gui_param.solver.FLUTT_AV = true;
    gui_param.solver.FLUTT_ENV = true;
    gui_param.solver.STATIC_AERO_AV = false;
    gui_param.solver.flutter.RHO = 1.225;
    analysis_select(hObject, false, false, false, true)
    
if isnan(str2double(get(handles.lmodes,'String'))) || str2double(get(handles.lmodes,'String')) == 0,
    set(handles.text15_modalbase,'Enable','Off');
    set(handles.pushbutton4_mselect,'Enable','Off');
    set(handles.text16_tracking,'Enable','Off');
    set(handles.pushbutton5_fmodes,'Enable','Off');
end

if isnan(str2double(get(handles.nfreq,'String'))) || str2double(get(handles.nfreq,'String')) <= 0,
    set(handles.pushbuttoninsertfreq,'Enable','Off');
end

if isnan(str2double(get(handles.nmach,'String'))) || str2double(get(handles.nmach,'String')) <= 0,
    set(handles.pushbuttoninsertmach,'Enable','Off');
end
    
    set(handles.popupmenurm,'Value',1);
    
save_gui_params('neocass_gui_param.mat', gui_param);



function nmach_Callback(hObject, eventdata, handles)
% hObject    handle to nmach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nmach as text
%        str2double(get(hObject,'String')) returns contents of nmach as a double
%
% Input number of Mach numbers for flutter envelope
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

get(hObject,'String');
nmach = str2double(get(hObject,'String'));

gui_param.solver.flutter.NMach = nmach;

if nmach > 0 && get(handles.radioenvelope,'Value'),
    set(handles.pushbuttoninsertmach,'Enable','On');
else
    set(handles.pushbuttoninsertmach,'Enable','Off');
end

fprintf(1, '\n - Number of Mach numbers: %d. \n', gui_param.solver.flutter.NMach);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function nmach_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nmach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in pushbuttoninsertmach.
function pushbuttoninsertmach_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttoninsertmach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% input Mach values for flutter envelope
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

nmach = gui_param.solver.flutter.NMach ;

if (nmach)

  machs = zeros(1, nmach);

  labels = {};

  for n=1:nmach
    labels{n} = ['M', num2str(n)];
  end

  mach_values = tableGUI('FigName','Input Mach Values','array', machs, 'ColNames', labels,'NumRows',1);
  values = str2double(mach_values); % values contain Mach values

  index = find(values);
  values = values(index);
  index = find(values <= 1.0); % delete supersonic Mach numbers
  values = values(index);
  gui_param.solver.flutter.Mach_list = unique(abs(values));
  gui_param.solver.flutter.NMach = length(gui_param.solver.flutter.Mach_list);

  if gui_param.solver.flutter.NMach

    fprintf(1, '\n - Mach number:');
    for n=1:gui_param.solver.flutter.NMach
      fprintf(1, ' %g', gui_param.solver.flutter.Mach_list(n));
    end
    fprintf(1, '. \n');

  else
    fprintf(1, '\n - No Mach number given. \n');
    gui_param.solver.flutter.Mach_list = [];
    gui_param.solver.flutter.NMach = 0;
  end

  save_gui_params('neocass_gui_param.mat', gui_param);

end


% --- Executes on button press in pushbutton4_mselect.
function pushbutton4_mselect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4_mselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

lmod = str2double(get(handles.lmodes,'String'));
modelist = cell(lmod,1);
global beam_model
if isfield(beam_model, 'Res'),
    if isfield(beam_model.Res, 'Omega'),
        omega = beam_model.Res.Omega/2/pi;
        lomega = length(omega);
    end
end

for i = 1:lmod,
    if ~exist('omega', 'var') || i > lomega,
        modelist{i} = ['mode ', num2str(i, '%02d'), '         '];
    else
        modelist{i} = ['mode ', num2str(i, '%02d'), '    ', num2str(omega(i), '% 5.4g'), '  Hz'];
    end
end

gui_param.solver.flutter.MSELECT = select_list(modelist, 'Modal Base: ', 'Mode Selection');
fprintf(1, '\n - List of selected modes for MSELECT: \n');
disp(gui_param.solver.flutter.MSELECT)

save_gui_params('neocass_gui_param.mat', gui_param);



% --- Executes on button press in pushbutton5_fmodes.
function pushbutton5_fmodes_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5_fmodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

lmod = str2double(get(handles.lmodes,'String'));
modelist = cell(lmod,1);
global beam_model
if isfield(beam_model, 'Res'),
    if isfield(beam_model.Res, 'Omega'),
        omega = beam_model.Res.Omega/2/pi;
        lomega = length(omega);
    end
end

for i = 1:lmod,
    if ~exist('omega', 'var') || i > lomega,
        modelist{i} = ['mode ', num2str(i, '%02d'), '         '];
    else
        modelist{i} = ['mode ', num2str(i, '%02d'), '    ', num2str(omega(i), '% 5.4g'), '  Hz'];
    end
end

gui_param.solver.flutter.FMODES = select_list(modelist, 'Mode Tracking: ', 'Mode Selection');
fprintf(1, '\n - List of selected modes for FMODES: \n');
disp(gui_param.solver.flutter.FMODES)

save_gui_params('neocass_gui_param.mat', gui_param);



% --- Executes on button press in radiobutton4_modalanalysis.
function radiobutton4_modalanalysis_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4_modalanalysis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4_modalanalysis
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

    gui_param.solver.EIG_AV = true;
    gui_param.solver.FLUTT_AV = false;
    gui_param.solver.STATIC_AERO_AV = false;
    gui_param.solver.FLUTT_ENV = false;
    analysis_select(hObject, false, true, false, false)
    
    if get(handles.popupmenurm,'Value') == 1 || get(handles.popupmenurm,'Value') == 2,
        set(handles.popupmenuid,'Enable','Off');
        set(handles.popupmenudof,'Enable','Off');
        set(handles.text5,'Enable','Off');
        set(handles.text6,'Enable','Off');
    elseif get(handles.popupmenurm,'Value') == 3,
        set(handles.popupmenuid,'Enable','On');
        set(handles.popupmenudof,'Enable','On');
        set(handles.text5,'Enable','On');
        set(handles.text6,'Enable','On');
    end
    
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes on button press in pushbutton_ok.
function pushbutton_ok_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ok (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); delete(fullfile(path, 'neocass_gui_param_backup.mat'));
close(handles.figure1);

% --- Executes on button press in pushbutton_apply.
function pushbutton_apply_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_apply (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
save_gui_params('neocass_gui_param_backup.mat', gui_param);


% --- Executes on button press in pushbutton_cancel.
function pushbutton_cancel_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param_backup = load(fullfile(path, 'neocass_gui_param_backup.mat'));
save_gui_params('neocass_gui_param.mat', gui_param_backup);
delete(fullfile(path, 'neocass_gui_param_backup.mat'));
close(handles.figure1);


% % --- Executes on button press in typical_maneuver.
% function typical_maneuver_Callback(hObject, eventdata, handles)
% % hObject    handle to typical_maneuver (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of typical_maneuver
% set(hObject, 'Value', true);
% set(handles.generic_maneuver, 'Value', false);
% 
% guidata(hObject, handles);
% 
% 
% % --- Executes during object creation, after setting all properties.
% function typical_maneuver_CreateFcn(hObject, eventdata, handles)
% set(hObject, 'Value', true);



% % --- Executes on button press in generic_maneuver.
% function generic_maneuver_Callback(hObject, eventdata, handles)
% % hObject    handle to generic_maneuver (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of generic_maneuver
% set(hObject, 'Value', true);
% set(handles.typical_maneuver, 'Value', false);
% 
% guidata(hObject, handles);
% 
% 
% % --- Executes during object creation, after setting all properties.
% function generic_maneuver_CreateFcn(hObject, eventdata, handles)
% set(hObject, 'Value', false);




function nstates_Callback(hObject, eventdata, handles)
% hObject    handle to nstates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nstates as text
%        str2double(get(hObject,'String')) returns contents of nstates as a double
%
% Uses number of flight conditions recovered from states.xml files
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
nstates = str2double(get(hObject,'String'));

gui_param.solver.aeros.NSTATES = nstates;

if nstates > 0 && get(handles.radiobuttontrim,'Value') && gui_param.solver.aeros.CS
    
    set(handles.pushbuttonselectstates,'Enable','On');
    
    % State matrix
    gui_param.solver.aeros.STATE_MAT = cell(nstates,length(gui_param.solver.aeros.STATE_MAT));
    
else
    
    set(handles.pushbuttonselectstates,'Enable','Off');
    
end
    

fprintf(1, '\n - Number of states: %d. \n', gui_param.solver.aeros.NSTATES);

save_gui_params('neocass_gui_param.mat', gui_param);



% --- Executes during object creation, after setting all properties.
function nstates_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nstates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end




% --- Executes on button press in pushbuttonselectstates.
function pushbuttonselectstates_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonselectstates (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
% Select flight conditions to be retained for calculations
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

nstates = gui_param.solver.aeros.NSTATES;
trim_param = gui_param.solver.aeros.TRIM_LABELS;

state_mat = UserManeuver(nstates, trim_param);

% if get(handles.typical_maneuver, 'Value') == true,
%     state_mat = TypicalManeuver(nstates, trim_param);
% elseif get(handles.generic_maneuver, 'Value') == true,
%     state_mat = CustomManeuver(nstates, trim_param);
% end

if ~isempty(state_mat)  % values contains retained flight conditions
    gui_param.solver.aeros.STATE_MAT = state_mat;
    nstates = size(state_mat, 1);
    gui_param.solver.aeros.NSTATES = nstates;
else
    nstates = 0;
    set(handles.nstates, 'String', num2str(nstates));
    set(handles.pushbuttonselectstates,'Enable','Off');
    gui_param.solver.aeros.NSTATES = nstates;
end

TRIMprintf(state_mat);

save_gui_params('neocass_gui_param.mat', gui_param);



% --- Executes on button press in setCS.
function setCS_Callback(hObject, eventdata, handles)
% hObject    handle to setCS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

trim_param = TRIMlabels;
CSsel = select_list(trim_param(17:22,2), 'Control Surfaces:', 'Selection List');

if ~isempty(CSsel)
    
    InotCS = [false(1,16) ~ismember((1:6), CSsel)];
    trim_param(InotCS',:) = [];
    
    % Trim labels
    gui_param.solver.aeros.TRIM_LABELS = trim_param;
    % State matrix
    gui_param.solver.aeros.STATE_MAT = cell(str2double(get(handles.nstates,'String')),length(trim_param));
    gui_param.solver.aeros.CS = true;
    
    if str2double(get(handles.nstates,'String')) > 0 && get(handles.radiobuttontrim,'Value')
        set(handles.pushbuttonselectstates,'Enable','On');
    else
        set(handles.pushbuttonselectstates,'Enable','Off');
    end
    
end

save_gui_params('neocass_gui_param.mat', gui_param);



