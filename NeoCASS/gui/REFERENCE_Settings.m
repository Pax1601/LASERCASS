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

function varargout = REFERENCE_Settings(varargin)
% REFERENCE_Settings M-file for REFERENCE_Settings.fig
%      REFERENCE_Settings, by itself, creates a new REFERENCE_Settings or raises the existing
%      singleton*.
%
%      H = REFERENCE_Settings returns the handle to a new REFERENCE_Settings or the handle to
%      the existing singleton*.
%
%      REFERENCE_Settings('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REFERENCE_Settings.M with the given input arguments.
%
%      REFERENCE_Settings('Property','Value',...) creates a new REFERENCE_Settings or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before REFERENCE_Settings_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to REFERENCE_Settings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2002-2003 The MathWorks, Inc.

% Edit the above text to modify the response to help REFERENCE_Settings

% Last Modified by GUIDE v2.5 03-Apr-2009 14:30:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @REFERENCE_Settings_OpeningFcn, ...
                   'gui_OutputFcn',  @REFERENCE_Settings_OutputFcn, ...
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


% --- Executes just before REFERENCE_Settings is made visible.
function REFERENCE_Settings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to REFERENCE_Settings (see VARARGIN)

% Choose default command line output for REFERENCE_Settings
handles.output = hObject;

% Update handles structure
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
save_gui_params('neocass_gui_param_backup.mat', gui_param);

init_reference(handles, gui_param)

guidata(hObject, handles);

% UIWAIT makes REFERENCE_Settings wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.
function varargout = REFERENCE_Settings_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function editcref_Callback(hObject, eventdata, handles)
% hObject    handle to editcref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editcref as text
%        str2double(get(hObject,'String')) returns contents of editcref as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.solver.aero.CREF = abs(str2double(get(hObject,'String')));
fprintf(1, '\n - Reference length: %g.\n', gui_param.solver.aero.CREF);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function editcref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editcref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function editbref_Callback(hObject, eventdata, handles)
% hObject    handle to editbref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editbref as text
%        str2double(get(hObject,'String')) returns contents of editbref as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.solver.aero.BREF = abs(str2double(get(hObject,'String')));
fprintf(1, '\n - Reference span: %g.\n', gui_param.solver.aero.BREF);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function editbref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editbref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function editsref_Callback(hObject, eventdata, handles)
% hObject    handle to editsref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editsref as text
%        str2double(get(hObject,'String')) returns contents of editsref as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

gui_param.solver.aero.SREF = abs(str2double(get(hObject,'String')));
fprintf(1, '\n - Reference surface: %g.\n', gui_param.solver.aero.SREF);

save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function editsref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editsref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in popupmenuvs.
function popupmenuvs_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuvs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenuvs contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuvs
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

contents = get(hObject,'String');
contents{get(hObject,'Value')};

value = str2num(strtok(contents{get(hObject,'Value')}));
gui_param.solver.aero.SXZ = value;

switch value
  case 0
    fprintf(1, '\n - Full model used.\n');
  case 1
    fprintf(1, '\n - Vertical simmetry enabled.\n');
  case -1
    fprintf(1, '\n - Vertical anti-simmetry enabled.\n');
end

save_gui_params('neocass_gui_param.mat', gui_param);

% --- Executes during object creation, after setting all properties.
function popupmenuvs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuvs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in popupmenuhs.
function popupmenuhs_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuhs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenuhs contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuhs
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

contents = get(hObject,'String');
contents{get(hObject,'Value')};

value = str2num(strtok(contents{get(hObject,'Value')}));
gui_param.solver.aero.SXY = value;

switch value
  case 0
    fprintf(1, '\n - No ground effect.\n');
    set(handles.editheight,'Enable','Off');
    gui_param.solver.aero.HEIGHT = 0.0;
  case 1
    fprintf(1, '\n - Horizontal simmetry enabled.\n');
    set(handles.editheight,'Enable','On');
    gui_param.solver.aero.HEIGHT = 0.0;
end

save_gui_params('neocass_gui_param.mat', gui_param);

% --- Executes during object creation, after setting all properties.
function popupmenuhs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuhs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in popupmenukn.
function popupmenukn_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenukn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenukn contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenukn
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

contents = get(hObject,'String');
contents{get(hObject,'Value')};

value = str2num(strtok(contents{get(hObject,'Value')}));
gui_param.solver.aero.DLM_ORDER = value;
fprintf(1, '\n - DLM kernel order: %d.\n', gui_param.solver.aero.DLM_ORDER);

save_gui_params('neocass_gui_param.mat', gui_param);

% --- Executes during object creation, after setting all properties.
function popupmenukn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenukn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end





function editheight_Callback(hObject, eventdata, handles)
% hObject    handle to editheight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editheight as text
%        str2double(get(hObject,'String')) returns contents of editheight as a double
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));

height = abs(str2double(get(hObject,'String')));

gui_param.solver.aero.HEIGHT = height;

fprintf(1, '\n - Height from the ground: %g.\n', gui_param.solver.aero.HEIGHT);
save_gui_params('neocass_gui_param.mat', gui_param);


% --- Executes during object creation, after setting all properties.
function editheight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editheight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pushbutton_ok.
function Ok_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ok (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); delete(fullfile(path, 'neocass_gui_param_backup.mat'));
close(handles.figure1);

% --- Executes on button press in pushbutton_apply.
function Apply_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_apply (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
save_gui_params('neocass_gui_param_backup.mat', gui_param);


% --- Executes on button press in pushbutton_cancel.
function Cancel_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path = neoguiscratchpath(); gui_param_backup = load(fullfile(path, 'neocass_gui_param_backup.mat'));
save_gui_params('neocass_gui_param.mat', gui_param_backup);
delete(fullfile(path, 'neocass_gui_param_backup.mat'));
close(handles.figure1);


